"""Lifecycle base shared by jev_server and llm_server (§8, §9).

Owns everything that is not endpoint-specific:
- parameters (generic ones declared here; backend ones read by prefix, e.g. ``mock.seed``),
  read-only after launch;
- backend loading by name (entry points), API keys (env or file, never params), configure-time
  ``check()``, and "no silent mock": a backend with ``key_env`` fails to configure without a key;
- budgets, the circuit breaker, deadlines and retries around every backend call (``call()``);
- ``/diagnostics`` at 1 Hz; optional record cassette (``record_path``) for the replay backends.

Subclasses set GROUP / BUILTIN / DEFAULT_BACKEND and implement ``setup_endpoint()`` (create the
service or action server; requests arriving while inactive are rejected by ``call()``).
"""
from __future__ import annotations

import concurrent.futures
import statistics
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

from jev_ai.backends import BackendError, load_backend
from jev_ai_servers.common import AsyncLoop, Budget, CircuitBreaker, cassette_append, read_key

PARAMS = {                       # generic parameters (§8, §9, §14); backend ones go under <backend>.
    'backend': '',
    'model': '',                 # '' = the backend's default
    'api_key_file': '',
    'timeout_s': 1.0,
    'max_retries': 1,
    'max_rps': 50.0,
    'max_tokens_per_min': 200_000.0,
    'breaker_errors': 5,
    'breaker_cooldown_s': 30.0,
    'record_path': '',           # append every successful answer to this cassette (JSONL)
    'autostart': True,           # configure + activate on startup (no lifecycle manager needed)
}

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


class BackendServer(LifecycleNode):
    GROUP = ''
    BUILTIN: dict[str, str] = {}
    DEFAULT_BACKEND = ''
    DEFAULTS: dict = {}          # per-server overrides of PARAMS defaults (e.g. timeout_s)

    def __init__(self, name: str, **kwargs):
        super().__init__(name, automatically_declare_parameters_from_overrides=True, **kwargs)
        for k, v in {**PARAMS, 'backend': self.DEFAULT_BACKEND, **self.DEFAULTS}.items():
            if not self.has_parameter(k):
                self.declare_parameter(k, v)
        # Launch-time config only. The parameter services are open to the whole ROS graph, and
        # record_path / api_key_file / <backend>.base_url set at runtime would let any peer write
        # files or ship a file's contents to its own host as an "API key".
        self.add_on_set_parameters_callback(lambda _: SetParametersResult(
            successful=False, reason=f'{name} parameters are read-only; set them at launch'))
        self.cb_group = ReentrantCallbackGroup()
        self.loop = AsyncLoop()
        self.backend = None
        self.active = False
        self.calls = self.errors = 0
        self.last_error = ''
        self.latencies: deque[float] = deque(maxlen=200)
        self.budget = Budget()
        self.breaker = CircuitBreaker()
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._publish_diagnostics)
        if self.p('autostart'):
            self._boot_timer = self.create_timer(0.0, self._boot)

    def p(self, name: str):
        return self.get_parameter(name).value

    # --- lifecycle -----------------------------------------------------------------------------

    def _boot(self) -> None:
        self._boot_timer.cancel()
        self.destroy_timer(self._boot_timer)
        if self.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            self.trigger_activate()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        name = self.p('backend')
        try:
            cls = load_backend(self.GROUP, name, self.BUILTIN)
            key = read_key(cls.key_env, self.p('api_key_file'))
            if cls.key_env and not key:
                raise BackendError(f'backend {name!r} needs an API key: set {cls.key_env} or '
                                   'api_key_file (use backend:=mock for keyless runs)')
            params = {k: v.value for k, v in self.get_parameters_by_prefix(name).items()}
            self.backend = cls(params, key)
            self.backend.node = self
            self.loop.run(self.backend.check(), timeout=max(5.0, self.p('timeout_s')))
        except Exception as e:                            # noqa: BLE001 - any failure = FAILURE
            self.get_logger().error(f'configure failed: {e}')
            self.last_error = str(e)
            self.backend = None
            return TransitionCallbackReturn.FAILURE
        self.budget = Budget(self.p('max_rps'), self.p('max_tokens_per_min'))
        self.breaker = CircuitBreaker(self.p('breaker_errors'), self.p('breaker_cooldown_s'))
        self.setup_endpoint()
        self.get_logger().info(f'configured: backend={name} model={self.p("model") or "default"}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.teardown_endpoint()
        if self.backend:
            self.loop.run(self.backend.close(), timeout=5.0)
        self.backend = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.active = False
        return TransitionCallbackReturn.SUCCESS

    def setup_endpoint(self) -> None:
        raise NotImplementedError

    def teardown_endpoint(self) -> None:
        pass

    # --- guarded backend call -------------------------------------------------------------------

    def call(self, make_coro, *, timeout_s: float | None = None, cancelled=lambda: False,
             account=lambda result: (0, 0.0)):
        """Run one backend call under the budget, breaker, deadline (wall time) and retries.

        ``make_coro()`` builds a fresh coroutine per attempt; ``account(result)`` returns
        (tokens, usd) for the budget. Returns (result, error, latency_ms); exactly one of
        result/error is set. 'cancelled' is not counted as a backend failure."""
        t0 = time.monotonic()
        if not self.active or self.backend is None:
            return None, 'server not active', 0.0
        if not self.breaker.allow(t0):
            return None, 'circuit breaker open', 0.0
        refused = self.budget.admit(t0)
        if refused:
            return None, refused, 0.0
        deadline = t0 + (timeout_s if timeout_s is not None else self.p('timeout_s'))
        self.calls += 1
        attempts = 1 + max(0, self.p('max_retries'))
        error = ''
        for _ in range(attempts):
            fut = self.loop.submit(make_coro())
            try:
                result = self._wait(fut, deadline, cancelled)
            except _Cancelled:
                return None, 'cancelled', (time.monotonic() - t0) * 1e3
            except BackendError as e:
                error = str(e)
                if e.retryable and time.monotonic() < deadline:
                    continue
                break
            except TimeoutError:
                error = 'deadline exceeded'
                break
            except Exception as e:                       # noqa: BLE001 - a plugin bug is an error
                error = f'{type(e).__name__}: {e}'
                break
            ms = (time.monotonic() - t0) * 1e3
            self.latencies.append(ms)
            self.breaker.success()
            tokens, usd = account(result)
            self.budget.record(time.monotonic(), tokens, usd)
            return result, '', ms
        self.errors += 1
        self.last_error = error
        self.breaker.failure(time.monotonic())
        return None, error, (time.monotonic() - t0) * 1e3

    @staticmethod
    def _wait(fut: concurrent.futures.Future, deadline: float, cancelled):
        while True:
            if cancelled():
                fut.cancel()
                raise _Cancelled
            left = deadline - time.monotonic()
            if left <= 0:
                fut.cancel()
                raise TimeoutError
            try:
                return fut.result(timeout=min(0.05, left))
            except concurrent.futures.TimeoutError:
                continue

    def record(self, key: str, value: dict) -> None:
        if self.p('record_path'):
            cassette_append(self.p('record_path'), key, value)

    # --- diagnostics ----------------------------------------------------------------------------

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        breaker = self.breaker.state(now)
        if self.backend is None:
            level, msg = ERROR, f'not configured: {self.last_error}' if self.last_error else \
                'not configured'
        elif breaker != 'closed':
            level, msg = ERROR if breaker == 'open' else WARN, f'circuit breaker {breaker}'
        elif not self.active:
            level, msg = WARN, 'inactive'
        else:
            level, msg = OK, 'ok'
        lat = sorted(self.latencies)
        values = {
            'backend': self.p('backend'), 'model': self.p('model'), 'active': self.active,
            'calls': self.calls, 'errors': self.errors, 'last_error': self.last_error,
            'breaker': breaker,
            'latency_ms_p50': round(statistics.median(lat), 1) if lat else '',
            'latency_ms_p95': round(lat[int(0.95 * (len(lat) - 1))], 1) if lat else '',
            'tokens_last_min': round(self.budget.tokens_last_min),
            'usd_last_hour': round(self.budget.usd_last_hour, 4),
        }
        status = DiagnosticStatus(level=level, name=self.get_fully_qualified_name(), message=msg,
                                  hardware_id=self.p('backend'),
                                  values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        arr = DiagnosticArray(status=[status])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(arr)


class _Cancelled(Exception):
    pass


def spin(node_cls, args=None) -> None:
    """Entry point body for a server executable."""
    rclpy.init(args=args)
    node = node_cls()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.loop.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
