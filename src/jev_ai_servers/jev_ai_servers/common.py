"""Pure helpers shared by jev_server and llm_server (no rclpy): keys, budgets, circuit breaker,
the asyncio bridge, and record/replay cassettes."""
from __future__ import annotations

import asyncio
import concurrent.futures
import hashlib
import json
import math
import os
import threading
from collections import deque
from pathlib import Path


def read_key(env: str, key_file: str = '') -> str:
    """API keys come from a file path (k8s Secret mount) or the environment, never from params."""
    if key_file:
        return Path(key_file).read_text().strip()
    return os.environ.get(env, '').strip() if env else ''


class Budget:
    """Sliding-window request limit (§9): requests/s (0 disables it). Tokens/min and USD/hour are
    tracked for diagnostics only; there is no token or spending cap (prototype)."""

    def __init__(self, max_rps: float = 50):
        self.max_rps = max_rps
        self._req: deque[float] = deque()
        self._tok: deque[tuple[float, float]] = deque()
        self._usd: deque[tuple[float, float]] = deque()

    def _trim(self, now: float) -> None:
        while self._req and now - self._req[0] >= 1.0:
            self._req.popleft()
        while self._tok and now - self._tok[0][0] >= 60.0:
            self._tok.popleft()
        while self._usd and now - self._usd[0][0] >= 3600.0:
            self._usd.popleft()

    def admit(self, now: float) -> str:
        """'' if a request may start now (and counts it), else the reason it may not."""
        self._trim(now)
        if self.max_rps and len(self._req) >= self.max_rps:
            return 'budget: max_rps'
        self._req.append(now)
        return ''

    def record(self, now: float, tokens: float, usd: float) -> None:
        # Fail closed: one NaN would make every later `>= max` comparison False for a minute.
        tokens = tokens if math.isfinite(tokens) and tokens >= 0 else math.inf
        self._tok.append((now, tokens))
        self._usd.append((now, usd))

    @property
    def tokens_last_min(self) -> float:
        return sum(t for _, t in self._tok)

    @property
    def usd_last_hour(self) -> float:
        return sum(u for _, u in self._usd)


class CircuitBreaker:
    """Opens after ``threshold`` consecutive failures; after ``cooldown_s`` lets one probe through
    (half-open); a success closes it. While open, servers reject fast and engines degrade (§9)."""

    def __init__(self, threshold: int = 5, cooldown_s: float = 30.0):
        self.threshold, self.cooldown_s = threshold, cooldown_s
        self.failures = 0
        self.opened_at: float | None = None

    def state(self, now: float) -> str:
        if self.opened_at is None:
            return 'closed'
        return 'half_open' if now - self.opened_at >= self.cooldown_s else 'open'

    def allow(self, now: float) -> bool:
        return self.state(now) != 'open'

    def success(self) -> None:
        self.failures, self.opened_at = 0, None

    def failure(self, now: float) -> None:
        self.failures += 1
        if self.failures >= self.threshold or self.opened_at is not None:
            self.opened_at = now          # a failed half-open probe re-opens


class AsyncLoop:
    """A dedicated asyncio thread (§8 threading): backends are async, ROS callbacks are not."""

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.stopped = False
        self._thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self._thread.start()

    def submit(self, coro) -> concurrent.futures.Future:
        """``future.cancel()`` cancels the asyncio task, which aborts the backend's request."""
        if self.stopped:                         # shutting down: fail now, not at the deadline
            coro.close()
            raise RuntimeError('server shutting down')
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def run(self, coro, timeout: float | None = None):
        fut = self.submit(coro)
        try:
            return fut.result(timeout)
        except concurrent.futures.TimeoutError:
            fut.cancel()
            raise TimeoutError(f'timed out after {timeout:g} s') from None

    def stop(self) -> None:
        """Cancel in-flight backend calls (their waiters return at once), then stop the loop."""
        self.stopped = True

        def cancel_all():
            for task in asyncio.all_tasks(self.loop):
                task.cancel()
            self.loop.call_soon(self.loop.stop)      # after the cancellations have run
        self.loop.call_soon_threadsafe(cancel_all)
        self._thread.join(timeout=2)


# --- record / replay cassettes ------------------------------------------------------------------

def cassette_key(*parts) -> str:
    return hashlib.sha256(json.dumps(parts, sort_keys=True, default=str).encode()).hexdigest()


def cassette_append(path: str, key: str, value: dict) -> None:
    """One JSON object per line: {"key": ..., "value": ...}. Written when ``record_path`` is set;
    the ``replay`` backends read it back (last entry per key wins)."""
    with open(path, 'a') as f:
        f.write(json.dumps({'key': key, 'value': value}) + '\n')


def cassette_load(path: str) -> dict[str, dict]:
    out = {}
    with open(path) as f:
        for line in f:
            if line.strip():
                row = json.loads(line)
                out[row['key']] = row['value']
    return out
