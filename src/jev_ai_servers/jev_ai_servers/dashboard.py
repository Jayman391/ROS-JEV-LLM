"""dashboard (A5, A5.5): live decision dashboard over HTTP (FRAMEWORK §3, §10, §12).

Subscribes ``decisions`` (DecisionRecord), ``labels`` (Label) and ``/diagnostics``, keeps the
last ``max_records`` records, and serves ``jev_ai/viz/dashboard.html`` at ``/`` and the page's
data at ``/api/state``: recent records, latest diagnostics per status name, and
``metrics.compute`` over everything kept (cached ~2 s). Read-only and without auth, so ``host``
defaults to 127.0.0.1 (in a container, bind 0.0.0.0 and publish the port on the host's loopback).
Requests whose Host header is not in ``allowed_hosts`` get 403, which stops DNS rebinding.

    ros2 run jev_ai_servers dashboard --ros-args -p port:=8080 -p records_path:=decisions.jsonl
"""
from __future__ import annotations

import gzip
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from jev_ai_msgs.msg import DecisionRecord, Label
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py import message_to_ordereddict

from jev_ai.core import metrics
from jev_ai.viz import report

METRICS_TTL_S = 2.0
LIVE_RECENT = 200          # records per poll: the page polls every second


def to_dict(msg) -> dict:
    return metrics.json_safe(message_to_ordereddict(msg))


class Dashboard(Node):
    def __init__(self, **kwargs):
        super().__init__('dashboard', **kwargs)
        param = lambda name, value: self.declare_parameter(name, value).value   # noqa: E731
        host, port = param('host', '127.0.0.1'), param('port', 8080)
        # '*' allows any Host (only behind a trusted proxy); add the robot's name/IP for LAN use
        self.allowed_hosts = {h.strip().lower() for h in
                              param('allowed_hosts', 'localhost,127.0.0.1,::1').split(',')}
        self.records = deque(maxlen=param('max_records', 5000))
        self.labels = deque(maxlen=self.records.maxlen)
        path = param('records_path', '')
        if path:
            self.records.extend(report.read_jsonl(path))
            self.get_logger().info(f'preloaded {len(self.records)} records from {path}')
        self.diagnostics: dict[str, dict] = {}
        self.lock = threading.Lock()
        self._metrics, self._metrics_t, self._dirty = None, 0.0, True

        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(DecisionRecord, 'decisions', self._on_record, qos)
        self.create_subscription(Label, 'labels', self._on_label, qos)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)

        self.page = report.render().encode()
        self.httpd = ThreadingHTTPServer((host, port), _handler(self))
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]          # the real one when port:=0
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'dashboard on http://{host}:{self.port}/')

    # --- subscriptions --------------------------------------------------------------------------

    def _on_record(self, msg: DecisionRecord) -> None:
        r = to_dict(msg)
        with self.lock:
            self.records.append(r)
            self._dirty = True

    def _on_label(self, msg: Label) -> None:
        x = to_dict(msg)
        with self.lock:
            self.labels.append(x)
            self._dirty = True

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            for s in msg.status:
                level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
                self.diagnostics[s.name] = {
                    'name': s.name, 'level': level, 'message': s.message,
                    'hardware_id': s.hardware_id, 'stamp': stamp,
                    'values': {kv.key: kv.value for kv in s.values}}

    # --- http -----------------------------------------------------------------------------------

    def state(self) -> dict:
        with self.lock:
            records, labels = list(self.records), list(self.labels)
            diagnostics = dict(self.diagnostics)
            stale = self._dirty and time.monotonic() - self._metrics_t >= METRICS_TTL_S
            if stale:
                self._dirty, self._metrics_t = False, time.monotonic()
        if stale or self._metrics is None:
            # ponytail: two requests may both recompute after the TTL; harmless
            self._metrics = metrics.compute(records, labels or None)
        return report.payload(records, self._metrics, diagnostics, mode='live', n=LIVE_RECENT)

    def destroy_node(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        super().destroy_node()


def _host_ok(header: str, allowed: set[str]) -> bool:
    """Host header without port ('[::1]:8080' -> '::1') is in ``allowed``, or '*' is."""
    h = header.strip().lower()
    h = h[1:h.find(']')] if h.startswith('[') else h.rsplit(':', 1)[0] if h.count(':') == 1 else h
    return '*' in allowed or h in allowed


def _handler(node: Dashboard):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = self.path.split('?', 1)[0]
            if not _host_ok(self.headers.get('Host', ''), node.allowed_hosts):
                self._send(403, 'text/plain', b'host not allowed (dashboard allowed_hosts)')
            elif route in ('/', '/index.html'):
                self._send(200, 'text/html; charset=utf-8', node.page)
            elif route == '/api/state':
                body = json.dumps(node.state(), separators=(',', ':')).encode()
                self._send(200, 'application/json', body, gzip_ok=True)
            else:
                self._send(404, 'text/plain', b'not found')

        def _send(self, code: int, ctype: str, body: bytes, gzip_ok: bool = False):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            # ~10x smaller: the page polls every second, often over robot Wi-Fi
            if gzip_ok and 'gzip' in self.headers.get('Accept-Encoding', ''):
                body = gzip.compress(body, 5)
                self.send_header('Content-Encoding', 'gzip')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):     # keep the ROS log clean
            pass

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
