"""ros2 jev status: keys, endpoints, backends, models, breaker and budget state (§0, §9)."""
import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from jev_ai_msgs.action import Reason
from jev_ai_msgs.srv import JevDecide
from rclpy.action import ActionClient
from ros2cli.verb import VerbExtension

LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
SHOWN = ('backend', 'model', 'breaker', 'calls', 'errors', 'latency_ms_p50', 'latency_ms_p95',
         'usd_last_hour', 'last_error')


class StatusVerb(VerbExtension):
    """Show jev_ai server health."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--jev', default='/jev/decide', help='JevDecide service')
        parser.add_argument('--llm', default='/llm/reason', help='Reason action')
        parser.add_argument('--wait', type=float, default=2.5, help='seconds to listen')

    def main(self, *, args):
        rclpy.init()
        node = rclpy.create_node('ros2jev_status')
        seen = {}
        node.create_subscription(DiagnosticArray, '/diagnostics',
                                 lambda m: seen.update({s.name: s for s in m.status}), 10)
        jev = node.create_client(JevDecide, args.jev)
        llm = ActionClient(node, Reason, args.llm)
        end = time.monotonic() + args.wait
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print('keys:')
        for k in ('TYPESAFE_API_KEY', 'ANTHROPIC_API_KEY', 'OPENAI_API_KEY'):
            print(f'  {k:<18} {"set" if os.environ.get(k) else "not set"}')
        print('endpoints:')
        print(f'  {args.jev:<18} {"ready" if jev.service_is_ready() else "NOT AVAILABLE"}')
        print(f'  {args.llm:<18} {"ready" if llm.server_is_ready() else "NOT AVAILABLE"}')
        servers = {n: s for n, s in seen.items() if s.name.endswith(('jev_server', 'llm_server'))}
        print('servers:' if servers else 'servers: no jev_ai diagnostics on /diagnostics')
        for name, s in sorted(servers.items()):
            values = {kv.key: kv.value for kv in s.values}
            print(f'  {name}  [{LEVEL.get(s.level[0] if isinstance(s.level, bytes) else s.level, "?")}]'
                  f' {s.message}')
            for k in SHOWN:
                if values.get(k, '') != '':
                    print(f'    {k:<18} {values[k]}')
        ok = jev.service_is_ready() and all(s.level in (0, b'\x00') for s in servers.values())
        node.destroy_node()
        rclpy.shutdown()
        return 0 if ok else 1
