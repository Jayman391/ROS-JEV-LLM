"""line_supervisor: the line's jev_ai supervisor (DECISION_MODEL §3).

Judgement (question sets, one item at a time each):
- ``release``: which product next when the feeder is free (acted on when final).
- ``disruption``: how to ride out an arm's fault or a station backlog (a per-arm setting, acted
  on at every phase: wait_recover / reroute / slow_line / pause_station).
- ``quality_watch``: what to do about an arm QA keeps flagging (monitor / inspect_tool /
  order_tool_change, as directives).
Rules: routing (scenario.route), hand-off reroutes, routing rework redos, release cadence.
"""
from __future__ import annotations

from collections import Counter, defaultdict, deque

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from line_interfaces.msg import ArmStatus, LineState
from line_interfaces.srv import Release, Route
from rclpy.node import Node

from assembly_line import scenario
from assembly_line.arm_agent import wear_word
from assembly_line.scenario import mmss
from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run

RELEASE_RETRY_S = 5.0
QA_RECORD = 10
FOCUS_TIMEOUT_S = 120.0     # sim s; > T_wait (40.5) with room for a Jev outage


def schedule(p: dict, products, shipped, on_line, t: float) -> list[dict]:
    """Per product: left to start, shipped, on the line, next due time and its urgency."""
    due = scenario.orders(p)
    out = []
    for pr, n in zip(p['products'], p['order_counts']):
        s, w = shipped.get(pr, 0), on_line.get(pr, 0)
        nxt = due[pr][min(s + w, len(due[pr]) - 1)] if s + w < n else None
        urgency = '' if nxt is None else 'late' if nxt < t else \
            'due soon' if nxt - t < 240 else 'on time'
        out.append({'product': pr, 'total': int(n), 'shipped': s, 'on_line': w,
                    'left': max(0, int(n) - s - w), 'due': nxt, 'urgency': urgency})
    return out


def encode_release(p: dict, sched: list[dict], routes: dict[str, str], line: str,
                   t: float) -> dict:
    orders = '; '.join(
        f'{o["product"]}: nothing left to start ({o["shipped"]}/{o["total"]} shipped)'
        if o['left'] == 0 else
        f'{o["product"]}: {o["left"]} left to start, {o["on_line"]} on the line, '
        f'{o["shipped"]}/{o["total"]} shipped, next due at {mmss(o["due"])} ({o["urgency"]})'
        for o in sched)
    return {'orders': orders,
            'routes': '; '.join(f'{pr}: {r}' for pr, r in routes.items()),
            'line': line, 'time': f'{mmss(t)} of the {mmss(p["t_max_s"])} shift elapsed'}


def encode_disruption(p: dict, event: dict, arm: ArmStatus | None, alternatives: str, line: str,
                      orders: str, t: float) -> dict:
    name = event['arm']
    ago = t - event['t']
    n = event['count']
    if event['kind'] == 'jam':
        est = event.get('est_clear_s', 0)
        when = 'soon' if est < 30 else 'in a while' if est < 90 else 'not for a long time'
        ev = (f'{name} jammed {ago:.0f} s ago (fault {n} this shift); expected to clear {when} '
              f'(~{est:.0f} s)')
    elif event['kind'] == 'sensor_dropout':
        ev = (f'{name} lost its sensors {ago:.0f} s ago (fault {n} this shift); it cannot check '
              'grips, alignment or wear until they return')
    else:
        ev = (f'station S{event.get("station", "?")} backed up: its buffer has been full for '
              f'{ago:.0f} s; {name} is its busiest arm')
    me = scenario.arm(p, name)
    if arm is None:
        a = f'station S{me["station"]}, tools {", ".join(sorted(me["tools"]))}'
    else:
        w = 'unknown' if arm.wear_est != arm.wear_est else \
            f'{wear_word(arm.wear_est)} ({arm.wear_est:.0%} est.)'
        a = (f'station S{me["station"]}, tools {", ".join(sorted(me["tools"]))}; now '
             f'{arm.state}; wear {w}; {arm.queue} part(s) waiting for it; {arm.jams} jam(s) '
             'this shift')
    return {'event': ev, 'arm': a, 'alternatives': alternatives, 'line': line, 'orders': orders}


def encode_quality(p: dict, name: str, flag: dict, record, line_rate: float,
                   arm: ArmStatus | None, since_change_s: float | None) -> dict:
    me = scenario.arm(p, name)
    flagged = [k for k, f in record if f]
    kinds = Counter(flagged)
    rec = (f'{len(flagged)} of its last {len(record)} operations flagged' +
           (f' ({", ".join(f"{k} x{c}" for k, c in kinds.items())})' if kinds else '') +
           f'; line average {line_rate:.0%} flagged')
    if arm is None or arm.wear_est != arm.wear_est:
        tool = 'wear unknown'
    else:
        tool = f'wear {wear_word(arm.wear_est)} ({arm.wear_est:.0%} est.)'
    if arm is not None:
        tool += f', {arm.ops_since_change} operations'
    tool += ' since its last tool change' + \
        (f' ({mmss(since_change_s)} ago)' if since_change_s is not None else ' (none this shift)')
    return {'arm': f'{name} at S{me["station"]}: QA flagged its {flag["kind"]} '
                   f'({flag["reading"]:.2f}) on part {flag["part"]}',
            'qa_record': rec, 'tool': tool,
            'line': f'{arm.queue if arm else 0} part(s) waiting for {name}; a tool change takes '
                    f'~{p["tool_change_s"]:.0f} s'}


class Supervisor(Node):
    def __init__(self):
        super().__init__('line_supervisor')
        self.p = scenario.from_node(self)
        share = get_package_share_directory('assembly_line')
        catalog = self.declare_parameter('catalog', share + '/config/supervisor.yaml').value
        run_id = self.declare_parameter('run_id', '').value or None
        jev = self.declare_parameter('jev_service', '/jev/decide').value
        llm = self.declare_parameter('llm_action', '/line_supervisor/llm/reason').value
        self.recipes = scenario.recipes(self.p)
        self.names = [a['name'] for a in scenario.arms(self.p)]

        self.line: LineState | None = None
        self.arms: dict[str, ArmStatus] = {}
        self.response: dict[str, str] = {}          # disruption setting per arm
        self.excluded: dict[str, str] = {}          # arm -> why routes avoid it
        self.pause_until: dict[str, float] = {}
        self.events: dict[str, dict] = {}           # last fault/backlog event per arm
        self.faults: Counter = Counter()
        self.qa: dict[str, deque] = defaultdict(lambda: deque(maxlen=QA_RECORD))
        self.last_change: dict[str, float] = {}
        self.full_since: dict[int, float] = {}
        self.queues = {'disruption': deque(), 'quality_watch': deque()}
        self.focus = {'disruption': '', 'quality_watch': '', 'release': ''}
        self.focus_t: dict[str, float] = {}         # when each focus was set
        self.flags: dict[str, dict] = {}            # quality_watch: arm -> the flag in focus
        self.last_release_try = -1e9
        self.releases = 0
        self.last = {}                              # last final selection per set (diagnostics)

        self.create_subscription(LineState, '/line/state', self._line, 10)
        for n in self.names:
            self.create_subscription(ArmStatus, f'/{n}/status',
                                     lambda m: self.arms.__setitem__(m.arm, m), 10)
        self.release_cli = self.create_client(Release, '/line/release')
        self.route_cli = self.create_client(Route, '/line/route')
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.engine = DecisionEngine(self, catalog, encode_state=self.encode, mask=self.mask,
                                     require=self.require, on_decision=self.on_decision,
                                     current=self.current, jev=jev, llm=llm, run_id=run_id,
                                     agent='/line_supervisor', all_phases=True)
        self.hierarchy = Hierarchy(self, agent='/line_supervisor', on_report=self.on_report)
        self.create_timer(1.0, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- line picture ----------------------------------------------------------------------------

    def _line(self, m: LineState) -> None:
        self.line = m
        for pt in m.parts:                          # rework redos (and anything else) unrouted
            rest = list(pt.route[pt.next_op:])
            if '' in rest:
                i = pt.next_op + rest.index('')
                self.reroute(pt, i)

    def shipped(self) -> dict[str, int]:
        return dict(zip(self.line.products, self.line.shipped)) if self.line else {}

    def on_line(self) -> dict[str, int]:
        return Counter(pt.product for pt in self.line.parts) if self.line else Counter()

    def wip_cap(self) -> int:
        slow = any(r == 'slow_line' for r in self.response.values())
        return int(self.p['wip_slow'] if slow else self.p['wip_cap'])

    def load(self) -> dict[str, float]:
        return {n: float(a.queue) for n, a in self.arms.items()}

    def line_sentence(self) -> str:
        if self.line is None:
            return 'unknown'
        full = [f'S{i + 1}' if i < len(self.line.buffer) - 1 else 'QA'
                for i, b in enumerate(self.line.buffer) if b >= self.line.buffer_cap]
        down = [f'{n} {a.state}' for n, a in sorted(self.arms.items())
                if a.state == 'jammed' or a.dropout]
        lane = sum(pt.where == 'lane' for pt in self.line.parts)
        out = [f'WIP {len(self.line.parts)} of cap {self.wip_cap()}',
               f'full buffers: {", ".join(full)}' if full else 'no full buffers',
               f'in trouble: {", ".join(down)}' if down else 'every arm healthy',
               f'{lane} part(s) in the rework lane']
        if self.response:
            out.append('in force: ' + ', '.join(f'{n} {r}' for n, r in self.response.items()))
        return '; '.join(out)

    def orders_sentence(self) -> str:
        t = self.line.t if self.line else 0.0
        total = sum(self.p['order_counts'])
        shipped = sum(self.shipped().values())
        expected = total * min(1.0, t / (0.85 * self.p['t_max_s']))
        pace = 'behind schedule' if shipped < expected - 2 else 'on schedule'
        return (f'{pace}: {shipped} of {total} shipped ({expected:.0f} expected by now); '
                f'{mmss(t)} of {mmss(self.p["t_max_s"])} elapsed')

    def alternatives(self, name: str) -> str:
        """Where each op this arm can do could go instead (fasten has no alternative)."""
        me = scenario.arm(self.p, name)
        fasten = any(name in e.split('+') for _, e in scenario.capable(self.p, 'fasten'))
        out = []
        for kind in sorted((me['tools'] - {'hold'}) | ({'fasten'} if fasten else set())):
            alts = scenario.capable(self.p, kind, set(self.excluded) | {name})
            out.append(f'{kind} -> ' + (', '.join(
                f'{e} (S{s}{", may need a lap" if s < me["station"] else ""})' for s, e in alts)
                if alts else 'none'))
        return '; '.join(out)

    # --- rules -----------------------------------------------------------------------------------

    def reroute(self, pt, from_op: int, avoid: set[str] = frozenset()) -> bool:
        """Re-plan pt's ops from from_op with the routing rule, avoiding excluded arms."""
        entries, _ = scenario.route(self.p, list(pt.kinds[from_op:]), at=pt.stop,
                                    exclude=set(self.excluded) | set(avoid), load=self.load())
        if entries == list(pt.route[from_op:]):
            return True
        if '' in entries and avoid:
            return False                             # no alternative: keep the route
        if not self.route_cli.service_is_ready():
            return False
        self.route_cli.call_async(Route.Request(part_id=pt.id, from_op=from_op, route=entries)) \
            .add_done_callback(lambda f: f.result().success or self.get_logger().debug(
                f'route part {pt.id}: {f.result().message}'))
        return True

    def reroute_around(self, name: str) -> None:
        if self.line is None:
            return
        for pt in self.line.parts:
            rest = list(pt.route[pt.next_op:])
            if any(name in e.split('+') for e in rest):
                # the op in progress can't move: try from the next one
                if not self.reroute(pt, pt.next_op, {name}) and pt.next_op + 1 < pt.n_ops:
                    self.reroute(pt, pt.next_op + 1, {name})

    def tick(self) -> None:
        now = self.now()
        for q, since in list(self.focus_t.items()):    # a decision that never finalised
            if self.focus[q] and now - since > FOCUS_TIMEOUT_S:
                self.get_logger().warn(f'{q}: no final decision on {self.focus[q]}; moving on')
                self.focus[q] = ''
                if q in self.queues:
                    self.next_item(q)
        for name, until in list(self.pause_until.items()):
            if now >= until:
                del self.pause_until[name]
                self.excluded.pop(name, None)
                self.response.pop(name, None)
                self.hierarchy.direct('resume', target=f'/{name}', ttl_s=30)
        if self.line is not None:
            self.backlog(now)
            self.maybe_release(now)
        self.publish_status()

    def backlog(self, now: float) -> None:
        for i, b in enumerate(self.line.buffer[:-1]):          # stations, not QA
            s = i + 1
            if b < self.line.buffer_cap:
                self.full_since.pop(s, None)
                continue
            since = self.full_since.setdefault(s, now)
            if now - since >= self.p['backlog_s']:
                self.full_since[s] = now                       # at most once per backlog_s
                here = [n for n in self.names if scenario.arm(self.p, n)['station'] == s]
                name = max(here, key=lambda n: self.arms[n].queue if n in self.arms else 0)
                self.event(name, {'kind': 'backlog', 'station': s, 't': since})

    def maybe_release(self, now: float) -> None:
        if self.focus['release'] or not self.line.feeder_free or \
                now - self.last_release_try < RELEASE_RETRY_S or \
                len(self.line.parts) >= self.wip_cap() or \
                not any(o['left'] for o in self.schedule()):
            return
        self.last_release_try = now
        self.releases += 1
        self.focus['release'] = f'slot{self.releases}'
        self.focus_t['release'] = now
        self.engine.trigger('feeder_free', 'release', subject=self.focus['release'], fresh=True)

    def schedule(self) -> list[dict]:
        return schedule(self.p, self.line.products, self.shipped(), self.on_line(),
                        self.line.t if self.line else 0.0)

    def routes(self) -> dict[str, str]:
        return {pr: scenario.describe_route(self.p, pr, set(self.excluded))
                for pr in self.p['products']}

    # --- reports ---------------------------------------------------------------------------------

    def on_report(self, msg, payload: dict) -> None:
        name = msg.agent.strip('/')
        if msg.kind == 'fault':
            self.faults[name] += 1
            self.event(name, {'kind': payload.get('kind', 'fault'), 't': self.now(),
                              'est_clear_s': payload.get('est_clear_s', 0)})
        elif msg.kind == 'recovered':
            if self.response.get(name) in ('reroute', 'slow_line', 'wait_recover'):
                del self.response[name]
                if self.excluded.get(name) == 'reroute':
                    del self.excluded[name]
        elif msg.kind == 'hand_off':
            pt = next((x for x in (self.line.parts if self.line else [])
                       if x.id == payload.get('part')), None)
            if pt is not None and pt.next_op == payload.get('op'):
                self.reroute(pt, pt.next_op, {name})
        elif msg.kind == 'tool_change':
            self.last_change[name] = self.now()
        elif msg.kind == 'qa_result':
            for op in payload.get('ops', []):
                for a in op['arm'].split('+'):
                    self.qa[a].append((op['kind'], op['reading'] < self.p['ok_min']))
            w = payload.get('worst') or {}
            if w and w['reading'] < self.p['ok_min']:
                arm = w['arm'].split('+')[0]
                self.flags[arm] = dict(w, part=payload.get('part'))
                self.enqueue('quality_watch', arm)

    def event(self, name: str, ev: dict) -> None:
        self.events[name] = dict(ev, arm=name, count=self.faults[name] or 1)
        self.enqueue('disruption', name)

    def enqueue(self, qset: str, name: str) -> None:
        if name != self.focus[qset] and name not in self.queues[qset]:
            self.queues[qset].append(name)
        self.next_item(qset)

    def next_item(self, qset: str) -> None:
        if self.focus[qset] or not self.queues[qset]:
            return
        self.focus[qset] = self.queues[qset].popleft()
        self.focus_t[qset] = self.now()
        self.engine.trigger(qset, qset, subject=self.focus[qset], fresh=True)

    # --- jev_ai callbacks ------------------------------------------------------------------------

    def encode(self, qset: str) -> dict[str, str]:
        t = self.line.t if self.line else 0.0
        name = self.focus.get(qset, '')
        if qset == 'release':
            return encode_release(self.p, self.schedule() if self.line else [], self.routes(),
                                  self.line_sentence(), t)
        keys = self.engine.cat.question_sets[qset].state_keys
        if not name:
            return dict.fromkeys(keys, 'none')
        if qset == 'disruption':
            return encode_disruption(self.p, self.events[name], self.arms.get(name),
                                     self.alternatives(name), self.line_sentence(),
                                     self.orders_sentence(), self.now())
        rec = list(self.qa[name])
        allops = [f for d in self.qa.values() for _, f in d]
        rate = sum(allops) / len(allops) if allops else 0.0
        since = self.now() - self.last_change[name] if name in self.last_change else None
        return encode_quality(self.p, name, self.flags[name], rec, rate, self.arms.get(name),
                              since)

    def current(self, qset: str) -> dict[str, str]:
        if qset == 'release':
            return {'next_product': 'hold'}
        if qset == 'disruption':
            return {'response': self.response.get(self.focus['disruption'], 'wait_recover')}
        return {'escalate': 'monitor'}

    def mask(self, qset: str) -> dict[str, set[str]]:
        if qset != 'release' or self.line is None:
            return {}
        routes = self.routes()
        return {'next_product': {o['product'] for o in self.schedule()
                                 if o['left'] == 0 or routes[o['product']].startswith('blocked')}}

    def require(self, qset: str) -> dict[str, str]:
        if qset == 'release':
            if not self.focus['release'] or self.line is None or not self.line.feeder_free or \
                    len(self.line.parts) >= self.wip_cap() or \
                    len(self.mask('release')['next_product']) == len(self.p['products']):
                return {'next_product': 'hold'}
            return {}
        if not self.focus[qset]:
            return self.current(qset)                  # nothing in focus: a forced no-op
        return {}

    def on_decision(self, d: Decision) -> None:
        q = d.question_set
        name = self.focus.get(q, '')
        if q == 'disruption' and d.act and name:
            self.apply(name, d.selected['response'], d)
        if not d.final:
            return
        self.last[q] = (name, next(iter(d.selected.values())), d.phase)
        if q == 'release' and name:
            self.focus['release'] = ''
            if d.selected['next_product'] != 'hold':
                self.release(d.selected['next_product'])
        elif q == 'quality_watch' and name:
            e = d.selected['escalate']
            if e != 'monitor':
                self.get_logger().info(f'quality_watch {name}: {e} ({d.phase})')
                flag = self.flags.get(name, {})
                self.hierarchy.direct('inspect_tool' if e == 'inspect_tool' else 'tool_change',
                                      target=f'/{name}', ttl_s=120,
                                      payload={'reason': f'QA flagged its {flag.get("kind")} '
                                                         f'({flag.get("reading")})'})
        if q in self.queues and name:
            self.focus[q] = ''
            self.next_item(q)

    def apply(self, name: str, response: str, d: Decision) -> None:
        """A disruption response is a per-arm setting; re-applying it is harmless."""
        prev = self.response.get(name)
        if prev == response:
            return
        self.get_logger().info(f'disruption {name}: {response} ({d.phase})' +
                               (f': {d.rationale[:80]}' if d.rationale else ''))
        self.response[name] = response
        if self.excluded.get(name) == 'reroute':
            del self.excluded[name]
        if response == 'reroute':
            self.excluded[name] = 'reroute'
            self.reroute_around(name)
        elif response == 'pause_station':
            self.excluded[name] = 'pause'
            self.pause_until[name] = self.now() + self.p['pause_s']
            self.hierarchy.direct('tool_change', target=f'/{name}', ttl_s=60)
            self.hierarchy.direct('pause', target=f'/{name}', ttl_s=60,
                                  payload={'seconds': self.p['pause_s']})
            self.reroute_around(name)

    # --- acting ----------------------------------------------------------------------------------

    def release(self, product: str) -> None:
        entries, laps = scenario.route(self.p, self.recipes[product], at=0,
                                       exclude=set(self.excluded), load=self.load())
        req = Release.Request(product=product, route=entries)

        def done(f):
            r = f.result()
            if r.success:
                self.get_logger().info(f'released part {r.part_id} ({product}): '
                                       f'{" ".join(e or "-" for e in entries)}')
            else:
                self.get_logger().warn(f'release {product}: {r.message}')
        self.release_cli.call_async(req).add_done_callback(done)

    def publish_status(self) -> None:
        sched = self.schedule() if self.line else []
        s = DiagnosticStatus(name='line_supervisor', level=DiagnosticStatus.OK,
                             message=self.orders_sentence())
        s.values = [KeyValue(key='orders_left', value=', '.join(
                        f'{o["product"]} {o["left"] + o["on_line"]}' for o in sched)),
                    KeyValue(key='wip_cap', value=str(self.wip_cap())),
                    KeyValue(key='responses', value=', '.join(
                        f'{n} {r}' for n, r in self.response.items()) or 'none'),
                    KeyValue(key='excluded', value=', '.join(self.excluded) or 'none')]
        s.values += [KeyValue(key=f'last_{q}', value=f'{n} {c} ({ph})'.strip())
                     for q, (n, c, ph) in self.last.items()]
        self.diag.publish(DiagnosticArray(status=[s]))


def main(args=None):
    run(Supervisor, args, threads=1)
