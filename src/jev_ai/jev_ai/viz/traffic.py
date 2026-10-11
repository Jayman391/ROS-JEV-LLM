"""Hierarchy traffic (FRAMEWORK §3 Report/Directive): every Directive and Report the dashboard
sees, matched across topics.

A relay (a radio model, a bridge) republishes the same message on another topic: seeing it there
is a delivery, and the delay between the two sightings is that hop's latency. A relay endpoint is
an agent's namespace whose topic carries another agent's messages (``/uav1/directives`` carries
``/coordinator``'s); a message an endpoint should get but has not seen within ``GRACE_S`` is
dropped. Without a relay (one shared topic, as in toy_patrol) there is nothing to judge: ``sent``.

Pure Python: the dashboard node calls ``observe`` and serves ``snapshot`` as ``traffic``.
"""
from __future__ import annotations

from collections import OrderedDict

GRACE_S = 3.0          # wall s before a missing copy counts as dropped (relays add < 1 s)
BUCKETS = 60           # timeline columns over the kept messages' time span
RECENT = 300           # messages per snapshot (the page polls every second)
LANES = ('down', 'down_lost', 'up', 'up_lost')


def namespace(topic: str) -> str:
    return topic.rsplit('/', 1)[0] or '/'


def _norm(name: str) -> str:
    return '/' + name.strip('/')


class Traffic:
    # ponytail: keeps the last max_messages; a 15 min, 8-drone mission is ~4k messages
    def __init__(self, max_messages: int = 5000):
        self.msgs: OrderedDict[tuple, dict] = OrderedDict()
        self.max = max_messages
        self.endpoints = {'down': set(), 'up': set()}
        self.agents: set[str] = set()                  # senders seen: only they are endpoints

    def observe(self, direction: str, topic: str, key: tuple, *, sender: str, target: str = '',
                kind: str = '', subject: str = '', priority: float = 0.0, stamp: float = 0.0,
                payload: str = '', wall: float) -> None:
        """One sighting. ``direction``: 'down' (Directive) or 'up' (Report); ``key`` identifies
        the message across topics; ``stamp`` is the sender's header time, ``wall`` ours."""
        node, sender = namespace(topic), _norm(sender)
        m = self.msgs.get(key)
        if m is None:
            m = self.msgs[key] = {'dir': direction, 'from': sender, 'target': target,
                                  'kind': kind, 'subject': subject, 'priority': priority,
                                  't': stamp, 'payload': payload[:300], 'seen': {}}
            while len(self.msgs) > self.max:
                self.msgs.popitem(last=False)
        m['seen'].setdefault(node, wall)
        self.agents.add(sender)
        if node != sender:
            self.endpoints[direction].add(node)

    def links(self, m: dict, now: float) -> list[tuple[str, str, float | None]]:
        """(destination, 'ok' | 'lost' | 'pending', latency ms) per destination of ``m``."""
        sender, seen = m['from'], m['seen']
        dests = {n for n in self.endpoints[m['dir']] | set(seen)
                 if n != sender and n in self.agents}
        if m['dir'] == 'down' and m['target'] not in ('', '*'):
            dests = {n for n in dests if n == _norm(m['target']) or n in seen}
        t0 = seen.get(sender, min(seen.values()))
        out = []
        for d in sorted(dests):
            if d in seen:
                out.append((d, 'ok', max(0.0, seen[d] - t0) * 1000))
            else:
                out.append((d, 'lost' if now - t0 > GRACE_S else 'pending', None))
        return out

    def snapshot(self, now: float) -> dict:
        """Per (supervisor, worker) pair counts and latency, a timeline, and recent messages."""
        msgs = list(self.msgs.values())
        if not msgs:
            return {'pairs': [], 'timeline': None, 'messages': []}
        t0, t1 = min(m['t'] for m in msgs), max(m['t'] for m in msgs)
        width = max(t1 - t0, 1e-9) / BUCKETS
        pairs: dict[tuple[str, str], dict] = {}
        recent = []
        for i, m in enumerate(msgs):
            links = self.links(m, now)
            unrelayed = (_norm(m['target']) if m['target'] not in ('', '*') else '*', 'sent', None)
            for d, status, lat in links or [unrelayed]:
                down = m['dir'] == 'down'
                sup, wrk = (m['from'], d) if down else (d, m['from'])
                p = pairs.get((sup, wrk))
                if p is None:
                    p = pairs[(sup, wrk)] = {
                        'supervisor': sup, 'worker': wrk, 'lat': [], 'last_t': None, 'last': '',
                        'down': dict.fromkeys(('sent', 'ok', 'lost', 'pending'), 0),
                        'up': dict.fromkeys(('sent', 'ok', 'lost', 'pending'), 0),
                        'lanes': {k: [0] * BUCKETS for k in LANES}}
                c = p[m['dir']]
                c['sent'] += 1
                if status != 'sent':
                    c[status] += 1
                if lat is not None:
                    p['lat'].append(lat)
                if status != 'lost' and (p['last_t'] is None or m['t'] >= p['last_t']):
                    p['last_t'], p['last'] = m['t'], ('↓ ' if down else '↑ ') + m['kind']
                b = min(BUCKETS - 1, int((m['t'] - t0) / width))
                p['lanes'][m['dir'] + ('_lost' if status == 'lost' else '')][b] += 1
            if i >= len(msgs) - RECENT:
                recent.append({k: m[k] for k in ('dir', 'from', 'target', 'kind', 'subject',
                                                 'priority', 't', 'payload')} |
                              {'to': [[d, s, None if lat is None else round(lat)]
                                      for d, s, lat in links]})
        for p in pairs.values():
            lat = sorted(p.pop('lat'))
            p['latency_ms_p50'] = lat[len(lat) // 2] if lat else None
        return {'pairs': sorted(pairs.values(), key=lambda p: (p['supervisor'], p['worker'])),
                'timeline': {'t0': t0, 't1': t1, 'buckets': BUCKETS},
                'messages': recent[::-1]}
