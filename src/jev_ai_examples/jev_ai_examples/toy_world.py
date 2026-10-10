"""The toy_patrol world, pure Python (no ROS): shared by the ROS agent (``toy_patrol``) and the
simulated episodes ``ros2 jev search`` runs (``toy_episode``).

A seeded random walk: battery drains (faster while inspecting), detections and smoke come and
go, the link flickers. ``step(now)`` advances it by one 0.5 s step and returns the engine events
it caused; ``oracle()`` is the ground truth labels are made of (A10).
"""
from __future__ import annotations

import random

STEP_S = 0.5


class ToyWorld:
    def __init__(self, rng: random.Random):
        self.rng = rng
        self.energy = rng.uniform(60, 95)
        self.task, self.detection, self.hazard, self.link = 'patrolling', 'none', 'none', 'good'
        self.inspect = None            # (commit_at, done_at) while inspecting
        self.reset_at = None
        self.orders, self.orders_until = 'none', 0.0

    # --- what the agent sees and may do (§5.1) --------------------------------------------------

    def encode(self, now: float) -> dict[str, str]:
        e = round(self.energy)
        level = 'ok' if e >= 40 else 'low' if e >= 15 else 'critical'
        orders = self.orders if now < self.orders_until else 'none'
        return {'task': self.task, 'energy': f'{level} ({e}%)', 'link': self.link,
                'hazards': self.hazard, 'detection': self.detection, 'orders': orders}

    def mask(self) -> dict[str, set[str]]:
        return {'next_action': {'inspect'}} if self.detection == 'none' else {}

    def require(self) -> dict[str, str]:
        return {'next_action': 'halt_now'} if self.energy <= 5 and self.task != 'halted' else {}

    def act(self, action: str | None, now: float) -> None:
        if action == 'halt_now' and self.task != 'halted':
            self.task, self.inspect, self.reset_at = 'halted', None, now + 8.0
        elif action == 'inspect' and self.inspect is None and self.task != 'halted':
            self.task = f'inspecting {self.detection.split(" at ")[0]}'
            self.inspect = (now + 1.0, now + 4.0)
        elif action == 'continue' and self.task != 'halted':
            self.task, self.inspect = 'patrolling', None

    def converge(self, target: str, issuer: str, ttl_s: float, now: float) -> None:
        """A supervisor directive (A7): share the target and record the orders."""
        if self.task == 'halted':
            return
        if self.detection == 'none' and target:
            self.detection = target                     # shared target: inspect is unmasked
        self.orders = f'converge on {target} (from {issuer})'
        self.orders_until = now + ttl_s

    def executing(self) -> str:
        return 'halt_now' if self.task == 'halted' else \
            'inspect' if self.inspect else 'continue'

    def oracle(self) -> dict[str, str]:
        """What a careful operator would choose now (labels for calibrate / ope)."""
        person = 'person' in self.detection
        if self.task == 'halted' or self.energy < 15:
            action = 'halt_now'
        else:
            action = 'inspect' if person else 'continue'
        urgency = 'high' if person else 'low' if self.detection != 'none' or \
            self.hazard != 'none' else 'none'
        return {'next_action': action, 'urgency': urgency}

    # --- dynamics ---------------------------------------------------------------------------

    def step(self, now: float) -> list[tuple]:
        """Advance one STEP_S. Events: ('trigger', name, priority) | ('commit_point', q) |
        ('completed', q)."""
        r, ev = self.rng.random, []
        if self.task == 'halted':
            if now >= self.reset_at:
                self.energy, self.task, self.reset_at = 100.0, 'patrolling', None
                ev.append(('trigger', 'battery_swapped', ''))
            return ev
        self.energy = max(0.0, self.energy - (0.45 if self.inspect else 0.25))
        if self.inspect:
            commit_at, done_at = self.inspect
            if commit_at and now >= commit_at:
                ev.append(('commit_point', 'next_action'))
                self.inspect = (None, done_at)
            if now >= done_at:
                self.inspect, self.detection, self.task = None, 'none', 'patrolling'
                ev.append(('completed', 'next_action'))
        if self.detection == 'none' and r() < 0.03:
            what = 'person-like shape' if r() < 0.4 else 'debris'
            self.detection = f'{what} at {self.rng.randint(5, 40)} m'
            ev.append(('trigger', 'detection', 'high'))
        elif self.detection != 'none' and not self.inspect and r() < 0.02:
            self.detection = 'none'
        if self.hazard == 'none' and r() < 0.01:
            self.hazard = 'smoke'
            ev.append(('trigger', 'hazard', ''))
        elif self.hazard != 'none' and r() < 0.05:
            self.hazard = 'none'
        self.link = 'weak' if r() < 0.1 else 'good'
        return ev
