"""``EpisodeRunner`` for ``ros2 jev search``: one seeded, headless shift per run.

``run(params, seed)`` writes the three catalogs with ``params`` applied
(``jev_ai.core.ope.overrides``; ``policy=P0..P3`` picks a named policy), launches
``assembly_line.launch.py`` with that seed on its own ROS domain, waits for line_scorer's episode
summary and returns J (SYSTEM_MODEL §8). Same seed = same wear rates, grips, jams and dropouts.

    ros2 jev search --runner assembly_line.episode:LineEpisode \\
        --runner-arg t_max_s=900 --space policy=P0,P1,P2,P3 --min-seeds 1 --max-seeds 4

    P0  Jev only (no augmentation budget)      P2  exploratory sampling (tau 1.0)
    P1  the catalogs as shipped                 P3  no status-quo inertia (kappa 0)
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from jev_ai.core.ope import overrides

POLICIES = {'P0': {'k_max': 0.01}, 'P1': {}, 'P2': {'tau': 1.0}, 'P3': {'kappa': 0.0}}
CATALOGS = ('supervisor', 'arm', 'qa')


class LineEpisode:
    def __init__(self, t_max_s: float = 900.0, jev_backend: str = 'mock',
                 llm_backend: str = 'mock', out_dir: str = '/tmp/assembly_line/search',
                 domain: int = 40, real_time_factor: float = 4.0):
        self.t_max, self.rtf = float(t_max_s), float(real_time_factor)
        self.jev, self.llm = jev_backend, llm_backend
        self.out, self.domain = Path(out_dir), int(domain)
        share = get_package_share_directory('assembly_line')
        self.raw = {k: yaml.safe_load(open(f'{share}/config/{k}.yaml')) for k in CATALOGS}

    def catalogs(self, params: dict, tag: str) -> dict[str, str]:
        params = dict(params)
        settings = {**POLICIES[params.pop('policy', 'P1')], **params}
        paths = {}
        for k, raw in self.raw.items():
            path = self.out / f'{tag}_{k}.yaml'
            path.write_text(yaml.safe_dump(overrides(raw, settings), sort_keys=False))
            paths[k] = str(path)
        return paths

    def run(self, params: dict, seed: int) -> float:
        self.out.mkdir(parents=True, exist_ok=True)
        tag = f's{seed}-{uuid.uuid4().hex[:6]}'
        cats = self.catalogs(params, tag)
        run_id = f'search-{tag}'
        summary = self.out / f'episode_{run_id}.json'
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain + seed % 30))
        cmd = ['ros2', 'launch', 'assembly_line', 'assembly_line.launch.py', f'seed:={seed}',
               f't_max_s:={self.t_max}', f'run_id:={run_id}', f'out_dir:={self.out}',
               f'jev_backend:={self.jev}', f'llm_backend:={self.llm}', 'dashboard_port:=0'] + \
            [f'{k}_catalog:={cats[k]}' for k in CATALOGS]
        with open(self.out / f'{run_id}.log', 'w') as log:
            proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            deadline = time.monotonic() + 1.5 * self.t_max / self.rtf + 120
            try:
                while not summary.exists() and time.monotonic() < deadline and \
                        proc.poll() is None:
                    time.sleep(2.0)
            finally:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
        if not summary.exists():
            return float('-inf')                       # the shift never ended: worst score
        return float(json.loads(summary.read_text())['J'])
