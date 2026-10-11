"""``EpisodeRunner`` for ``ros2 jev search`` (B7): one seeded, headless Gazebo mission per run.

``run(params, seed)`` writes the drone and coordinator catalogs with ``params`` applied
(``jev_ai.core.ope.overrides``: ``tau=0.8``, ``kappa=0``, ``next_action.tau=1.0``, ``k_max=12``...;
``policy=P0..P4`` picks a named policy, below), launches ``swarm.launch.py`` with that seed on its
own ROS domain and Gazebo partition, waits for mission_scorer's episode summary, stops the launch
and returns J (SYSTEM_MODEL §6). Same seed = same targets, wind, visibility and faults (CRN).

    ros2 jev search --runner drone_swarm.episode:SwarmEpisode \\
        --runner-arg t_max_s=300 num_drones=4 --space policy=P0,P1,P2,P3 \\
        --min-seeds 1 --max-seeds 4

Named policies (DESIGN B7):
    P0  Jev only: no augmentation budget (the LLM is never asked)
    P1  the catalogs as shipped (Jev + LLM, act-then-revise)
    P2  exploratory sampling (tau 1.0)
    P3  no status-quo inertia (kappa 0)
    P4  the catalog patch from ``ros2 jev calibrate`` (runner arg ``calibrated=<patch.yaml>``)
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

POLICIES = {'P0': {'k_max': 0.01}, 'P1': {}, 'P2': {'tau': 1.0}, 'P3': {'kappa': 0.0}, 'P4': {}}


class SwarmEpisode:
    def __init__(self, num_drones: int = 4, t_max_s: float = 300.0, jev_backend: str = 'mock',
                 llm_backend: str = 'mock', out_dir: str = '/tmp/drone_swarm/search',
                 domain: int = 70, calibrated: str = '', gui: bool = False):
        self.n, self.t_max = int(num_drones), float(t_max_s)
        self.jev, self.llm, self.gui = jev_backend, llm_backend, bool(gui)
        self.out, self.domain, self.calibrated = Path(out_dir), int(domain), calibrated
        share = get_package_share_directory('drone_swarm')
        self.raw = {k: yaml.safe_load(open(f'{share}/config/{k}.yaml'))
                    for k in ('drone', 'coordinator')}

    def catalogs(self, params: dict, tag: str) -> dict[str, str]:
        params = dict(params)
        name = params.pop('policy', 'P1')
        settings = {**POLICIES[name], **params}
        paths = {}
        for k, raw in self.raw.items():
            if name == 'P4' and self.calibrated:
                from jev_ai.core.calibrate import merge
                raw = merge(raw, yaml.safe_load(open(self.calibrated)))
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
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain + seed % 30),
                   IGN_PARTITION=f'search_{tag}')
        cmd = ['ros2', 'launch', 'drone_swarm', 'swarm.launch.py', f'seed:={seed}',
               f'num_drones:={self.n}', f't_max_s:={self.t_max}', f'run_id:={run_id}',
               f'out_dir:={self.out}', f'jev_backend:={self.jev}', f'llm_backend:={self.llm}',
               f'drone_catalog:={cats["drone"]}', f'coordinator_catalog:={cats["coordinator"]}',
               'dashboard_port:=0', f'gui:={str(self.gui).lower()}']
        with open(self.out / f'{run_id}.log', 'w') as log:
            proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            deadline = time.monotonic() + 2.0 * self.t_max + 180
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
            return float('-inf')                       # the mission never ended: worst score
        return float(json.loads(summary.read_text())['J'])
