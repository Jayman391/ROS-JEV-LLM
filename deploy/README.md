# deploy/ — jev_ai as microservices (FRAMEWORK §13)

Optional: the library never needs it. One image, one container per ROS node.

```bash
# image (any ROS 2 distro with ros:<distro>-ros-base; humble and jazzy verified)
docker build -f deploy/docker/Dockerfile -t jev-ai:humble .
docker build -f deploy/docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t jev-ai:jazzy .

# compose: the framework (deploy/compose/compose.yaml) + the toy fleet
docker compose -f deploy/compose/toy.yaml up -d          # http://localhost:8080
docker compose -f deploy/compose/toy.yaml down

# Kubernetes (Docker Desktop): base + an app overlay
kubectl apply -k deploy/k8s/overlays/toy
kubectl -n jev-ai scale statefulset patrol --replicas 5
kubectl -n jev-ai port-forward svc/dashboard 8080
kubectl delete -k deploy/k8s/overlays/toy
```

| Piece | What it holds |
|---|---|
| `docker/Dockerfile` | `ros:<distro>-ros-base` + rosdep + pinned SDKs + the workspace (`--merge-install`) |
| `compose/compose.yaml` | `x-jev-ai-node` anchor; `jev_server`, `llm_server`, `decision_logger`, `dashboard`, `decide_server` (profile `decide`) |
| `compose/toy.yaml` | `include:`s the above with `toy.env`; three patrol robots |
| `k8s/base` | Deployments for the servers, logger (PVC) and dashboard; ConfigMaps `jev-ai-env` (backends, params file, run_id) and `jev-ai-params` (site overrides) |
| `k8s/overlays/toy` | `patrol` StatefulSet: pod `patrol-N` is agent `patrol_N` with seed N |
| `k8s/overlays/turtlesim` | turtlesim (offscreen), arena, `decide_server`, `turtle` StatefulSet (Python, C++, BT) |

**Keys** never go in images, params or manifests. Compose reads the repo's `.env`; k8s reads an
optional Secret: `kubectl -n jev-ai create secret generic jev-ai-keys --from-env-file=.env`, then
set `JEV_BACKEND=typesafe` / `LLM_BACKEND=anthropic` in the overlay's `jev-ai-env`.

**Resources and budgets.** Pods request 50m CPU / 128Mi and are capped at 500m / 384–512Mi; API
calls are bounded by the servers' `max_rps` and circuit breakers
(`params.yaml`), reported on the dashboard and by `ros2 jev status`.

**DDS.** Discovery is multicast on the compose network and on a single-node cluster's pod
network. Multi-node clusters need a Fast DDS Discovery Server (`ROS_DISCOVERY_SERVER`); not
included until needed.
