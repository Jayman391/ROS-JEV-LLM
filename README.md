# ROS-JEV: `jev_ai`, fast + deliberate decisions for ROS 2

`jev_ai` gives a ROS 2 robot two ways to decide, and fuses them:

- **System 1: Jev** (TypeSafe's decision model), about 160 ms. It returns a probability for each option of each question, every time the robot asks.
- **System 2: an LLM** (Claude), a few seconds. When Jev is unsure, or the stakes are high, the LLM reads the same situation and gives its own weights.
- **Fusion.** The robot acts on Jev's pick at once, then revises if the LLM disagrees. Probabilities are pooled, weighted by Jev's confidence. Hard rules (masks, forced answers) always win. An irreversible action needs both systems to rank it first.

You describe your decisions in a YAML **catalog**, and write four small callbacks:

- `encode` turns state into text;
- `mask` lists forbidden options;
- `require` gives forced answers;
- `on_decision` acts.

Everything else is configurable without code: when to ask the LLM, how sticky choices are, how much randomness to use, and how a fleet shares it.

```
 state ──encode──► Jev (S1) ──p──► act now (PROVISIONAL)
                       │ unsure?
                       └─► LLM (S2) ──q──► fuse ──► keep or revise (KEPT / REVISED)
 masks / require() ─────────────────────────┘   (hard rules always win)
```

## Quickstart (keyless, about 5 minutes after the image build)

```bash
docker compose build                      # once: ROS 2 Humble + Gazebo Fortress, ~10 GB
docker compose up -d && docker compose exec ros2 bash
# inside the container (the repo is at /ws/ROS-JEV):
colcon build && source install/setup.bash
ros2 launch jev_ai_examples toy_patrol.launch.py     # 3 robots, mock Jev + mock LLM
```

Open **http://localhost:8080** for the dashboard, which shows:

- **Decisions:** every decision and how it ended, with filters.
- **Comms:** supervisor ⇄ worker messages.
- **Metrics.**

The smallest agent you can copy is `src/jev_ai_examples/jev_ai_examples/minimal.py`, about 40 lines. To run it:

```bash
ros2 launch jev_ai_bringup bringup.launch.py         # servers + dashboard
ros2 run jev_ai_examples minimal                     # second shell
```

### Live models

```bash
cp .env.example .env            # then fill TYPESAFE_API_KEY and ANTHROPIC_API_KEY
docker compose up -d            # recreates the container with the keys in its environment
ros2 launch jev_ai_examples toy_patrol.launch.py jev_backend:=typesafe llm_backend:=anthropic
```

Keys are read from the environment only. Never put them in a catalog, in ROS parameters or in the repo.

## Examples

| Example | What it shows | Run |
|---|---|---|
| `minimal` | the 4 callbacks, nothing else | `ros2 run jev_ai_examples minimal` |
| `toy_patrol` | a fleet with shared noise, a supervisor, reports and directives, labels for calibration | `ros2 launch jev_ai_examples toy_patrol.launch.py` |
| `turtlesim` | the same agent in Python, C++ and a behavior tree | `ros2 launch jev_ai_examples turtlesim.launch.py` |
| [`drone_swarm`](examples/drone_swarm/README.md) | search and rescue in Gazebo: 8 workers, 2 relays and a coordinator. Rules fly; Jev and Claude judge | `ros2 launch drone_swarm swarm.launch.py gui:=true` |
| [`assembly_line`](examples/assembly_line/) | five arms at four stations build three products while pallets move arm to arm; supervisor (release / disruption / tool drift), per-arm op checks and fasten coop, QA with two-key scrap; discrete-event line_sim with RViz or a Gazebo Fortress puppet view | `ros2 launch assembly_line assembly_line.launch.py` |

## Docs

| Read | For |
|---|---|
| [docs/GUIDE.md](docs/GUIDE.md) | building an agent: the decision cycle, the catalog, the callbacks, hierarchies, C++ and behavior trees |
| [docs/TUNING.md](docs/TUNING.md) | **tuning the System 1 / System 2 balance**: goal → dial, presets, and every knob with its default |
| [docs/FRAMEWORK.md](docs/FRAMEWORK.md) | the specification (the math, guarantees, message types) |
| [deploy/README.md](deploy/README.md) | one container per node: docker compose and Kubernetes |
| [docs/DESIGN.md](docs/DESIGN.md) | roadmap and decision log |

## Packages

| Package | |
|---|---|
| `jev_ai` | the engine (`DecisionEngine`, `Hierarchy`), the pure-Python core (catalog, fusion, commitment, metrics) and the dashboard page |
| `jev_ai_servers` | `jev_server`, `llm_server`, `decide_server`, `decision_logger`, `dashboard`; the backends are mock, replay, typesafe, llama_ros, anthropic and openai_compat |
| `jev_ai_msgs` | messages: `JevDecide`, `Reason`, `DecisionRecord`, `Directive`, `Report`, `Label` |
| `jev_ai_cpp`, `jev_ai_bt` | C++ clients and BehaviorTree.CPP nodes (`Decide`, `JevAsk`, `Reason`) |
| `jev_ai_bringup` | `bringup.launch.py` (servers + logger + dashboard), default params, a catalog template |
| `ros2jev` | the `ros2 jev` CLI: `status`, `ask`, `report`, `calibrate`, `replay`, `ope`, `search` |

## The dev container

ROS 2 Humble and Gazebo Fortress run in an Ubuntu 22.04 container (Docker Desktop, Apple Silicon or x86). GUI apps, such as the Gazebo GUI and `rviz2`, draw on a virtual display that you view in the browser.

- **Gazebo / rviz2:** http://localhost:6080/vnc.html?resize=scale. The password is `password`, set by `VNC_PASSWORD` in the `Dockerfile`. Run `gui-fullscreen` in the container to maximise the window.
- **Shells:** open one `docker compose exec ros2 bash` per process. They share the container, so they see each other's topics. Stop everything with `docker compose down`.
- **Headless Gazebo:** `ign gazebo -s -r <world>.sdf`. Bridge topics with `ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'`.

**Troubleshooting:**

- **Blank noVNC:** check `docker compose ps`, then `docker compose restart`.
- **Wrong password:** rebuild after editing `VNC_PASSWORD`.
- **Slow Gazebo GUI:** rendering is software-only (`llvmpipe`). Use headless mode, or shrink the window.
- **Port 6080 or 8080 in use:** change the left side of the port mapping in `docker-compose.yml`.
- **Stale image:** `docker compose build --no-cache`.
