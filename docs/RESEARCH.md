# Research: LLMs + System One (Jev) models in ROS 2 drone swarms

Snapshot as of 2026-10-10. Items marked UNVERIFIED had no authoritative source.

## 1. TypeSafe Jev (System One model)

Jev was released 2026-09-15. It returns **typed, probabilistic decisions** instead of text. You give it a `state` and a map of named questions, and it answers every question in parallel in one pass.

| Question type | `criteria` | Answer fields |
|---|---|---|
| `noul` (yes/no) | optional `{true, false}` descriptions | `noul` = P(yes) |
| `choice` | map option → description (≤255 options) | `choice`, `probabilities`, `confidence` |
| `score` | ordered list of 2–10 levels | `score` (expected level index), `legend`, `probabilities`, `confidence` |

**API**
- **Endpoint:** `POST https://api.typesafe.ai/v1/systemone`, with `Authorization: Bearer $TYPESAFE_API_KEY`.
- **Spec:** OpenAPI at https://api.typesafe.ai/openapi.json
- **Models:** `jev-latest` and `jev-preview` both point to `jev-1.13.0`. Pin the exact version when you tune confidence thresholds.
- **Python SDK:** `pip install typesafe-sdk`, then `TypeSafeClient().system_one(state=..., questions={...})`. It has `Choice`, `Score` and `Noul` helpers and retries 429/529 itself. There is no C++ SDK.
- **OpenRouter:** OpenRouter offers the same API at `https://openrouter.ai/api/v1/systemone`, so the SDK works with `base_url="https://openrouter.ai/api"`. Its alpha Decisions API is at `/api/alpha/decisions`.
- **Limits:** 64k-token context, and 32k for state plus the longest question (OpenRouter says 32k overall). 80 req/s. $0.042 per million input tokens; output is free. TypeSafe's sample workflow ran in ~0.1 s. Third-party claims of 70–500 ms are UNVERIFIED.
- **Known weaknesses ([model jaggedness doc](https://docs.typesafe.ai/model-jaggedness/jev-1.13.md)):**
  - Weak at arithmetic, counting, dates and precise numbers.
  - Biased towards the first option.
  - Accuracy drops with long or noisy state.
  - **Design consequence:** do the numeric work in code (in C++), and give Jev short, already-interpreted state such as `"battery": "low (18%)"`.
- **ROS integrations:** none exist. This POC appears to be the first.

Sources: [docs.typesafe.ai quickstart](https://docs.typesafe.ai/introduction/quickstart), [API](https://docs.typesafe.ai/api.md), [Python SDK](https://github.com/typesafe-ai/typesafe-sdk-python), [OpenRouter Jev guide](https://openrouter.ai/docs/guides/community/jev), [What is Jev (OpenRouter)](https://openrouter.ai/blog/insights/what-is-jev/), [Kai Waehner on System One architecture](https://www.kai-waehner.de/blog/2026/09/28/how-system-one-models-like-jev-change-enterprise-ai-architecture/), [The Agentic Review](https://theagenticreview.substack.com/p/jev-and-the-system-one-model-a-decision).

## 2. Anthropic API (Claude 5.5)

- **Model IDs:** `claude-haiku-5-5` (drones) and `claude-sonnet-5-5` (coordinator). Both have 1M context and 128K max output.
- **Effort:** `output_config: {"effort": "low"|"medium"|"high"|"xhigh"|"max"}`. GA, no beta header.
  - Defaults: Haiku 5.5 `medium`, Sonnet 5.5 `high`.
- **Structured output:** `output_config: {"format": {"type": "json_schema", "schema": {...}}}`. In Python, use `client.messages.parse(..., output_format=Model)`.
- **Streaming:** `client.messages.stream(...)` yields text deltas, which map naturally to ROS action feedback.
- **C++:** there is no official SDK; the alternative is raw HTTPS with libcurl. This POC keeps the LLM calls in Python nodes.

Sources: [models overview](https://platform.claude.com/docs/en/about-claude/models/overview), [effort](https://platform.claude.com/docs/en/build-with-claude/effort), [structured outputs](https://platform.claude.com/docs/en/build-with-claude/structured-outputs), [Python SDK helpers](https://github.com/anthropics/anthropic-sdk-python/blob/main/helpers.md).

## 3. Prior art: LLMs in ROS 2 and swarms

| Project | Architecture | What we reuse |
|---|---|---|
| [NASA JPL ROSA](https://github.com/nasa-jpl/rosa) ([paper](https://arxiv.org/abs/2410.06472)) | A LangChain agent that wraps ROS topics, services and params as tools | Robot-specific system prompt plus a small tool vocabulary |
| [llama_ros](https://github.com/mgonzs13/llama_ros) | llama.cpp inside ROS 2. LLM calls are **actions** (`GenerateResponse.action`), with grammar-constrained output | "LLM call = action, streamed tokens = feedback"; constrained output |
| [Auromix ROS-LLM](https://github.com/Auromix/ROS-LLM) | GPT function calling mapped onto robot motion. Stale since 2023 | — |
| [bob_llm](https://docs.ros.org/en/rolling/p/bob_llm/) | An LLM exposed as a ROS service | — |
| [PromptCraft-Robotics](https://github.com/microsoft/PromptCraft-Robotics) | The LLM is shown a small high-level function API (AirSim drones) | Expose *formations and actions*, never raw velocities |
| [SwarmGPT](https://arxiv.org/abs/2412.08428) | The LLM proposes waypoints and an optimisation safety filter corrects them (Crazyflies) | **LLM proposes, deterministic controller disposes** |
| [SkySim](https://arxiv.org/abs/2602.01226) (2026) | ROS 2 + Gazebo + Crazyflie swarm. Gemini produces waypoints; a 20 Hz potential-field safety filter | Closest prior system; validates the layering |
| [RoCo](https://arxiv.org/abs/2307.04738) | One LLM per robot; the robots negotiate | Per-drone reasoning agent |
| [SMART-LLM](https://arxiv.org/abs/2309.10062) | Task decomposition → coalitions → allocation | Prompt structure for the coordinator planner |

The gap this POC fills is a **System 1 / System 2 split**. A cheap, fast, calibrated classifier (Jev) handles every routine decision. The LLM is called only when Jev's confidence is low or it flags an escalation. None of the systems above has a calibrated fast path; they call the LLM for everything.

## 4. Simulation: Gazebo Fortress (paired with Humble)

- **Drone model:** `https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4` with these plugins:
  - `MulticopterMotorModel`, one per rotor.
  - `MulticopterVelocityControl`, which listens on `/<robotNamespace>/gazebo/command/twist` (`ignition.msgs.Twist`) and `/<robotNamespace>/enable`.
  - `OdometryPublisher`, which publishes `/model/<name>/odometry`.
  - Example: [`multicopter_velocity_control.sdf`](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/multicopter_velocity_control.sdf). It was verified headless in our container: a `+z` twist made the drone climb.
- **Multiple drones:** the world uses one `<include>` per drone, each with its own `<name>` and `robotNamespace`. `ros_gz_sim create` has no namespace flag.
- **Sensors without a GPU:** IMU, altimeter, air pressure, magnetometer and navsat each need their world system plugin. Cameras and gpu_lidar need ogre2, and only run under Xvfb/llvmpipe; a camera dropped the real-time factor to ~0.24, so this POC uses no cameras by default.
- **Rejected options:**
  - PX4 SITL: Harmonic and later only, and MAVLink adds too much overhead for this POC.
  - CrazySim: needs Garden.
  - Crazyswarm2 sim: has no Gazebo world.

## 5. Microservices: ROS 2 in Compose and Kubernetes

- **DDS discovery:** use the **Fast DDS Discovery Server**. Every node sets `ROS_DISCOVERY_SERVER=discovery:11811`, and CLI/debug containers add `ROS_SUPER_CLIENT=TRUE`. The same setup works in Compose and k8s.
  - Kubernetes CNIs drop multicast ([Canonical](https://ubuntu.com/blog/exploring-ros-2-kubernetes-configurations)).
  - rmw_zenoh has Humble binaries, but its maintainers say they are untested.
  - CycloneDDS peer lists scale badly.
- **ign-transport (Gazebo ↔ bridge):** discovery is multicast, and `IGN_PARTITION` defaults to `hostname:user`, so separate containers must share an explicit `IGN_PARTITION`. In k8s, the Gazebo server and `ros_gz_bridge` share one pod (one network namespace).
- **Prior k8s work:** [eProsima DDS Router: ROS 2 & k8s](https://eprosima-dds-router.readthedocs.io/en/stable/rst/use_cases/ros_cloud.html), [ros_k8s](https://github.com/fujitatomoya/ros_k8s), [FogROS2](https://github.com/BerkeleyAutomation/FogROS2). No maintained Helm chart exists, so we use plain kustomize (built into kubectl).
