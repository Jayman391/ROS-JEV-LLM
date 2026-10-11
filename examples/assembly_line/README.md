# Assembly line example

Five robot arms at four stations build three products as pallets move station to station; a QA
station inspects every finished part. Sensors are mocked (the truth plus noise), the line is a
discrete-event simulator, and jev_ai makes the judgement calls while rules do the routine work:

| who | decides | how |
|---|---|---|
| `line_supervisor` | which product next; how to ride out a faulted arm; whether a drifting arm needs a tool change | Jev + Sonnet |
| `arm_agent` ×5 | what to do about a marginal grip, misalignment or worn tool; whether to keep waiting for a late fasten partner | Jev + Haiku, only off the routine path |
| `qa_agent` | pass / rework / quarantine / scrap (scrap needs two keys) | Jev + Haiku, only for flagged parts |

```bash
ros2 launch assembly_line assembly_line.launch.py              # keyless mocks; dashboard :8080
ros2 launch assembly_line assembly_line.launch.py rviz:=true   # line view
ros2 launch assembly_line assembly_line.launch.py jev_backend:=typesafe llm_backend:=anthropic
```

| package | contents |
|---|---|
| `line_interfaces` | `ArmStatus`, `LineState`, `QaReading`, `PartOutcome`; `Release`, `Route`, `Dispose`; `Operate` |
| `line_control` (C++) | `line_sim`: the world (`line.hpp`, gtested without ROS), `/clock`, arm action servers, RViz markers |
| `assembly_line` (Py) | agents, scorer, mock rules, catalogs, scenario, launch, `ros2 jev search` runner |

Docs: [system model](../../docs/examples/assembly_line/SYSTEM_MODEL.md) (the world, sensors,
faults, scoring) and [decision model](../../docs/examples/assembly_line/DECISION_MODEL.md) (every
question, rule and guard). Deploy: `deploy/compose/assembly_line.yaml`,
`deploy/k8s/overlays/assembly_line`, image `deploy/docker/Dockerfile.assembly_line`.
