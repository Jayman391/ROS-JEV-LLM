# Data flows

These are the runtime flows of `jev_ai`, traced from the code. Section numbers in parentheses (§) refer to [FRAMEWORK.md](../FRAMEWORK.md). Component details are in [components.md](components.md).

1. [Decision cycle (in-process, autonomous)](#1-decision-cycle-in-process-autonomous)
2. [Commitment state machine](#2-commitment-state-machine)
3. [Fusion and selection pipeline (one question)](#3-fusion-and-selection-pipeline-one-question)
4. [Irreversible actions](#4-irreversible-actions)
5. [Coupled noise](#5-coupled-noise)
6. [LLM augmentation request](#6-llm-augmentation-request)
7. [Remote decisions (C++ via decide_server)](#7-remote-decisions-c-via-decide_server)
8. [Hierarchy: reports up, directives down](#8-hierarchy-reports-up-directives-down)
9. [Records, metrics and replay](#9-records-metrics-and-replay)
10. [Failures and degradation](#10-failures-and-degradation)
11. [Server startup (lifecycle)](#11-server-startup-lifecycle)

---

## 1. Decision cycle (in-process, autonomous)

This is the common path: a tick or app trigger, then Jev, then a **PROVISIONAL** act, then the LLM, then **REVISED** or **KEPT**. Jev's ~100 ms answer is acted on immediately, and the LLM's seconds-long answer can only revise it. The robot never waits on System 2 for a reversible action.

```mermaid
sequenceDiagram
    autonumber
    participant App as App node<br/>(callbacks)
    participant Eng as DecisionEngine
    participant Com as Commitment<br/>(pure)
    participant Jev as jev_server<br/>jev/decide
    participant LLM as llm_server<br/>llm/reason
    participant Rec as decisions topic

    Note over Eng: 10 Hz timer: tick due (tick_hz) or a trigger queued
    App->>Eng: trigger('detection', priority='high')   (optional)
    Eng->>App: encode_state(qset)
    App-->>Eng: {task, energy, link, ...}  (validated against state_keys)
    Eng->>Jev: JevDecide(state, questions)  [async, jev_seq = n]
    Jev-->>Eng: answers p_J, question_errors
    Note over Eng: drop if jev_seq advanced (late answer)
    Eng->>App: mask(qset), require(qset)   (fresh)
    Eng->>Eng: decision event due?<br/>trigger · argmax change > margin · current masked · require differs · no current
    alt no event due
        Eng-->>App: on_idle(qset)   (decide_server only)
    else event due
        Eng->>Com: on_event(state, p_J, masks, triggers)
        Com-->>Eng: Step PROVISIONAL (generation g, request_llm=priority)
        Eng->>Rec: DecisionRecord(PROVISIONAL)
        Eng->>App: on_decision(PROVISIONAL)  → app acts now
        Eng->>Eng: TokenBucket.take(priority)
        Eng->>LLM: Reason goal(system, context, schema)  [tagged g]
        LLM-->>Eng: result JSON {weights, params, rationale, flags}
        Note over Eng: drop if g advanced, invalid schema → 1 retry
        Eng->>App: mask(qset), require(qset)   (fresh again)
        Eng->>Com: on_llm(g, q_L, masks)   (same Gumbel noise)
        alt augmented pick differs
            Com-->>Eng: REVISED (act) + COMMITTED (no act)
            Eng->>Rec: REVISED, COMMITTED
            Eng->>App: on_decision(REVISED)  → app switches
        else same pick
            Com-->>Eng: KEPT (final, no act)
            Eng->>Rec: KEPT
        end
    end
```

Short paths that skip the LLM:

| Condition | Phase |
|---|---|
| every selecting question is `require`d | **FORCED** (final; no models consulted for those) |
| `N_fail` consecutive Jev failures | **FALLBACK** (`on_degraded` called) |
| set has `augment: false`, or `c ≥ skip_llm_above_c`, and nothing irreversible is pending | **COMMITTED** at once |
| rate-limited, `llm_server` unavailable, goal rejected, LLM error | PROVISIONAL → **COMMITTED** (δ ≡ 0; metrics label `LLM_MISSING`) |
| `T_wait` passes with no LLM answer | PROVISIONAL → **COMMITTED**, or **FALLBACK** if an irreversible class was pending |

## 2. Commitment state machine

There is one `Commitment` per `(agent, question set)`. `generation` increments at every decision event, so a newer event silently supersedes an open one: its LLM result is discarded on arrival.

```mermaid
stateDiagram-v2
    [*] --> Event: decision event
    state Event <<choice>>
    Event --> FORCED: all selecting questions required
    Event --> FALLBACK: N_fail Jev failures (degraded)
    Event --> COMMITTED: no augmentation needed
    Event --> PROVISIONAL: otherwise (act now, request LLM)

    state Await <<choice>>
    PROVISIONAL --> Await: LLM result (same generation and mask hash)
    Await --> REVISED: augmented pick differs
    Await --> KEPT: same pick
    Await --> FALLBACK: irreversible pending and no LLM weights for it
    REVISED --> COMMITTED: immediately (non-acting)

    PROVISIONAL --> COMMITTED: T_wait / LLM failed (Jev-only)
    PROVISIONAL --> FALLBACK: T_wait / LLM failed with irreversible pending
    PROVISIONAL --> [*]: superseded by a newer event (generation++)

    FORCED --> [*]
    FALLBACK --> [*]
    COMMITTED --> [*]
    KEPT --> [*]
```

| Phase | `final` | `act` (calls `on_decision`) |
|---|---|---|
| PROVISIONAL | no | yes |
| REVISED | no | yes |
| KEPT | yes | no (only with `all_phases`) |
| COMMITTED (after PROVISIONAL or REVISED) | yes | no (only with `all_phases`) |
| COMMITTED (direct) · FORCED · FALLBACK | yes | yes |

Per-question **hold rules** sit inside selection:
- **dwell:** no voluntary switch within `T_dwell` of the last switch. A mask-forced switch is immediate.
- **committed:** an `until_commit` class past `commit_point()` is held until `completed()`. Masks, `require` and a *different* irreversible class winning the §6.4 rule still override it.

## 3. Fusion and selection pipeline (one question)

`fusion.decide_question` turns Jev's distribution and the optional LLM weights into one selected class plus a full trace.

```mermaid
flowchart TB
    in["p_J (Jev) · q_L (LLM, optional) · masks · require · current · noise key"]
    in --> fbm["fallback in mask? → unmask it (flag fallback_mask_ignored)"]
    fbm --> rqm["required in mask? → unmask it (flag require_overrides_mask)"]
    rqm --> noise["Gumbel noise g(a) = −log(−log U)<br/>U = H(run_id, group|agent, question, epoch, a)"]
    noise --> jerr{"p_J missing<br/>(Jev error)?"}
    jerr -- yes --> keep["keep current if selectable, else fallback<br/>(report-type acts: nothing)"]
    jerr -- no --> pool

    subgraph pool["pool (§6.2)"]
        direction TB
        c["c = 1 − H(p_J)/log|A|"]
        b["β(c) = β_min + (β_max − β_min)(1−c)^γ<br/>β_eff = β · exp(−age/T_stale)"]
        l["ℓ(a) = centred log q_L over unmasked classes"]
        d["δ(a) = clip(β_eff·ℓ(a), ±B)<br/>δ ≤ 0 on irreversible classes"]
        z["z(a) = log p_J(a) + δ(a) + b_q(a)<br/>z = −∞ if masked"]
        c --> b --> d
        l --> d --> z
    end

    pool --> act{"act"}
    act -- "required" --> forced["selected = required (FORCED path)"]
    act -- expectation --> exp["values: level_mean, level_median"]
    act -- threshold --> thr["true if softmax(z)[true] ≥ θ"]
    act -- none --> none["no selection"]
    act -- "sample / argmax" --> sel

    subgraph sel["_select (§6.3–6.4)"]
        direction TB
        irr{"argmax z irreversible and<br/>wins by m_irrev on fused AND Jev-only z?"}
        irr -- "yes, augmented" --> pickirr["select it (irreversible path)"]
        irr -- "yes, not yet augmented" --> pend["hold current / fallback<br/>path irreversible_pending → wait for LLM"]
        irr -- no --> held{"held past commit point?"}
        held -- yes --> h["keep current"]
        held -- no --> dwell{"inside T_dwell?"}
        dwell -- yes --> dw["keep current"]
        dwell -- no --> gm["Gumbel-max over reversible, unmasked:<br/>argmax (z + κ·1[current])/τ + g<br/>propensity = softmax(...)[selected]"]
    end
```

The LLM's weight is largest when Jev is **uncertain** (low c), shrinks as the advice ages, and can never move a class by more than B nats (default log 20, a 20× odds change). Only masks and `require` are absolute.

## 4. Irreversible actions

`toy_patrol`'s `halt_now` shows the three outcomes. The class is never sampled.

```mermaid
sequenceDiagram
    autonumber
    participant App
    participant Eng as DecisionEngine
    participant Jev as jev_server
    participant LLM as llm_server

    App->>Eng: encode → energy "low (12%)"
    Eng->>Jev: JevDecide
    Jev-->>Eng: halt_now leads by ≥ m_irrev
    Eng->>App: PROVISIONAL (holds current class, path irreversible_pending)
    Eng->>LLM: Reason (priority reserved, bucket keeps a token for this)
    alt LLM agrees: rule still holds on fused AND Jev-only logits
        LLM-->>Eng: weights favour halt_now
        Eng->>App: REVISED halt_now → then COMMITTED
    else LLM vetoes: δ ≤ 0 pulls halt_now below the margin
        LLM-->>Eng: weights favour continue
        Eng->>App: re-pick among reversible classes, same noise → REVISED or KEPT
    else no answer within T_wait, or no weights for this question
        Eng->>App: FALLBACK (catalog fallback, always reversible)
    end
    Note over App,Eng: Real emergencies use require(): energy ≤ 5% → FORCED halt_now,<br/>deterministic, never waits on an API.
```

## 5. Coupled noise

Agents in one **group** coordinate without talking by sampling with shared Gumbel noise. Noise is a pure hash of logged keys, so every decision can be re-derived exactly offline.

```mermaid
flowchart LR
    runid["run_id<br/>(one per launch)"] --> h
    agent["agent"] --> h
    q["question"] --> h
    ep{"answering a<br/>directive?"}
    ep -- yes --> e1["epoch = d:issuer:id<br/>ρ = coupling_directive (1.0)"]
    ep -- no --> e2["epoch = t:floor(t_node / epoch_s)<br/>ρ = coupling (0.5)"]
    e1 & e2 --> h["H(couple, run_id, agent, question, epoch) < ρ ?"]
    h -- "yes (and group ≠ agent)" --> kg["key = group → shared noise"]
    h -- no --> ka["key = agent → own noise"]
    kg & ka --> g["g(a) = −log(−log H(gumbel, run_id, key, question, epoch, a))"]
```

- The provisional and the augmented decision **reuse the same g**, so a revision happens only when augmentation moved the distribution enough to change the winner.
- Coupling needs a shared `run_id`. `toy_patrol.launch.py` mints one and passes it to every agent, the supervisor and `decide_server`.

## 6. LLM augmentation request

```mermaid
flowchart LR
    subgraph sys["system (stable → prompt-cached)"]
        s1["catalog llm.system template ({agent})"]
        s2["questions, classes, criteria"]
        s3["mask semantics, objective, flag meanings"]
    end
    subgraph ctx["context (per call)"]
        c1["State in &lt;untrusted&gt; (NFKC-folded, &lt; &gt; escaped)"]
        c2["Jev probabilities, classes shuffled with seed run_id:agent:decision_id"]
        c3["forbidden-now classes, current class"]
        c4["last 3 final decisions in &lt;untrusted&gt;"]
    end
    schema["response_schema from catalog:<br/>weights{q:{class:0..1}}, params (bounded),<br/>rationale ≤ 2 sentences, flags"]
    sys & ctx & schema --> goal["Reason goal"] --> srv["llm_server → backend"]
    srv --> parse["parse_response:<br/>validate, clamp params to llm_params,<br/>surface flags (option_missing, data_suspect)"]
    parse --> qL["q_L per question → Commitment.on_llm"]
```

## 7. Remote decisions (C++ via decide_server)

C++ owns no decision maths. `decide_server` runs the same engine, with the goal supplying what in-process callbacks would.

```mermaid
sequenceDiagram
    autonumber
    participant Cpp as C++ node<br/>DecisionClient
    participant DS as decide_server
    participant Eng as hosted DecisionEngine<br/>(agent key, non-autonomous)
    participant Jev as jev_server
    participant LLM as llm_server

    Cpp->>DS: Decide goal(agent, set, state, mask, require, current, triggers)
    DS->>DS: validate set + state keys (else abort with "error: ...")
    DS->>DS: preempt older goal for (agent, set) → aborted
    DS->>Eng: completed:/commit_point: hooks, then trigger(name | '' = tick, fresh)
    Eng->>Jev: JevDecide
    Jev-->>Eng: p_J
    alt tick and no decision event due
        Eng-->>DS: on_idle
        DS-->>Cpp: result = standing final decision (same decision_id)
    else decision event
        Eng-->>DS: PROVISIONAL
        DS-->>Cpp: feedback PROVISIONAL → on_decision
        Eng->>LLM: Reason
        LLM-->>Eng: weights
        Eng-->>DS: REVISED (act) / KEPT / COMMITTED (final)
        DS-->>Cpp: feedback REVISED (if any), then result (final)
    end
    Note over Cpp: start(hz): sends a tick goal only when none is in flight,<br/>so C++ cadence matches tick_hz.
```

## 8. Hierarchy: reports up, directives down

This is the A7 toy: N patrol workers and one supervisor. The supervisor is itself an engine, and only its *final* `converge` is sent down.

```mermaid
sequenceDiagram
    autonumber
    participant W1 as patrol_1
    participant W2 as patrol_2
    participant S as supervisor
    participant Jev as jev_server
    participant LLM as llm_server

    W1->>W1: final decision with report_now = true
    W1->>S: Report(status, subject=detection, priority=urgency.level_mean, beliefs=fused)
    S->>S: on_report: new detection? → engine.trigger('report')
    S->>Jev: JevDecide(fleet_order)
    S->>LLM: Reason
    S->>S: final order = converge
    S->>W1: Directive(converge, target, ttl 15 s, id k)
    S->>W2: (same Directive, target "*")
    par each worker
        W1->>W1: accept? target · allowlist /supervisor · not expired · not duplicate
        W1->>W1: on_directive: adopt target, orders = "converge on ..."
        W1->>W1: engine.trigger(directive:converge, fresh, epoch d:/supervisor:k)
        W1->>Jev: JevDecide
    and
        W2->>W2: same filter, orders, trigger: epoch d:/supervisor:k, ρ = 1.0
        W2->>Jev: JevDecide
    end
    Note over W1,W2: Same run_id + group + directive epoch → identical Gumbel noise,<br/>so the workers' samples agree as far as their fused distributions allow.
```

## 9. Records, metrics and replay

```mermaid
flowchart LR
    eng["every engine<br/>(in-process + decide_server)"] -- "DecisionRecord per phase" --> topic(("decisions"))
    lab["app / scorer"] -- Label --> ltopic(("labels"))
    topic & ltopic --> logger["decision_logger"] --> jsonl[("decisions-*.jsonl<br/>labels.jsonl")]
    topic & ltopic --> dash["dashboard<br/>(ring buffer)"]
    diag(("/diagnostics")) --> dash
    dash -- "metrics.compute" --> web["browser :8080"]
    jsonl --> report["ros2 jev report"] -- "metrics.compute" --> html[("report.html")]
    jsonl -.-> future["calibrate · replay · ope · search<br/>(planned, A10)"]
    future -.-> patch[/"policy: patch<br/>(β, γ, b_q, B, τ, κ, ρ)"/] -.-> catalog[("catalog")]

    jsrv["jev_server / llm_server<br/>record_path set"] -- "successful answers" --> cas[("cassette JSONL")]
    cas -- "backend:=replay" --> jsrv2["jev_server / llm_server<br/>replay backend"]
```

- A *decision* is all records sharing `(run_id, agent, question_set, decision_id)`, one per phase. Its outcome is read from the phase path.
- Each record carries both clocks (`header` = node clock, `wall_stamp`), the catalog and policy hashes, the models, latencies, tokens and `cost_usd`. Per question it carries `p_jev`, `q_llm`, β, masks, logits, τ, κ, **noise** and **propensity**: everything per-decision OPE needs.

## 10. Failures and degradation

```mermaid
flowchart TB
    subgraph llm["System 2 failures"]
        l1["llm_server down / inactive"] --> d0
        l2["breaker open → goal rejected"] --> d0
        l3["timeout (T_wait) / error / refusal / truncation"] --> d0
        l4["invalid JSON or schema"] --> retry["1 retry"] --> d0
        l5["rate-limited by engine bucket"] --> d0
        d0["δ ≡ 0: Jev-only<br/>PROVISIONAL → COMMITTED<br/>(irreversible pending → FALLBACK)"]
    end
    subgraph jev["System 1 failures"]
        j1["one question errors"] --> k1["that question keeps its current class"]
        j2["whole call fails / times out / unavailable"] --> k2["keep current class"]
        k2 --> n{"N_fail (3)<br/>in a row?"}
        n -- yes --> fb["FALLBACK + on_degraded()"]
    end
    subgraph both["Both down"]
        b1["hold fallback; app owns any domain failsafe"]
    end
```

| Guard | Mechanism |
|---|---|
| Late Jev answer | `jev_seq` mismatch → dropped; Jev deadline `jev_timeout` checked by the engine timer |
| Late LLM answer | `generation` mismatch → dropped; open goal cancelled once its decision resolves |
| Masks changed while the LLM was thinking | mask hash recomputed on arrival; mismatch → dropped |
| Server-side overload | `max_rps` and `max_tokens_per_min` refuse fast; the breaker opens after 5 consecutive failures and probes after 30 s |
| Key missing for a live backend | configure FAILURE, `/diagnostics` ERROR "not configured: ... needs an API key" |

## 11. Server startup (lifecycle)

```mermaid
sequenceDiagram
    autonumber
    participant L as ros2 launch
    participant S as jev_server / llm_server<br/>(BackendServer)
    participant EP as entry points
    participant B as backend
    participant D as /diagnostics

    L->>S: start with params.yaml + backend:=...
    S->>S: declare params · lock them read-only · start 1 Hz diagnostics
    S->>S: autostart → trigger_configure()
    S->>EP: load_backend(group, name)  (installed first, then built-in)
    S->>S: read_key(key_env or api_key_file)
    alt key_env set and no key
        S-->>D: ERROR "not configured: backend needs an API key"
    else
        S->>B: construct(params under <backend>.*, key)
        S->>B: await check()   (key, endpoint, model)
        alt check fails
            S-->>D: ERROR "not configured: ..."
        else ok
            S->>S: Budget, CircuitBreaker, setup_endpoint()
            S->>S: trigger_activate() → active
            S-->>D: OK (backend, model, breaker closed)
        end
    end
```

`decide_server`, `decision_logger` and `dashboard` are plain nodes. `decide_server` exits at startup if its `catalog` parameter is missing.
