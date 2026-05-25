# Esterel Plan Generator

A standalone, ROS-free Python toolkit for generating **Esterel plan graphs** from temporal PDDL domains and problems. Given a PDDL domain and problem, it invokes an external temporal planner and constructs the same Esterel graph structure that [ROSPlan](https://github.com/KCL-Planning/ROSPlan)'s `PDDLEsterelPlanParser` produces — with no ROS installation required.

Two tools are provided:

| Script | Purpose |
|---|---|
| `esterel_plan_generator.py` | Single domain/problem pair — run planner, build graph, write output |
| `batch_esterel.py` | Multiple pairs — parallel execution, JSON results, summary reports |

---

## Background

ROSPlan represents plans as **Esterel graphs**: directed graphs in which nodes are plan-start, action-start, and action-end events, and edges encode temporal and causal relationships. Three edge types exist:

| Edge type | Meaning |
|---|---|
| `CONDITION` | Causal ordering: node B waits for node A because A produces a precondition B requires |
| `START_END_ACTION` | Fixed-duration constraint between an action's start and end nodes |
| `INTERFERENCE` | Ordering constraint arising from effect/condition conflicts between concurrent actions |

The C++ implementation inside ROSPlan requires a running ROS stack and three Knowledge Base services. This toolkit replaces those services with a pure-Python PDDL parser, making the pipeline usable in any environment.

Verified against ROSPlan's output on 23 benchmark instances across five IPC domains — graphs are topologically identical for identical input plans.

---

## Repository Structure

```
esterel-plan-generator/
├── esterel_plan_generator.py   # Single-pair generator (also importable as a library)
├── batch_esterel.py            # Batch runner with parallel execution and JSON output
├── planners/
│   ├── popf                    # POPF — Partial Order Planning Forward (Linux x86_64)
│   └── lpg-td                  # LPG-td 1.4 — Local Search Temporal Planner (Linux x86_64)
└── examples/
    ├── depots/
    │   ├── instance-01/        # domain.pddl + problem.pddl
    │   └── instance-02/
    ├── driverlog/
    │   ├── instance-01/
    │   └── instance-02/
    ├── rovers/
    │   ├── instance-01/
    │   └── instance-02/
    ├── satellite/
    │   ├── instance-01/
    │   └── instance-02/
    └── zenotravel/
        ├── instance-01/
        └── instance-02/
```

---

## Requirements

- Python ≥ 3.7 (standard library only — no third-party packages)
- Linux x86_64 (for the bundled planner binaries)
- A temporal PDDL planner on `PATH` or referenced by path (bundled planners ready to use)

---

## Quick Start

```bash
# Make bundled planners executable
chmod +x planners/popf planners/lpg-td

# Run on a single pair — POPF planner, 10-second timeout
python3 esterel_plan_generator.py \
    examples/rovers/instance-01/domain.pddl \
    examples/rovers/instance-01/problem.pddl \
    --planner "timeout TIMEOUT planners/popf -n DOMAIN PROBLEM" \
    --timeout 10 \
    --output summary

# Save full graph (JSON + YAML + plan text) to a directory
python3 esterel_plan_generator.py \
    examples/depots/instance-01/domain.pddl \
    examples/depots/instance-01/problem.pddl \
    --planner "timeout TIMEOUT planners/popf -n DOMAIN PROBLEM" \
    --timeout 10 \
    --output-dir results/depots-01/

# Use an existing plan file instead of running the planner
python3 esterel_plan_generator.py \
    examples/satellite/instance-01/domain.pddl \
    examples/satellite/instance-01/problem.pddl \
    --plan-file my_plan.txt \
    --output json

# Batch: run all examples in parallel, save per-pair JSON and a summary
python3 batch_esterel.py \
    --scan examples/ \
    --planner "timeout TIMEOUT planners/popf -n DOMAIN PROBLEM" \
    --timeout 10 \
    --jobs 4 \
    --output-dir results/ \
    --summary-json results/summary.json
```

---

## Bundled Planners

All three planners are pre-built static binaries for **Linux x86_64**. No installation required beyond `chmod +x`.

### POPF — Partial Order Planning Forward

```bash
planners/popf -n DOMAIN PROBLEM
```

A forward-chaining temporal planner that supports full PDDL 2.1 durative actions. Produces an anytime sequence of improving plans; the final plan before timeout is used. Source and documentation: [KCL-Planning/popf](https://github.com/KCL-Planning/popf).

| Flag | Effect |
|---|---|
| `-n` | Disable numeric optimisation (faster for purely propositional/durational problems) |

Recommended planner command:

```
timeout TIMEOUT planners/popf -n DOMAIN PROBLEM
```

### LPG-td 1.4 — Local Search Temporal Planner

```bash
planners/lpg-td -o DOMAIN -f PROBLEM -n 1
```

A local-search-based temporal planner developed at the University of Brescia. Supports PDDL 2.2 including durative actions and timed initial literals. Typically finds a solution quickly; the `-n 1` flag stops after the first solution.

| Flag | Effect |
|---|---|
| `-o DOMAIN` | Path to the domain file |
| `-f PROBLEM` | Path to the problem file |
| `-n 1` | Stop after the first solution found |

Recommended planner command (note the different argument order):

```
timeout TIMEOUT planners/lpg-td -o DOMAIN -f PROBLEM -n 1
```

> **Note:** LPG-td's plan output format differs slightly from POPF. The generator's parser handles both formats.

### OPTIC-CLP — Optimising Preferences and Time-Dependent Costs

```bash
planners/optic-clp DOMAIN PROBLEM
```

An extension of POPF with support for preferences (`:constraints`) and time-dependent cost metrics. Produces plans in the same format as POPF. Binary is 32-bit (statically linked) and runs on 64-bit Linux without additional libraries.

| Flag | Effect |
|---|---|
| `-N` | Ignore preferences (faster; equivalent to POPF behaviour) |
| `-E` | Skip computing the cost of the initial state (useful for large problems) |

Recommended planner command:

```
timeout TIMEOUT planners/optic-clp DOMAIN PROBLEM
```

Source: [KavrakiLab/optic](https://github.com/KavrakiLab/optic) — precompiled objects: [Dongbox/optic-clp-release](https://github.com/Dongbox/optic-clp-release).

### Using Other Planners

Any temporal planner that writes plans in standard POPF/OPTIC format is compatible:

```
<time>: (<action-name> <param1> ... <paramN>)  [<duration>]
```

Pass the command as a template string with `DOMAIN`, `PROBLEM`, and `TIMEOUT` tokens:

```bash
python3 esterel_plan_generator.py domain.pddl problem.pddl \
    --planner "timeout TIMEOUT /path/to/my-planner DOMAIN PROBLEM" \
    --timeout 60
```

Other compatible planners (must be built from source):

- **TFD** — [Temporal Fast Downward](https://tfd.informatik.uni-freiburg.de/): heuristic search-based temporal planner

---

## Example Domains

The `examples/` directory contains two instances of each of five classical IPC benchmark domains.

| Domain | Actions | Key features |
|---|---|---|
| **Depots** | lift, drop, drive, load, unload | Logistics + stacking; resource contention |
| **Driverlog** | walk, board-truck, drive-truck, disembark-truck | Path planning with drivers and trucks |
| **Rovers** | navigate, calibrate, take\_image, sample\_rock/soil, communicate\_\* | Multi-rover sensing and communication |
| **Satellite** | turn\_to, switch\_on/off, calibrate, take\_image, send\_image | Satellite scheduling with instrument management |
| **Zenotravel** | board, fly, zoom, debark, refuel | Passenger transport with fuel constraints |

`instance-01` is the smallest problem in each domain (good for quick tests); `instance-02` is moderately larger.

---

## `esterel_plan_generator.py` — CLI Reference

```
usage: esterel_plan_generator.py [-h] [--planner CMD] [--timeout SECONDS]
                                  [--data-path DIR] [--epsilon FLOAT]
                                  [--output {summary,json}] [--output-dir DIR]
                                  [--plan-file FILE]
                                  domain problem
```

### Positional arguments

| Argument | Description |
|---|---|
| `domain` | Path to the PDDL domain file |
| `problem` | Path to the PDDL problem file |

### Optional arguments

| Flag | Default | Description |
|---|---|---|
| `--planner CMD` | `timeout TIMEOUT popf -n DOMAIN PROBLEM` | Planner command template. `DOMAIN`, `PROBLEM`, and `TIMEOUT` are substituted at runtime |
| `--timeout SECONDS` | `60` | Planner timeout in seconds, substituted for `TIMEOUT`. Set `0` to disable |
| `--data-path DIR` | `/tmp/rosplan_esterel` | Directory for the planner's raw output. Ignored when `--plan-file` is set |
| `--epsilon FLOAT` | `0.1` | Safety margin (seconds) for TIL upper-bound edges |
| `--output {summary,json}` | `summary` | Output mode: human-readable summary or full JSON graph |
| `--output-dir DIR` | _(none)_ | Write `esterel_plan.json`, `esterel_plan.yaml`, and `plan.txt` into this directory |
| `--plan-file FILE` | _(none)_ | Use an existing plan file instead of running the planner |

### Exit codes

| Code | Meaning |
|---|---|
| `0` | Plan found and graph built successfully |
| `1` | Planner found no solution, or a fatal error occurred |

---

## `batch_esterel.py` — CLI Reference

```
usage: batch_esterel.py [-h] [--config FILE] [--pair DOMAIN PROBLEM]
                         [--scan DIR] [--planner CMD] [--timeout SECONDS]
                         [--jobs N] [--data-path DIR] [--epsilon FLOAT]
                         [--output-dir DIR] [--summary-json FILE] [--verbose]
```

### Input sources (combinable)

| Flag | Description |
|---|---|
| `--config FILE` | JSON config file listing pairs (see below) |
| `--pair DOMAIN PROBLEM` | Explicit pair; repeatable |
| `--scan DIR` | Recursively scan for `domain.pddl` + `problem.pddl` pairs |

### Key options

| Flag | Default | Description |
|---|---|---|
| `--planner CMD` | `timeout TIMEOUT popf -n DOMAIN PROBLEM` | Global planner command template |
| `--timeout SECONDS` | `60` | Planner timeout; substituted for `TIMEOUT` |
| `--jobs N` | `1` | Parallel workers (each runs one pair at a time) |
| `--output-dir DIR` | _(none)_ | Save one JSON file per pair here |
| `--summary-json FILE` | _(none)_ | Write machine-readable summary (success/fail, timing, counts) |
| `--verbose` | off | Print full tracebacks for failed pairs |

### Config file format

```json
{
  "planner":   "timeout TIMEOUT planners/popf -n DOMAIN PROBLEM",
  "timeout":   10,
  "data_path": "/tmp/esterel_batch",
  "epsilon":   0.1,
  "pairs": [
    {
      "name":    "rovers-01",
      "domain":  "examples/rovers/instance-01/domain.pddl",
      "problem": "examples/rovers/instance-01/problem.pddl"
    },
    {
      "name":    "rovers-02-lpg",
      "domain":  "examples/rovers/instance-02/domain.pddl",
      "problem": "examples/rovers/instance-02/problem.pddl",
      "planner": "timeout TIMEOUT planners/lpg-td -o DOMAIN -f PROBLEM -n 1"
    }
  ]
}
```

Per-pair `planner` overrides the global planner for that pair only. Override precedence:

```
per-pair "planner"  >  CLI --planner  >  config "planner"  >  built-in default
CLI --timeout       >  config "timeout"  >  built-in default (60 s)
```

---

## Output Formats

### Summary (default `--output summary`)

```
Esterel plan: 27 nodes, 75 edges

  Node   0 [PLAN_START      ] plan_start
  Node   1 [ACTION_START    ] calibrate_start
  Node   2 [ACTION_END      ] calibrate_end
  ...

  Edge   0 [CONDITION         ] [0] → [1]  [0.001, inf]
  Edge   1 [START_END_ACTION  ] [1] → [2]  [5.000, 5.000]
  Edge   5 [INTERFERENCE      ] [2] → [4]  [0.001, inf]
  ...
```

### JSON (`--output json` or `esterel_plan.json`)

```json
{
  "nodes": [
    {
      "node_id": 1,
      "node_type": 0,
      "name": "calibrate_start",
      "edges_in": [0],
      "edges_out": [1, 3],
      "action": {
        "action_id": 0,
        "name": "calibrate",
        "parameters": [{"key": "s", "value": "spectrometer0"}, ...],
        "duration": 5.0,
        "dispatch_time": 0.0
      }
    }
  ],
  "edges": [
    {
      "edge_id": 0,
      "edge_name": "edge_0",
      "edge_type": 0,
      "source_ids": [0],
      "sink_ids": [1],
      "duration_lower_bound": 0.001,
      "duration_upper_bound": null
    }
  ]
}
```

`node_type` constants: `ACTION_START=0`, `ACTION_END=1`, `PLAN_START=2`.  
`edge_type` constants: `CONDITION=0`, `START_END_ACTION=1`, `INTERFERENCE=2`.  
`duration_upper_bound` is `null` for unconstrained (infinity) edges.

### YAML (`esterel_plan.yaml`)

Byte-for-byte compatible with ROSPlan's `rostopic echo /rosplan_parsing_interface/complete_plan` output. Load with `yaml.safe_load_all()` and take the first document.

---

## Library API

```python
from esterel_plan_generator import generate_esterel_plan, generate_esterel_plan_from_text
from esterel_plan_generator import run_planner, parse_domain, parse_problem_tils
from esterel_plan_generator import plan_to_dict, plan_to_rosplan_dict, _dump_rosplan_yaml

# Full pipeline: run planner → build graph
plan = generate_esterel_plan(
    domain_path     = "examples/rovers/instance-01/domain.pddl",
    problem_path    = "examples/rovers/instance-01/problem.pddl",
    planner_command = "timeout TIMEOUT planners/popf -n DOMAIN PROBLEM",
    timeout         = 10,
)
if plan:
    print(f"{len(plan.nodes)} nodes, {len(plan.edges)} edges")

# Skip the planner — use an existing plan string
plan = generate_esterel_plan_from_text(
    plan_text    = open("my_plan.txt").read(),
    domain_path  = "examples/rovers/instance-01/domain.pddl",
    problem_path = "examples/rovers/instance-01/problem.pddl",
)

# Serialise to JSON-compatible dict
import json
print(json.dumps(plan_to_dict(plan), indent=2))

# Write ROSPlan-compatible YAML
with open("esterel_plan.yaml", "w") as fh:
    _dump_rosplan_yaml(plan_to_rosplan_dict(plan), fh)
```

---

## Supported PDDL Features

| Feature | Supported |
|---|---|
| `:durative-action` with `at start`, `over all`, `at end` conditions and effects | Yes |
| `:action` (instantaneous) | Yes — preconditions treated as at-start, effects as at-end |
| Typed parameters, including grouped typing (`?a ?b - type`) | Yes |
| Negative conditions (`not`) | Yes |
| Conjunctive conditions and effects (`and`) | Yes |
| Constants in predicates | Yes |
| Timed Initial Literals (`:init (at T (...))`) | Yes |
| Numeric conditions and effects | Silently skipped |
| Quantified conditions/effects (`forall`, `exists`) | Silently skipped |
| Conditional effects | Not supported |
| `:derived` predicates | Not supported |

---

## Limitations

- **Numeric fluents** are ignored during causal analysis. Plans relying on numeric resource constraints may produce incomplete graphs.
- **Concurrent actions with identical names** are distinguishable only by their `action_id`; node name labels will share the same prefix.
- **Quantified effects** are not expanded and are excluded from causal analysis.
- The planner is invoked via `subprocess.run` with `shell=True`, inheriting the current shell environment.
- Bundled binaries target **Linux x86_64** only. On other platforms, supply your own planner binary.

---

## Acknowledgements

- Graph construction algorithm is a Python translation of `KCL_rosplan::PDDLEsterelPlanParser::createGraph()` from [ROSPlan](https://github.com/KCL-Planning/ROSPlan).
- POPF is developed by the [KCL Planning Group](https://github.com/KCL-Planning/popf).
- LPG-td is developed at the [University of Brescia](https://lpg.unibs.it/lpg/).
- Benchmark domains are from the [International Planning Competition (IPC)](https://ipc.icaps-conference.org/) problem sets.
