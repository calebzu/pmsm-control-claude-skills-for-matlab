---
name: simulink-layout-tidy
description: Tidy the layout of an already-built Simulink model — make it compact and readable, remove block overlaps and lines that cut through blocks, and honestly minimize line-line crossings without ever falsely promising zero. Quantifies block overlaps, line-through-block hits, line-line crossings, model extent, and graph planarity (K3,3/K5 detection) before and after arranging, then exports a screenshot for human sign-off. Use when a Simulink or .slx block diagram looks messy, cluttered, overlapping, or tangled; when asked to clean up, arrange, beautify, declutter, or improve the readability of a Simulink diagram; or after auto-building a model that needs visual polish. General-purpose across any domain (not motor-specific). Local MATLAB only. Skip for non-Simulink diagrams, pure simulation or numerical questions, or any request that changes model behavior or port connections.
metadata:
  version: "1.0"
---

# simulink-layout-tidy

Make an already-built Simulink model compact, readable, and overlap-free — and report line crossings **honestly**, never promising a zero that the graph's topology forbids.

## Safety invariants (read first)

In Simulink a block's `Position` and a line's `Points` are purely **cosmetic** and orthogonal to the port connections. That fact is the whole basis for this skill being safe.

- **L1 contract (default, zero-risk).** This skill MUST only modify block `Position` and line `Points`. It MUST NOT add or remove blocks and MUST NOT change any port connection. *Why:* the compiled model is then byte-for-byte identical, so simulation results cannot change and **no functional re-test is needed**.
- **L2 contract (opt-in, explicit only).** L2 additionally allows (a) replacing long-range / multi-consumer wires with `Goto`/`From` and (b) wrapping a functional cluster into a `Subsystem`. These are logically equivalent but **change the block set**, so after enabling L2 you MUST run one smoke simulation and confirm no `NaN`/`Inf` and unchanged behavior. See [references/l2_contract.md](references/l2_contract.md).
- **NEVER hard-gate "zero line crossings."** A non-planar graph (one containing a `K3,3` or `K5` minor) cannot be drawn crossing-free on a plane (Kuratowski). Forcing zero would reject mathematically-valid models. Use the tiered gates below instead.

## Quick start (L1)

```matlab
addpath('scripts');
load_system('my_model');
rpt = tidy_layout('my_model');          % diagnose -> arrange -> de-overlap -> re-measure -> screenshot
% rpt.before / rpt.after hold the metric structs; rpt.screenshot is the PNG path.
```

Diagnose planarity (decides whether zero crossings is even reachable):

```matlab
extract_graph('my_model', 'graph.json');           % blocks=nodes, lines=edges
% then, from a venv/env with networkx:
% python3 scripts/planarity_check.py graph.json     (exit 2 == proven non-planar)
```

## Workflow

| Step | Action | Where |
|---|---|---|
| 1 | Measure BEFORE: overlaps, line-block hits, crossings, extent | `layout_metrics.m` |
| 2 | Diagnose planarity → is zero-crossing reachable at all? | `extract_graph.m` + `planarity_check.py` |
| 3 | Build two candidates: minimal-move (de-overlap only) and arrange | `tidy_layout.m` |
| 4 | Keep whichever has fewer crossings; NEVER accept an arrange regression | `tidy_layout.m` |
| 5 | Measure AFTER + assert hard gates + export PNG for human sign-off | `tidy_layout.m` |

`arrangeSystem` optimizes placement and orthogonal routing but **not** crossing number — on a non-planar graph it can *increase* crossings (the fixture goes 9→15). So `tidy_layout` measures the arrange cost on a **throwaway copy** and falls back to the minimal-move layout (original routing + just-enough de-overlap) whenever arrange is strictly worse on crossings. The reported `after` never regresses crossings vs `before`; crossings remain a soft reported metric, never a hard gate.

## Acceptance gates (tiered by reachability)

| Criterion | Gate type |
|---|---|
| block-block overlaps == 0 | **hard assert** (always achievable) |
| line-through-block hits == 0 (or tiny threshold) | hard assert |
| line-line crossings | **soft report + soft gate**: planar graph → expect ~0; non-planar → only require ≤ L1 floor, or resolved via L2 |
| screenshot "not ugly" | **human-in-the-loop** — user eyeballs the PNG |

Full rationale and thresholds: [references/gates.md](references/gates.md). Metric algorithms (overlap / crossing / planarity, with the verified cross-product test): [references/algorithms.md](references/algorithms.md).

## Self-contained fixture

`make_dirty_fixture.m` programmatically builds an ugly model (overlapping blocks, long crossing lines, a non-planar `K3,3` fan-in). `selftest.m` runs the full L1 flow + planarity check against it — no external model needed:

```matlab
addpath('scripts'); selftest        % builds fixture, tidies, asserts gates, prints planarity verdict
```
