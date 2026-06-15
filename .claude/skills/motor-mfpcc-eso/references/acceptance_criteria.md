# Acceptance Criteria — Phase 6 two-layer check

Run the **validation case** (build_sop.md §7: paper Table I plant, 1000 rpm ramp 0.1 s, TL = 5 N·m
@ 0.25 s, StopTime 0.4 s, Ts = 1e-6, Tsc = 1e-4). All metrics from the logged timeseries; index every
channel by **its own** `.time` (build_sop.md §6 trap).

## Layer 0 — build asserts (before any sim)

- `beta01 == 2*(1-z12)` and `beta02 == (1-z12)^2/Tsc` (closed-form chain, eso_derivation.md §3).
  Validation numbers: `omega0=8500, beta01=1.7, beta02=7225`.
- Chart I/O counts: MFPCC_ESO 6/6, SpeedPI 2/1 (chart_contract.md).
- Output path is Unit Delay (IC=0), not ZOH.

## Layer 1 — physics (visual 4-check + 1 method-specific)

| # | Check | PASS condition |
|---|---|---|
| 1 | Motor rotates and tracks | final speed ≥ 98% of 1000 rpm, no reversal/oscillation |
| 2 | abc currents | symmetric AC sinusoidal (NOT DC-locked), amplitude grows at load step |
| 3 | Te energy balance | post-step mean `Te ≈ TL + B·ωm` (≈ 5.10 N·m) within 5% |
| 4 | id near zero | small positive bias allowed — see Layer 2 #4; NOT ampere-scale |
| 5 | ESO sanity (method-specific) | steady `|F̂| = √(Fa²+Fb²) ≈ ωe·ψf/Ls` (≈ 8700 A/s @ 1000 rpm) within 10% |

Layer 1 is a precondition: if any visual check fails, no Layer 2 number is trustworthy
(base skill discipline — broken-FOC defense).

## Layer 2 — metrics (steady-state window: last 0.05 s before TL step, and post-step window)

| # | Metric | Band | Rationale |
|---|---|---|---|
| 1 | iq tracking error (mean, steady) | abs ≤ 0.05 A | deadbeat: near-zero mean error |
| 2 | iq ripple (std, steady) | ≤ 0.08 A | reference run: 0.038 |
| 3 | load-step speed dip | ≤ 5% | reference run: 2.15% |
| 4 | id bias @ 1000 rpm | 0.05 – 0.25 A | ωe² law envelope (reference 0.13 A); **outside band in EITHER direction = investigate** — too high: ESO mis-gained; ~0 exactly: suspicious (check the chart isn't using machine params, model-free boundary) |
| 5 | speed steady error | ≤ 2% | outer PI with integrator |

## Reporting

- State each metric with its measured value vs band — no rounding a FAIL into a PASS.
- The id bias is **expected physics** (known_limitations.md #1). Report it as a characterized
  limitation, never as a defect to be tuned away, and never silently.
- Export a speed/iq/abc/F̂ figure (`print -dpng`) for the mandatory human visual review.
