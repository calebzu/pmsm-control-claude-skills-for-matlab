# Acceptance Criteria — three-vector FCS-MPC

Accept in two stages: **visual 4-check + 3-vector liveness first** (a failure here means a broken build regardless of numbers), then the **§E ripple** quantitative check — read honestly (the paper anchor is experimental; see below).

## Stage 1 — Visual 4-check + liveness (base discipline; mandatory before any metric)

Run Scenario A (no-load, `omega_ref` 0→1000 rpm step, 0.3 s). Confirm on scopes:

1. **Motor rotates & tracks** — speed reaches ~1000 rpm. Startup overshoot is expected under current-limited start; a flat-zero speed = stall (check §DC-CRIT polarity + §A theta).
2. **`iq` tracks `iq_ref` / `id≈0`** — iq saturates at `±iq_max` during accel, ~0 at no-load steady; `id` stays ≈ 0 throughout (dual-axis control nulls d as well). A small steady `id` bias (~one forward-Euler prediction-lag, e.g. ~0.07 A on the paper set; no delay compensation in the basic formulation) is expected — what matters is **no drift** and `Δid ≪ ODC`, not a literal zero.
3. **`i_abc` is AC** — phase currents are zero-mean sinusoids at the electrical fundamental under switching ripple. DC-locked `abc` = broken (theta / gate / polarity). **No-load FFT caveat**: at no-load the fundamental current (~0.06 A) is buried under switching ripple, so a naive global `max(fft)` returns the switching frequency — band-filter near `Pn·rpm/60` (e.g. 66.7 Hz @1000 rpm, Pn=4) to confirm the fundamental.
4. **Torque balance** — `Te ≈ 1.5·Pn·psif·iq` (= 1.44·iq for the paper set); accel matches `J·dω/dt`; steady no-load `Te ≈ 0`.

**Plus three-vector liveness (§TV-CRIT):** `t_i, t_j, t_z` are all meaningfully nonzero in steady state (the controller genuinely sequences three segments). If one is pinned at 0/`Tsc` every period, it has degenerated to dual/single-vector — a finding, not success.

## Stage 2 — §E ripple metric (quantitative — read HONESTLY)

Run Scenario B (steady 1000 rpm, no-load). Compute on a settled window (e.g. 0.2-0.3 s), sampled at the **fast** rate:
```
Δi_q = std(i_q,1) ,  Δi_d = std(i_d,1)     % population std (/N) = §E RMS-of-deviation
```
Paper Table 4 (Xu Yanping 2018) TV-MPCC @1000 rpm no-load: **`Δi_d = 0.21 A`, `Δi_q = 0.24 A`** (vs ODC-MPCC 0.69/0.41, traditional 1.10/1.10).

**Validated reproduction result** (paper Table-3 SPMSM, `Vdc=300`, `Tsc=100 μs`, ideal sim): `Δi_d ≈ 0.085 A`, `Δi_q ≈ 0.158 A` — **LOWER than the paper, ~0.4-0.66×, NOT a match.**

**How to read this (do not over-claim):**
- Paper Table 4 is **experimental** (real hardware: deadtime, sensor noise/quantization, real switching all raise ripple). An **ideal simulation** is expected to give *lower* ripple. The lower number is therefore consistent, not a defect.
- What you verify is **method fidelity + trend**: dual-axis deadbeat, 3 live vectors, `id≈0`, and TV ripple ≪ ODC's 0.69/0.41 — all reproduced.
- **Do NOT tune `Vdc`/`Ts`/window to hit the experimental absolute** — that would fit-to-the-paper and void the yardstick. Quote `Vdc`+`Tsc` with any ripple number.
- A `Vdc`/`Ts`/window sensitivity sweep is a legitimate *diagnostic* if you want to explain the gap, but it is not a pass/fail gate.

## Common-failure quick map

| Symptom | Likely cause | Pointer |
|---|---|---|
| `add_block` "invalid destination" | position array passed as the dest path | §J-CRIT, [anti_patterns.md](anti_patterns.md) #3 |
| `Could not locate EMLChart` (0 charts) | wrong Stateflow class; R2024b is `EMChart` / use `MATLABFunctionConfiguration.FunctionScript` | §G-CRIT, [anti_patterns.md](anti_patterns.md) #2 |
| `Stateflow:SignalTypePropagationError` | inherited/continuous Slicer feeding SPS bridge | §G-CRIT, [anti_patterns.md](anti_patterns.md) #1 |
| `DataBackPropagationSizeSuffix` on gate | gate is `[1 6]` row not `[6]` column | §D-CRIT, [anti_patterns.md](anti_patterns.md) #4 |
| speed=0, currents=0, `vds=vqs=0` | reversed DC polarity (diode clamp) | §DC-CRIT, [anti_patterns.md](anti_patterns.md) #6 |
| `t_z` (or `t_i`/`t_j`) pinned every period | T5 clamp collapsing to dual/single vector | §TV-CRIT, [anti_patterns.md](anti_patterns.md) #5 |
| theta over-integrates / abc wrong frequency | `ThetaSrc` not DISCRETE @Tsc | §A-CRIT |
| `iq` tracks but `id` drifts | cost/`id_ref` wrong, or saliency mismatch | [design_decisions.md](design_decisions.md) |
