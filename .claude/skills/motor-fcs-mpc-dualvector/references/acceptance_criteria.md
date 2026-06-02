# Acceptance Criteria — dual-vector FCS-MPC

Accept in two stages: **visual 4-check first** (a failure here means a broken build regardless of numbers), then the **§E ripple** quantitative check against the operating point.

## Stage 1 — Visual 4-check (base discipline; mandatory before any metric)

Run Scenario A (no-load, `omega_ref` 800→400 rpm @0.3 s, 0.6 s). Confirm on scopes:

1. **Motor rotates & tracks** — speed reaches ~800 rpm, steps cleanly to 400 rpm. Startup overshoot is expected under current-limited start (see startup note below); a flat-zero speed = stall (check §DC-CRIT polarity + §A theta).
2. **`iq` tracks `iq_ref`** — saturates at `±iq_max` during accel/brake, ~0 at no-load steady state.
3. **`i_abc` is AC** — phase currents are zero-mean sinusoids at the electrical fundamental under finite-set switching ripple. DC-locked `abc` = broken (theta / gate / polarity).
4. **Torque balance** — `Te ≈ Kt·iq` (e.g. `Te ≈ 1.5·Pn·psif·iq_max` at saturation); accel matches `J·dω/dt`; steady no-load `Te ≈ 0`.

Plus dual-vector liveness: the controller uses **multiple vectors** (not stuck on one index) and `Top` **varies in `(0, Tsc)`** (not pinned at 0 or Tsc) — confirms two vectors are actually being sequenced.

## Stage 2 — §E ripple metric (quantitative, vs operating point)

Run Scenario B (steady 1000 rpm, no-load). Compute on a settled window (e.g. 0.2-0.3 s):
```
Δi_q = std(i_q) ,  Δi_d = std(i_d)         % population std (/N) = §E RMS-of-deviation
```
(`std(x,1)` in MATLAB.) **⚠️ Sample `i_d/i_q` at the fast plant rate `Ts`, NOT the control rate `Tsc`** — q-axis deadbeat forces `i_q` to its reference at every `Tsc` boundary, so a `Tsc`-rate logger aliases the intra-period switching ripple away (falsely tiny `Δi_q`). Paper Table 2 dual-vector @1000 rpm: **`Δi_d = 0.35 A`, `Δi_q = 0.30 A`** (vs single-vector/traditional 1.15/1.5).

**Independent Phase-C reproduction** (Table-1 SPMSM, `Vdc=300`, `Tsc=100 μs`, fast-rate `Ts` sampling): `Δi_q = 0.293 A`, `Δi_d = 0.319 A` — within ~9% of Table 2; ripple cut 5.1×/3.6× vs single-vector.

Caveats when comparing:
- Ripple scales ~`Vdc·Tsc/L`; quote `Vdc` and `Tsc` with any ripple number. A different `Vdc` shifts the absolute values.
- `Δi_d > Δi_q` is expected (q-axis is deadbeat, d-axis is not).
- The §E metric is **RMS-of-deviation** (= std), NOT mean-absolute-deviation and NOT torque ripple — do not substitute a torque-MAD script (the paper's eq 18-19 has a missing-square typo).

## Common-failure quick map

| Symptom | Likely cause | Pointer |
|---|---|---|
| speed=0, currents=0, `vds=vqs=0` | reversed DC polarity (diode clamp) | [crit_conditions.md §DC](crit_conditions.md), [anti_patterns.md](anti_patterns.md) #1 |
| `Stateflow:SignalTypePropagationError` | inherited/continuous chart feeding SPS bridge | §G-CRIT |
| `DataBackPropagationSizeSuffix` on gate | gate is `[1 6]` row not `[6]` column | §D-CRIT |
| theta over-integrates / abc wrong frequency | `ThetaSrc` not DISCRETE @Tsc | §A-CRIT |
| ripple way off Table 2 | wrong `Vdc`/`Tsc`, or wrong machine params | [parameter_defaults.md](parameter_defaults.md) |
| `iq` tracks but `id` huge | cost weighting / `id_ref` wrong, or saliency mismatch | [design_decisions.md](design_decisions.md) |
