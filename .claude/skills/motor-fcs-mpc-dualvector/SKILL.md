---
name: motor-fcs-mpc-dualvector
description: PMSM Dual-Vector Finite-Control-Set MPC Builder. Build an inner-loop two-vectors-per-period finite-control-set MPC current controller for a three-phase voltage-source-inverter-driven PMSM (SPMSM / mild-saliency IPMSM via parameterization) in Simulink, with an outer speed PI providing iq_ref. Two vectors per control period (V_opt1 + V_j) with q-axis-deadbeat time allocation cut switching-cycle current ripple ~3-5x below single-vector FCS-MPC. Use when constructing, reproducing, porting, or extending a dual-vector / two-vector FCS-MPC current-control simulation in Simulink (keywords dual-vector MPC, two-vector MPC, double-vector MPCC, deadbeat time allocation, duty-cycle MPC, 双矢量模型预测, 占空比 MPC). Skip for single-vector FCS-MPC (use motor-fcs-mpc), FOC, DTC, SMC, sensorless, scalar V/Hz, BLDC trapezoidal, induction-motor MPC, three-or-more-vector / multi-step-horizon MPC, strong-saliency IPMSM MTPA, weak-field, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-fcs-mpc-dualvector — PMSM Dual-Vector Finite-Control-Set MPC Builder

Three-phase 2-level voltage-source inverter + PMSM (SPMSM / mild-saliency IPMSM via parameterization). Inner loop = **dual-vector** FCS-MPC current control in the dq frame: each control period applies **two** voltage vectors — the first optimal active vector `V_opt1` for a deadbeat-computed on-time `t_opt1`, then a second vector `V_j` for the remainder `Tsc − t_opt1`. Outer loop = speed PI providing `iq_ref`. The second vector + intra-period time split drive switching-cycle current ripple far below single-vector FCS-MPC at the same `Tsc`.

Distilled from Xu Yanping et al. 2017 (Two-Vector Model Predictive Current Control for PMSM, Trans. China Electrotech. Soc. 32(20):222-230). Control-law formulas are the signed **N2–N5** in [pmsm_formulas.md §E](../../../shared/formulas/pmsm_formulas.md); plant physics, prediction, speed PI, Clarke/Park and the 8-vector set are reused by pointer from the same file (§0-§7 / §A / §B.5).

Layered on [motor-pmsm-base](../motor-pmsm-base/SKILL.md). All base discipline applies (Goto TagVisibility, Vdc/BEMF rule, Visual 4-check, broken-FOC defense).

## What makes dual-vector different (vs single-vector `motor-fcs-mpc`)

| Aspect | Single-vector (`motor-fcs-mpc`) | Dual-vector (this skill) |
|---|---|---|
| Vectors per period | 1 (held the whole `Tsc`) | 2: `V_opt1` for `t_opt1`, then `V_j` |
| Modulator | none; one switch state per period | none; **time-slicer** sequences two states within `Tsc` |
| Extra novelty | — | q-axis deadbeat **time allocation** (N3), duty-weighted **average voltage** (N4) |
| Chart sample-time | INHERITED + dual ZOH | **two-rate DISCRETE**: controller @ `Tsc`, slicer @ `Ts` (see rule 4) |
| Cost (this reference) | L2 weighted | **L1 unweighted** (paper eq 7) |
| Ripple @ same `Tsc` | baseline | ~3-5x lower (paper Table 2) |

There is **no SVPWM and no Anti_Park** — the selected discrete vectors are applied directly; the gate comes from the time-slicer, not a PWM modulator.

## Must-Follow Rules

1. **Plan first.** Before any `add_block`, write a numbered plan: parameter table, design-decision choices ([design_decisions.md](references/design_decisions.md)), build-script structure. Get user approval.
2. **One-click reproducibility.** Inject all parameters via `set_param(mdl, 'InitFcn', sprintf(...))`. Model must Run from `.slx` double-click in a fresh MATLAB session. See [crit_conditions.md §J-CRIT](references/crit_conditions.md).
3. **Controller chart hardcodes machine params via `sprintf`.** The `DualVecMPC` chart embeds `Rs/Ld/Lq/psif` as numeric literals at build time; the literals MUST equal the `InitFcn` values **digit-for-digit** (assert it in the build). Never put them in an external `.m` file (drifts from the plant). See [crit_conditions.md §K-CRIT](references/crit_conditions.md).
4. **Two-rate DISCRETE sample times** (the validated structure — see [crit_conditions.md §G-CRIT](references/crit_conditions.md)):
   - `DualVecMPC` and `ThetaSrc` → `MATLABFunctionConfiguration.UpdateMethod='Discrete'`, `SampleTime='s_Tsc'` (compute once per control period and hold).
   - `TimeSlicer` → `Discrete`, `SampleTime='s_Ts'` (must run at the **fast** plant rate to slice within the period), fed by a **Digital Clock** @ `Ts`.
   - `SampleTime` is silently ignored unless `UpdateMethod='Discrete'` is set first. A continuous/inherited gate **fails to propagate** into the discrete SimPowerSystems bridge.
5. **DC bus polarity must match.** Wire `DC +(RConn) → UB RConn(1)(+)` and `DC −(LConn) → UB RConn(2)(−)`. Reversed polarity forward-biases the bridge freewheel diodes, clamps the DC link to ≈0, and the motor sees no voltage (vds=vqs=0, zero current, rotor stalls). See [anti_patterns.md](references/anti_patterns.md) #1 — this is the most common dual-vector build failure.
6. **`gate` is a 6-element COLUMN vector.** The `TimeSlicer` output must be `[Sa+;Sa−;Sb+;Sb−;Sc+;Sc−]` (column) so the inferred size `[6 1]` matches the Universal_Bridge gate port `[6]`; a `[1 6]` row triggers a back-propagation size error. See [crit_conditions.md §D-CRIT](references/crit_conditions.md).
7. **One `theta_e`, integrated at `Tsc`, feeds both Plark and the controller.** `theta_e += Tsc·(Pn·w)` in a persistent var, wrapped via `atan2(sin,cos)`. Do NOT use the PMSM bus `theta`. If routed via Goto/From, `TagVisibility='global'`. See [crit_conditions.md §A-CRIT](references/crit_conditions.md).
8. **Outer PI saturation is mandatory.** `LimitOutput='on'`, limits `[−iq_max, +iq_max]` with `1.5·Pn·psif·iq_max ≥ 1.3·TL_max`, back-calculation anti-windup. See [parameter_defaults.md](references/parameter_defaults.md).

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate inputs + sanity grid | base/[pre_build_grid.md](../motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant layer (powergui Discrete @ Ts, DC, UB Inverter, PMSM Salient-pole, TL) — **check DC polarity** | [crit_conditions.md](references/crit_conditions.md), [anti_patterns.md](references/anti_patterns.md) #1 |
| 2 | Measurement layer (BusSelector → Clark → Plark, theta_e source) | [crit_conditions.md §A](references/crit_conditions.md) |
| 3 | Outer speed PI (RPM↔rad/s, mandatory saturation) | [parameter_defaults.md](references/parameter_defaults.md), [scripts/speed_pi_design.m](scripts/speed_pi_design.m) |
| 4 | `DualVecMPC` two-stage controller chart | [algorithm_pseudocode.md](references/algorithm_pseudocode.md) + [crit_conditions.md §G/§K](references/crit_conditions.md) |
| 5 | `TimeSlicer` two-vector sequencer + Digital Clock | [algorithm_pseudocode.md](references/algorithm_pseudocode.md) §slicer + [crit_conditions.md §G/§D](references/crit_conditions.md) |
| 6 | Logging (port DataLogging or To Workspace @ Tsc) | [acceptance_criteria.md](references/acceptance_criteria.md) |
| 7 | Solver (fixed-step discrete, FixedStep = Ts; powergui Discrete) + InitFcn injection | [crit_conditions.md §J](references/crit_conditions.md) |
| 8 | Self-tests + acceptance (visual 4-check, then §E ripple vs operating point) | [acceptance_criteria.md](references/acceptance_criteria.md) |

If issues arise, consult [crit_conditions.md](references/crit_conditions.md) (A/D/G/J/K) and [anti_patterns.md](references/anti_patterns.md).

## Required User Inputs

Ask the user before starting. Defaults in [parameter_defaults.md](references/parameter_defaults.md).

| Group | Parameter |
|---|---|
| Machine | `Rs` (Ω), `Ld, Lq` (H; SPMSM: `Ld=Lq=Ls`), `psif` (V·s), `Pn`, `J` (kg·m²), `F` (N·m·s) |
| Power stage | `Vdc` (V) — BEMF margin; ripple scales with `Vdc·Tsc/L`, so it co-sets the ripple level |
| Sampling | `Ts` (plant solver, ~1 μs), `Tsc` (control period, paper 100 μs @ 10 kHz; `Tsc/Ts ≥ 50`) |
| Outer loop | `Kp_w, Ki_w` (recommend `speed_pi_design.m`; B=0 ⇒ Symmetric Optimum a=4), `iq_max` |
| MPC | cost form (`L1-unweighted` = paper / `L2-weighted` = production option), `id_ref` (SPMSM/mild-IPMSM: 0) |
| Scenario | `StopTime`, `omega_ref` profile (RPM), `TL_step_time`, `TL_value` (`< 1.5·Pn·psif·iq_max`) |

## Triggers / Skip

| ✅ Use | ❌ Skip |
|---|---|
| Build / port / extend a dual-vector (two-vector) FCS-MPC simulation in Simulink | Single-vector FCS-MPC → [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) |
| Two vectors/period, q-deadbeat time allocation, duty-cycle MPC | FOC, DTC, SMC, sensorless, scalar V/Hz, BLDC trapezoidal |
| Generalizing dual-vector MPC to a new SPMSM / mild-saliency machine | Three-or-more vectors, multi-step horizon (N>1), induction-motor MPC |
| | Strong-saliency IPMSM MTPA, weak-field; pure theory questions |

## Generalization Across Machine Sub-Types

| Sub-type | Parameter constraint | Strategy |
|---|---|---|
| SPMSM | `Ld == Lq` | `id_ref = 0`. The N2–N5 general `(Ld,Lq)` form reduces verbatim to the paper's SPMSM equations. |
| IPMSM mild saliency | `Lq > Ld`, `Lq/Ld ≤ 1.5` | `id_ref = 0` workable; the general form already carries the salient cross-terms. |
| IPMSM strong saliency | `Lq/Ld ≥ 2` | `id_ref` from MTPA (out of v1 scope; ask user) |

Topology does not change — same blocks, same wiring, same two-stage chart, same CRIT conditions. Only parameters and `id_ref` strategy differ. Out-of-scope: SynRM (`psif ≈ 0`), IM, BLDC trapezoidal — different prediction equations.

## Sibling Skills

- [motor-pmsm-base](../motor-pmsm-base/SKILL.md) — base infrastructure (this skill layers on it)
- [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) — single-vector FCS-MPC (the natural comparison baseline)
- [motor-dtc-pmsm](../motor-dtc-pmsm/SKILL.md) — Direct Torque Control alternative
- [motor-smc-pmsm](../motor-smc-pmsm/SKILL.md) — Sliding Mode Control alternative
