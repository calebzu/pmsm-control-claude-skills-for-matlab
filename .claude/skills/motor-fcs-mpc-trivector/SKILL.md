---
name: motor-fcs-mpc-trivector
description: PMSM Three-Vector Finite-Control-Set MPC Builder. Build an inner-loop three-vectors-per-period finite-control-set MPC current controller for a three-phase voltage-source-inverter-driven PMSM (SPMSM / mild-saliency IPMSM via parameterization) in Simulink, with an outer speed PI providing iq_ref. Each control period synthesizes an expected voltage vector from two adjacent active vectors + one zero vector and splits the period into three sub-intervals (t_i, t_j, t_z) so that BOTH i_d and i_q hit their references in one period (dual-axis deadbeat) — cutting current ripple below single- and dual-vector FCS-MPC at the same Tsc. Use when constructing, reproducing, porting, or extending a three-vector / triple-vector FCS-MPC current-control simulation in Simulink (keywords three-vector MPC, triple-vector MPC, TV-MPCC, dual-axis deadbeat, expected voltage vector synthesis, sector-based MPCC, 三矢量模型预测, 期望电压矢量). Skip for single-vector FCS-MPC (use motor-fcs-mpc), dual-vector / two-vector FCS-MPC (use motor-fcs-mpc-dualvector), FOC, DTC, SMC, sensorless, scalar V/Hz, BLDC trapezoidal, induction-motor MPC, multi-step-horizon (N>1) MPC, strong-saliency IPMSM MTPA, weak-field, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-fcs-mpc-trivector — PMSM Three-Vector Finite-Control-Set MPC Builder

Three-phase 2-level voltage-source inverter + PMSM (SPMSM / mild-saliency IPMSM via parameterization). Inner loop = **three-vector** FCS-MPC current control in the dq frame: each control period synthesizes an **expected voltage vector** per sector from the two **adjacent active** vectors `u_i, u_j` plus a zero vector, and splits the period into three sub-intervals — `u_i` for `t_i`, `u_j` for `t_j`, zero for `t_z = Tsc − t_i − t_j` — chosen so that **both `i_d` and `i_q` reach their references in one period (dual-axis deadbeat)**. Outer loop = speed PI providing `iq_ref`, `id_ref = 0`. Synthesizing an arbitrary-direction, arbitrary-amplitude voltage vector and nulling both axes drives switching-cycle current ripple below single- and dual-vector FCS-MPC at the same `Tsc`.

Distilled from Xu Yanping et al. 2018 (Three-Vector Model Predictive Current Control for PMSM, Trans. China Electrotech. Soc. 33(5):980-986). Control-law formulas are the signed **T2(b)–T6(b)** in [pmsm_formulas.md §F](../../../shared/formulas/pmsm_formulas.md); plant physics, prediction, speed PI, Clarke/Park, the 8-vector set and the 6-sector decode are reused by pointer from `pmsm_formulas.md` (incl. §B.4/§B.5). The q-axis slope (T2a) is reused by pointer from §E.1 (N2), and the cost criterion from §F.0 (control strategy, non-signed).

Layered on [motor-pmsm-base](../motor-pmsm-base/SKILL.md). All base discipline applies (Goto TagVisibility, Vdc/BEMF rule, Visual 4-check, broken-FOC defense).

## What makes three-vector different (vs dual-vector `motor-fcs-mpc-dualvector` and single-vector)

| Aspect | Single-vector | Dual-vector | Three-vector (this skill) |
|---|---|---|---|
| Vectors per period | 1 (held) | 2: `V_opt1` + `V_j` | **3: `u_i` + `u_j` (adjacent active) + zero** |
| Deadbeat axes | none (search only) | **q only** (scalar `t_opt1`) | **d AND q** (2×2 solve for `t_i,t_j,t_z`) |
| d-axis slope needed | no | no | **yes (T2b)** |
| Candidate set | 7/8 basic vectors | vector pairs | **6 synthesized expected vectors (one per sector)** |
| Time allocation | — | q-deadbeat (N3) | **dual-axis deadbeat 2×2 (T3)** + 3-case clamp (T5) |
| Chart sample-time | INHERITED + dual ZOH | two-rate DISCRETE | **two-rate DISCRETE**: controller/theta @ `Tsc`, slicer @ `Ts` |
| Cost (this reference) | L2 weighted | L1 unweighted | **L1 unweighted** over the 6 expected vectors (paper eq 7) |

There is **no SVPWM and no Anti_Park** — the selected discrete vectors are applied directly; the gate comes from a 3-segment time-slicer, not a PWM modulator.

## Must-Follow Rules

1. **Plan first.** Before any `add_block`, write a numbered plan: parameter table, design-decision choices ([design_decisions.md](references/design_decisions.md)), build-script structure. Get user approval.
2. **One-click reproducibility.** Inject all parameters via `set_param(mdl, 'InitFcn', sprintf(...))`. Model must Run from `.slx` double-click in a fresh MATLAB session. See [crit_conditions.md §J-CRIT](references/crit_conditions.md).
3. **Controller chart hardcodes machine params via `sprintf`.** The `TVMPCC` chart embeds `Rs/Ld/Lq/psif` as numeric literals at build time; the literals MUST equal the `InitFcn` values **digit-for-digit** (assert it in the build). See [crit_conditions.md §K-CRIT](references/crit_conditions.md).
4. **Two-rate DISCRETE sample times** (the validated structure — see [crit_conditions.md §G-CRIT](references/crit_conditions.md)):
   - `TVMPCC` and `ThetaSrc` → `MATLABFunctionConfiguration.UpdateMethod='Discrete'`, `SampleTime='Tsc'`.
   - `Slicer` → `Discrete`, `SampleTime='Ts'` (must run at the **fast** plant rate to slice three segments within the period), fed by a **Digital Clock** @ `Ts`.
   - `SampleTime` is silently ignored unless `UpdateMethod='Discrete'` is set first. A continuous/inherited gate **fails to propagate** into the discrete SimPowerSystems bridge (`Stateflow:SignalTypePropagationError`).
5. **`gate` is a 6-element COLUMN vector.** The `Slicer` output must be `[Sa+;Sa−;Sb+;Sb−;Sc+;Sc−]` (column) so the inferred size `[6 1]` matches the Universal_Bridge gate port `[6]`; a `[1 6]` row triggers `DataBackPropagationSizeSuffix`. See [crit_conditions.md §D-CRIT](references/crit_conditions.md).
6. **R2024b: set the MATLAB-Function code via the config object, not the wrong Stateflow class.** The chart class is `Stateflow.EMChart` (NOT `EMLChart`); the robust route is `get_param(blk,'MATLABFunctionConfiguration').FunctionScript = code`. See [crit_conditions.md §G-CRIT](references/crit_conditions.md), [anti_patterns.md](references/anti_patterns.md) #2.
7. **DC bus polarity must match.** `DC +(RConn) → UB RConn(1)(+)`, `DC −(LConn) → UB RConn(2)(−)`. Reversed polarity clamps the link via the freewheel diodes → `vds=vqs=0`, zero current, rotor stalls (no error). See [crit_conditions.md §DC-CRIT](references/crit_conditions.md), [anti_patterns.md](references/anti_patterns.md) #6.
8. **One `theta_e`, integrated at `Tsc`, feeds both Plark and the controller.** `theta_e += Tsc·(Pn·w)` persistent, wrapped via `atan2(sin,cos)`. Do NOT use the PMSM bus `theta`. Goto/From ⇒ `TagVisibility='global'`. See [crit_conditions.md §A-CRIT](references/crit_conditions.md).
9. **Outer PI saturation is mandatory.** `LimitOutput='on'`, `[−iq_max, +iq_max]`, `1.5·Pn·psif·iq_max ≥ 1.3·TL_max`, back-calculation anti-windup. See [parameter_defaults.md](references/parameter_defaults.md).
10. **Keep all three vectors live.** The T5 three-case clamp must degrade gracefully (drop the out-of-range vector) — not silently collapse to a single/dual vector. Confirm in acceptance that `t_i, t_j, t_z` are all meaningfully nonzero in steady state. See [anti_patterns.md](references/anti_patterns.md) #5.

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate inputs + sanity grid | base/[pre_build_grid.md](../motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant layer (powergui Discrete @ Ts, DC, UB Inverter, PMSM Salient-pole, TL) — **check DC polarity** | [crit_conditions.md §DC](references/crit_conditions.md), [anti_patterns.md](references/anti_patterns.md) #6 |
| 2 | Measurement layer (BusSelector → Clark → Plark, theta_e source) | [crit_conditions.md §A](references/crit_conditions.md) |
| 3 | Outer speed PI (RPM↔rad/s, mandatory saturation) | [parameter_defaults.md](references/parameter_defaults.md), [scripts/speed_pi_design.m](scripts/speed_pi_design.m) |
| 4 | `TVMPCC` controller chart (6-sector loop, 2×2 deadbeat, 3-case clamp, 6-candidate cost) | [algorithm_pseudocode.md](references/algorithm_pseudocode.md) + [crit_conditions.md §G/§K](references/crit_conditions.md) |
| 5 | `Slicer` 3-segment sequencer + Digital Clock | [algorithm_pseudocode.md](references/algorithm_pseudocode.md) §slicer + [crit_conditions.md §G/§D](references/crit_conditions.md) |
| 6 | Logging (To Workspace @ fast rate for ripple; scopes) | [acceptance_criteria.md](references/acceptance_criteria.md) |
| 7 | Solver (fixed-step discrete, FixedStep = Ts; powergui Discrete) + InitFcn injection | [crit_conditions.md §J](references/crit_conditions.md) |
| 8 | Self-tests + acceptance (visual 4-check + 3-vector liveness, then §E ripple) | [acceptance_criteria.md](references/acceptance_criteria.md) |

If issues arise, consult [crit_conditions.md](references/crit_conditions.md) (A/D/G/J/K/DC) and [anti_patterns.md](references/anti_patterns.md).

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
| Build / port / extend a three-vector (triple-vector) FCS-MPC simulation in Simulink | Dual-vector FCS-MPC → [motor-fcs-mpc-dualvector](../motor-fcs-mpc-dualvector/SKILL.md); single-vector → [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) |
| 3 vectors/period (2 adjacent active + zero), dual-axis deadbeat, expected-voltage synthesis | FOC, DTC, SMC, sensorless, scalar V/Hz, BLDC trapezoidal |
| Generalizing three-vector MPC to a new SPMSM / mild-saliency machine | Multi-step horizon (N>1), induction-motor MPC |
| | Strong-saliency IPMSM MTPA, weak-field; pure theory questions |

## Generalization Across Machine Sub-Types

| Sub-type | Parameter constraint | Strategy |
|---|---|---|
| SPMSM | `Ld == Lq` | `id_ref = 0`. The T2–T6 general `(Ld,Lq)` form reduces verbatim to the paper's SPMSM equations. |
| IPMSM mild saliency | `Lq > Ld`, `Lq/Ld ≤ 1.5` | `id_ref = 0` workable; the general form already carries the salient cross-terms. The d-axis deadbeat (T3) genuinely uses `Ld`. |
| IPMSM strong saliency | `Lq/Ld ≥ 2` | `id_ref` from MTPA (out of v1 scope; ask user) |

Topology does not change — same blocks, same wiring, same controller+slicer charts, same CRIT conditions. Only parameters and `id_ref` strategy differ. Out-of-scope: SynRM (`psif ≈ 0`), IM, BLDC trapezoidal — different prediction equations.

## Sibling Skills

- [motor-pmsm-base](../motor-pmsm-base/SKILL.md) — base infrastructure (this skill layers on it)
- [motor-fcs-mpc-dualvector](../motor-fcs-mpc-dualvector/SKILL.md) — dual-vector FCS-MPC (the direct predecessor; q-only deadbeat)
- [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) — single-vector FCS-MPC
- [motor-dtc-pmsm](../motor-dtc-pmsm/SKILL.md) — Direct Torque Control alternative
- [motor-smc-pmsm](../motor-smc-pmsm/SKILL.md) — Sliding Mode Control alternative
