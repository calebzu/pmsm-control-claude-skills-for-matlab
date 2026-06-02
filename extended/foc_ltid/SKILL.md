---
name: motor-foc-ltid-pmsm
description: PMSM FOC + Load-Torque Estimator + Feed-Forward Builder. Build a field-oriented speed controller for a three-phase voltage-source-inverter-driven PMSM (SPMSM / mild-saliency IPMSM via parameterization) in Simulink, augmented with a linear load-torque state estimator (Luenberger + Kalman-filter family, 7 selectable variants on one shared structure) whose estimate is fed forward into the q-axis current reference to reject load-step speed dips. dq current PI ×2 with back-EMF decoupling, Symmetric-Optimum speed PI with anti-windup, selectable equation-linear plant or Simscape switched plant. Use when constructing, reproducing, porting, or extending an FOC speed loop with disturbance/load-torque estimation and feed-forward compensation (keywords FOC, field-oriented control, load-torque observer, load torque estimator, Luenberger observer, Kalman filter load estimation, feed-forward disturbance compensation, LTID, 负载转矩观测器, 前馈补偿, 卡尔曼). Skip for FCS-MPC, DTC, SMC, sensorless/position estimation, scalar V/Hz, BLDC trapezoidal, induction-motor FOC, strong-saliency IPMSM MTPA, weak-field/flux-weakening, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-foc-ltid-pmsm — PMSM FOC + Load-Torque Estimator + Feed-Forward Builder

Three-phase 2-level voltage-source inverter + PMSM. **Field-oriented speed control** (outer Symmetric-Optimum speed PI → `iq_ref`; inner dq current PI ×2 with cross-coupling + back-EMF decoupling feed-forward; `id_ref = 0`), augmented with a **linear load-torque state estimator** on the model `x = [ω_m; T_L]`, `u = T_e`, `y = ω_m` — **seven selectable realizations** on one shared structure (pole-placed Luenberger, steady-state KF, steady colored KF, recursive KF, recursive colored KF, correlated-noise KF, continuous Kalman-Bucy). The estimate `T̂_L` is **fed forward** into `iq_ref` (via the torque constant) to cut the load-step speed dip the speed PI alone cannot. v1 baseline supports SPMSM and mild-saliency IPMSM (`Lq/Ld < 2`, `id_ref = 0`); strong-saliency MTPA deferred.

Layered on [motor-pmsm-base](../../.claude/skills/motor-pmsm-base/SKILL.md). All base discipline applies (plant build standard, dq conventions, building-blocks SOP, broken-FOC defense, visual 4-check).

## ⚠️ Estimator validation note

The base FOC plant + dq current PI + speed PI formulas are reused by pointer from the base `pmsm_formulas.md` §0–§7, §A. The **load-torque estimator family** (F2–F10: augmented model, observability, Luenberger, recursive/steady/colored/correlated KF, Kalman-Bucy) is documented in `formulas/pmsm_foc_ltid_formulas.md`, each formula cross-checked against ≥2 independent academic sources (see base/[theory_anchor.md](../../.claude/skills/motor-pmsm-base/references/theory_anchor.md)).

**Validation requirement**: a visual review on the MATLAB desktop (motor rotates / `iq` tracks / `iabc` AC sinusoidal / `Te` energy balance) **plus** the FF-on-vs-FF-off dip-reduction demonstration. Neither can be skipped.

## Must-Follow Rules

1. **Plan first** — numbered plan with the input table (below), the design-decision choices (discretization path, KF tuning, plant choice — see the build template's parameter block), and the build-script structure. Get user approval before the first `add_block`.
2. **One-click reproducibility** — every numeric originates in the model `InitFcn` (machine, PI gains, **all** estimator matrices via `c2d`/`place`/`dlqe`, the augmented `n=3` colored model, the F9.a cross-term, the F10 continuous intensities, plus guarded `ff_enable`/`est_sel`/`plant_sel` defaults). No external design script may need to run first. See base one-click discipline.
3. **The six structural modules are non-negotiable** — (1) plant exposing `ω_m, θ_e, i_abc/i_dq, T_e` + `T_L` input; (2) outer speed PI (saturation + anti-windup); (3) inner dq current PI ×2 with decoupling/back-EMF FF (`id_ref=0`); (4) Clarke/Park transforms on a single global `θ_e`; (5) load-torque estimator on `x=[ω_m;T_L]`; (6) feed-forward of `T̂_L` into `iq_ref` with an enable. Omitting any one breaks the method.
4. **`θ_e` Goto `TagVisibility='global'` is MANDATORY in every plant variant** — the base `Anti_Park`/`Plark` consume `θ_e` via an internal `From "The"`; a default `'local'` Goto is the classic silent failure (FOC degenerates to lab-frame open-loop: motor stalls, `iabc` DC-locked). A build self-test must assert global visibility on every `The` Goto (look-under-masks). See base/[broken_foc_diagnostics.md](../../.claude/skills/motor-pmsm-base/references/broken_foc_diagnostics.md) §F-CRIT 1.
5. **Estimator on the augmented model + observability check** — build `x=[ω_m;T_L]`, `u=T_e`, `y=ω_m` with `ṪL≈0` (random-walk internal model, F2). Assert observability `rank(O)=2` (F3, `det≈-1/J≠0`) in `InitFcn` — a rank-deficient model is a build error, not a tuning issue.
6. **All 7 variants on ONE shared structure = coverage, NOT copy** — reproduce the variant *structure/tuning method* from F4–F10; never copy magic Q/R/pole numbers or a selector topology from any source. Realize the recursive KFs **inline** (`KF_Estimator_EML` atom / inline MATLAB Function) so the model is one-click. The steady-state KF must collapse onto the Luenberger (F7.b) — verify their `T̂_L` std match as a built-in cross-check.
7. **Feed-forward enable is mandatory** — `iq_ref += (T̂_L/Kt)·ff_enable`, with `ff_enable` toggling FF-on vs FF-off. The headline demonstrable effect is **FF-on reduces the load-step speed dip and recovery time**; FF-off is the baseline. This is the method's reason to exist (see misconception #3).
8. **Speed PI = Symmetric Optimum, not pole-zero-cancellation** — a PZC/IMC-tuned speed PI has a disturbance-rejection slow pole fixed by the plant mechanical ratio `−B/J`, **independent of bandwidth** (the Phase-4.5 trap: large inertia → long load-settling no matter the gain). Use Symmetric Optimum (`a=4`, `Teq=1/αc`) so rejection is bandwidth-dependent, and verify the evaluation window `T_window ≥ 5·τ_max` (`τ_max=2·a·Teq`). See base Phase-4.5 sanity check.
9. **Reproducible measurement noise (fixed seed)** — inject `ω_m,meas = ω_m + n` with a fixed-seed `Random Number` at a sub-`ts` rate. The sensor noise is the KF's reason to exist; without it a plain observer suffices and the Kalman story is unmotivated.
10. **R2024b estimator sizing recipe + visual 4-check** — the `KF_Estimator_EML` atom and the inline persistent/continuous charts (KF6/KF7) need their port `Size` **pinned** (DataType=Inherit, chart `SampleTime='-1'`); R2024b output-size inference fails for `persistent xhat=zeros(size(A,1),1)` and continuous-feedback charts. See `building_blocks/foc_ltid_api_notes.md` §E.1. Run the base visual 4-check before trusting any metric.

## Build Flow

Layered on the base flow ([motor-pmsm-base](../../.claude/skills/motor-pmsm-base/SKILL.md) Build Flow); method-specific steps:

| Phase | Action | Where |
|---|---|---|
| 0 | Validate inputs + base sanity grid (Vdc/BEMF, Goto visibility, FF dimensions, sector-7) | base/[pre_build_grid.md](../../.claude/skills/motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant: **Variant Subsystem** (`plant_sel`) — EQN dq integrator chain ‖ Simscape switched (base atoms by pointer); `θ_e` Goto global in **each** variant | base/[plant_modeling.md](../../.claude/skills/motor-pmsm-base/references/plant_modeling.md) |
| 2 | Speed-sensor noise (`ω_m,meas = ω_m + n`, fixed seed) | (this skill) Rule 9 |
| 3 | Outer speed PI (Symmetric Optimum + back-calculation anti-windup + `iq` saturation) | `../../shared/formulas/pmsm_formulas.md` §A |
| 4 | Inner dq current PI ×2 (IMC) + cross-decoupling/back-EMF FF; `id_ref=0` | `formulas/pmsm_foc_ltid_formulas.md` F1 |
| 5 | Estimator bank: augmented F2 model, observability (F3), **7 variants** on one selector (F4–F10) | `formulas/pmsm_foc_ltid_formulas.md` F2–F10 |
| 6 | Feed-forward: `T̂_L` → `1/Kt` → `×ff_enable` → summed into `iq_ref` (F8) | `formulas/pmsm_foc_ltid_formulas.md` F8 |
| 7 | Modulation (Simscape variant only): Anti_Park + SVPWM + Universal Bridge | base/[building_blocks.md](../../.claude/skills/motor-pmsm-base/references/building_blocks.md) |
| 8 | Logger (speed / TLhat ×7 / cur / trq / iabc) + `θ_e` Goto self-test | base/[measurement_logger.md](../../.claude/skills/motor-pmsm-base/references/measurement_logger.md) |
| 9 | Solver (`ode23tb` var-step, `MaxStep=ts/2`) + InitFcn injection + idempotent self-tests | (this skill) Rule 2 |
| 10 | Visual 4-check + FF-on/off dip-reduction demo | base/[sanity_visual.md](../../.claude/skills/motor-pmsm-base/references/sanity_visual.md) |

The conventional build script is `scripts/build_template.m` (parameterized; edit the `InitFcn` parameter block for your machine).

## Required User Inputs

Ask before starting. The build template carries generic SPMSM defaults; override for the target machine.

| Group | Parameters |
|---|---|
| **Machine** (7) | `Rs`, `Ld`, `Lq` (mild-saliency `Lq/Ld < 2` v1), `psif`, `pn`, `J`, `B` (viscous; `Kt=1.5·pn·psif` derived) |
| **Current PI** (2) | `alpha_c` (IMC bandwidth, default 2000 rad/s), `Vmax` (dq voltage saturation) |
| **Speed PI** (2) | `a_so` (Symmetric-Optimum factor, default 4), `iq_max` (saturation + Lyapunov-free bound) |
| **Estimator** (≈6) | observer poles (variant 1), `Qd`/`Rd` (variant 4; `Rd=noise_var` honest), colored pole `a_col` + `Q3`/`R3` (3/5), cross-term `Mcorr` (6), `Qc`/`Rc` bridge (7) — all generic, documented as illustrative |
| **Noise** (3) | `noise_var`, `noise_seed`, `ts_noise` (default `ts/2`) |
| **Sampling** (1) | `ts` (controllers/estimators discrete period, default 1e-4) |
| **Plant select** (1) | `plant_sel` (1 = equation-linear baseline, 2 = Simscape switched) |
| **Run select** (2) | `est_sel` (1..7), `ff_enable` (0/1) |
| **Scenario** (4) | `wref_val`, `t_ramp`, `tL_on`/`tL_off`/`TL_val`, `StopTime` |
| **Simscape** (3, only `plant_sel=2`) | `Vdc_val`, `Tpwm`, `Ts` (powergui discrete step) |

## Estimator variants (formula → realization)

All on `x=[ω_m;T_L]` (colored variants augment to `n=3`); all feed the same F8 FF path. **1:1 = coverage, not copy.**

| # | Variant | Formula | Realization |
|---|---|---|---|
| 1 | Pole-placed Luenberger | F4/F5 | `Discrete State-Space`, prediction form `(Ad−Lp·Cd)`, `Lp=place(Ad',Cd',z)'` |
| 2 | Steady-state KF | F7 (DARE) | **same DSS realization** as #1, gain `Lp=Ad·M`, `M=dlqe(Ad,I,Cd,Qd,Rd)` → demonstrates F7.b |
| 3 | Steady colored KF | F7 + F9.b case B | `n=3` augmented (Gauss-Markov meas-noise state), `dlqe` constant gain |
| 4 | Recursive KF | F6 | `KF_Estimator_EML` atom, `n=2`, per-step Riccati |
| 5 | Recursive colored KF | F6 + F9.b case B | 2nd `KF_Estimator_EML` instance sized `n=3` |
| 6 | Correlated-noise KF | F9.a | inline MATLAB Function, gain carries cross-term `M` |
| 7 | Continuous Kalman-Bucy | F10 | inline MATLAB Function (continuous Riccati ODE) + continuous `Integrator` |

> **F9.b colored variants use case B** (shape the *measurement* noise, keep `T_L` as state 2): the shared atom hardcodes `est_load=xhat(2)`, so the shaping state must be appended (case A would relocate `T_L`). Both placements are valid F9.b.

## Triggers / Skip

| ✅ Use | ❌ Skip |
|---|---|
| Build / port / extend an FOC speed loop with load-torque estimation + feed-forward | FCS-MPC, DTC, SMC, sensorless/position estimation, scalar V/Hz, BLDC trapezoidal |
| Luenberger / Kalman-family load-torque observer + FF disturbance rejection | Induction-motor FOC, SynRM (`ψ_f≈0`), strong-saliency IPMSM MTPA, weak-field |
| Demonstrating FF-on vs FF-off speed-dip reduction at a load step | Pure theory (observability proofs, KF derivation), paper writing |
| Generalizing the method to a new PMSM machine parameter set | MATLAB perf tuning, unit tests |

## Generalization Across Sub-Types

| Sub-type | Constraint | Strategy |
|---|---|---|
| SPMSM | `Ld ≈ Lq` | `id_ref = 0` |
| IPMSM mild | `Lq > Ld`, `Lq/Ld < 2` | `id_ref = 0` workable |
| IPMSM strong | `Lq/Ld ≥ 2` | MTPA mid-loop required (out of v1 scope) |

Topology is invariant: same six modules, same estimator structure, same FF path, same CRIT conditions. The **plant** can be swapped between the equation-linear baseline (clean mechanical dynamics for the estimator study) and the Simscape switched plant (realism; survives PWM ripple) via `plant_sel` with no controller change — the estimator/FF layer is plant-agnostic.

## Shareable assets (by pointer — do not duplicate)

- `../../shared/formulas/pmsm_formulas.md` — base PMSM physics §0–§7 + speed-PI §A (signed).
- `formulas/pmsm_foc_ltid_formulas.md` — method formulas **F1** (dq current-PI IMC + decoupling), **F2** (augmented state model), **F3** (observability), **F4/F5** (Luenberger), **F6** (recursive KF), **F7** (steady-state KF ↔ Luenberger equivalence), **F8** (feed-forward law), **F9.a/F9.b/F10** (correlated / colored / continuous).
- `../../shared/building_blocks/pmsm_blocks.slx` (+ manifest) — base atoms (Clarke/Park/Anti_Park transforms, Simscape PMSM/Universal_Bridge/SVPWM/sources/powergui).
- `building_blocks/foc_ltid_blocks.slx` (+ manifest) — the `KF_Estimator_EML` atom (generic discrete KF recursion; matrices via input ports, zero numerics).
- `building_blocks/foc_ltid_api_notes.md` — R2024b platform facts (§A.1 Fcn-no-mod/rem; §E.1 KF sizing recipe).

## Sibling Skills

- [motor-pmsm-base](../../.claude/skills/motor-pmsm-base/SKILL.md) — base infrastructure
- [motor-fcs-mpc](../../.claude/skills/motor-fcs-mpc/SKILL.md) — FCS-MPC current control
- [motor-dtc-pmsm](../../.claude/skills/motor-dtc-pmsm/SKILL.md) — Direct Torque Control
- [motor-smc-pmsm](../../.claude/skills/motor-smc-pmsm/SKILL.md) — Sliding Mode Control speed loop
