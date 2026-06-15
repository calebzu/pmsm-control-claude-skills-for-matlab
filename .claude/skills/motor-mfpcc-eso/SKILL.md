---
name: motor-mfpcc-eso
description: PMSM Model-Free Predictive Current Control + ESO Builder. Build an inner-loop model-free DEADBEAT predictive current controller in the stationary αβ frame — first-order ultralocal model (ẏ = F + α·u) with a linear Extended State Observer estimating the lumped disturbance F — modulated by SVPWM at fixed switching frequency, with an outer speed PI providing iq_ref, for a three-phase voltage-source-inverter-driven SPMSM in Simulink. The current control law uses NO machine parameters (no Rs/Ls/ψf) — only the input gain α and the ESO bandwidth ω0. Use when constructing, reproducing, porting, or extending an MFPCC / ultralocal-model / ESO-based deadbeat current-control simulation in Simulink (keywords MFPCC, model-free predictive current control, ultralocal model, extended state observer, ESO, deadbeat predictive current control, ADRC-style current loop, 无模型预测电流控制, 扩张状态观测器). Skip for FCS-MPC of any vector count (single/dual/three-vector — this method is continuous-set deadbeat + SVM, NOT FCS-MPC; use the motor-fcs-mpc family), FOC PI current loops, DTC, SMC speed loops, sensorless, scalar V/Hz, BLDC trapezoidal, induction-motor predictive control, IPMSM with saliency, weak-field / MTPA, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-mfpcc-eso — PMSM Model-Free Deadbeat Predictive Current Control + ESO Builder

Three-phase 2-level voltage-source inverter + SPMSM. Inner loop = **model-free deadbeat predictive
current control in the stationary αβ frame**: plant abstracted as the first-order ultralocal model
`di_s/dt = F + α·u_s`, a **linear ESO** estimating current + lumped disturbance `F` (absorbs
resistance, back-EMF, all parameter variation), and a closed-form **deadbeat inversion** with
one-step delay compensation producing a continuous `(uα, uβ)` command that **SVPWM** modulates at
fixed switching frequency. Outer loop = speed PI providing `iq_ref`; `id_ref = 0`. Only **2 tuned
parameters**: `α` and `ω0` (via pole `z12`). Distilled from Y. Zhang, J. Jin, L. Huang, *IEEE Trans.
Ind. Electron.* 68(2):993-1002, 2021 (DOI 10.1109/TIE.2020.2970660). Layered on
[motor-pmsm-base](../motor-pmsm-base/SKILL.md); all base discipline applies (Visual 4-check, broken-FOC defense).

## ⚠ Method Identity (PINNED)

**Deadbeat PCC + ESO + SVM = CONTINUOUS control set. NOT FCS-MPC.** No vector enumeration, no cost
function, no switching-state selection — voltage solved algebraically, handed to a modulator. Do NOT
layer this on the motor-fcs-mpc family. Full evidence: formulas §D.

## Formula Sources (single source of truth)

- Method formulas: [pmsm_formulas.md §G](../../../shared/formulas/pmsm_formulas.md) — ultralocal model
  (4)/(7), ESO (8)–(17), deadbeat (18)–(21), method identity, parameters. Plant by reference:
  [pmsm_formulas.md §8](../../../shared/formulas/pmsm_formulas.md) (αβ complex-vector SPMSM;
  `Ld = Lq = Ls`). ESO derivation + tuning rationale: [eso_derivation.md](references/eso_derivation.md).

## Must-Follow Rules

1. **Plan first** — numbered plan with parameter table and build-script structure. Get user approval.
2. **One-click reproducibility** — all parameters injected via `set_param(mdl,'InitFcn',...)`;
   `wref`/`TL` profiles as From-Workspace **inline expressions**. See [build_sop.md](references/build_sop.md).
3. **Model-free boundary + literal params** — the `MFPCC_ESO` chart uses ONLY `alpha, beta01,
   beta02, Tsc` (embedded as build-time numeric literals == InitFcn, asserted; bare workspace names
   do NOT resolve in charts). `Rs/Ls/psi_f` inside the chart voids the method. See [chart_contract.md](references/chart_contract.md).
4. **Chart I/O contract is fixed** — `MFPCC_ESO`: 6 in `(ialpha, ibeta, theta_e, omega_e, iq_ref,
   id_ref)` → 6 out `(ua, ub, iq_meas, id_meas, Fa_o, Fb_o)`. See [chart_contract.md](references/chart_contract.md).
5. **Double-`F` warning** — PMSM mask `F` = viscous friction `B` (plant); ESO state `F` = lumped
   disturbance (controller). Same letter, unrelated quantities. Never cross-assign.
6. **Glue: inputs = six ZOH @ Tsc, output = Unit Delay @ Tsc (NOT ZOH)** — the `ua/ub` Unit Delays
   realize the 1-step delay that deadbeat eqs (20)/(21) compensate; a ZOH applies `u(k)` in the same
   cycle → horizon broken → divergence. Chart stays INHERITED. See [chart_contract.md](references/chart_contract.md) §3.
7. **ESO gains by closed form, asserted in build** — `omega0 = (1 − z12)/Tsc`, `beta01 = 2·omega0·Tsc`,
   `beta02 = omega0²·Tsc`. Default `z12 = 0.15`, `alpha = 50`. See [eso_derivation.md](references/eso_derivation.md) §3.
8. **All-discrete solver** — model `FixedStepDiscrete` @ `Ts` + powergui `Discrete` @ `Ts`;
   `SpeedPI` chart DISCRETE @ `Tsc`; re-commit both chart configs after wiring. See [build_sop.md](references/build_sop.md) §5.
9. **SVPWM workspace contract + sector-7 fix** — the library SVPWM reads `Tpwm` and `Vdc_val` from
   workspace; carrier is hardcoded for `Tpwm = 1e-4`; apply the MultiPortSwitch sector-7 startup fix
   on the local instance. See [build_sop.md](references/build_sop.md) §4.
10. **`id` bias grows ∝ ωe² — expected physics, not a bug** — do not "fix" it during acceptance; check
    it stays inside the documented envelope. See [known_limitations.md](references/known_limitations.md) #1.

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate inputs + sanity grid | base/[pre_build_grid.md](../motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant layer (powergui Discrete, DC, UB, PMSM, TL inline-FW, BusSelector) + measurement (Clark abc→αβ, `Gain_Pn` ×2 for θe/ωe) | [build_sop.md](references/build_sop.md) §2-§3 |
| 2 | Outer `SpeedPI` chart (PI + anti-windup clamp ±iq_max) | [chart_contract.md](references/chart_contract.md) §2 |
| 3 | `MFPCC_ESO` chart (two-channel ESO + deadbeat + delay comp) | [chart_contract.md](references/chart_contract.md) §1 |
| 4 | Glue: 6 input ZOH @ Tsc + 2 output Unit Delay @ Tsc | [chart_contract.md](references/chart_contract.md) §3 |
| 5 | Modulation (SVPWM library + sector-7 fix) → UB gate | [build_sop.md](references/build_sop.md) §4 |
| 6 | Loggers (10 To Workspace) + Scope; solver + InitFcn + chart re-commit guard | [build_sop.md](references/build_sop.md) §5-§6 |
| 7 | Acceptance (Layer 1 physics + Layer 2 metrics) | [acceptance_criteria.md](references/acceptance_criteria.md) |

## Required User Inputs

Ask the user before starting. Validation-case defaults in [build_sop.md](references/build_sop.md) §7.

| Group | Parameters |
|---|---|
| Machine (7) | `Rs`, `Ls` (= Ld = Lq, SPMSM), `psi_f`, `Pn`, `J`, `B` (→ PMSM mask `F`), `Vdc` |
| Control (5) | `alpha` (default 50), `z12` (default 0.15), `iq_max`, `Kp_w`, `Ki_w` |
| Sampling (3) | `Ts` (plant, default 1e-6), `Tsc` = `Tpwm` (control = switching, default 1e-4) |
| Scenario (5) | `wref_rpm`, `ramp_t`, `TL_val`, `TL_t`, `StopTime` |

## Triggers / Skip

| ✅ Use | ❌ Skip |
|---|---|
| Build / port / extend MFPCC-ESO (ultralocal + ESO + deadbeat + SVM) in Simulink | FCS-MPC single/dual/three-vector → motor-fcs-mpc family |
| Model-free / parameter-free current loop with fixed switching frequency | FOC PI loops, DTC, SMC, sensorless, scalar V/Hz, BLDC |
| Generalizing MFPCC-ESO to a new SPMSM parameter set | IPMSM saliency, MTPA, weak-field; IM predictive control; pure theory |

## Generalization Across Machine Sub-Types

| Sub-type | Constraint | Verdict |
|---|---|---|
| SPMSM | `Ld = Lq = Ls` | ✅ supported; `id_ref = 0` |
| IPMSM (any saliency) | `Ld ≠ Lq` | ❌ out of scope — saliency introduces 2θe coupling that breaks the αβ ultralocal premise (pmsm_formulas §8) |
| any SPMSM param set | — | topology unchanged (same blocks/wiring/charts); `alpha`/`z12` defaults transfer (parameter-free law) |

## Sibling Skills

- [motor-pmsm-base](../motor-pmsm-base/SKILL.md) — base infrastructure; [motor-smc-pmsm](../motor-smc-pmsm/SKILL.md) — closest architecture (outer + inner loop + SVPWM + UB)
- [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) / dualvector / trivector — FCS-MPC family (different identity: finite set); [motor-dtc-pmsm](../motor-dtc-pmsm/SKILL.md) — DTC
