---
title: FOC-LTID Building Blocks Manifest
method: foc-ltid (FOC + load-torque linear estimator + feed-forward compensation)
placeholder_convention: the estimator's matrices (A,B,C,D,Q,R) are INPUT PORTS — wire Constant blocks sourced from InitFcn workspace variables; the atom carries no numeric values
topology: ⚠️ single unwired atom; no wiring, no design hint
---

# Building Blocks Manifest — `foc_ltid_blocks.slx`

> **License & trademark notice**: `foc_ltid_blocks.slx` is a **user-authored** Simulink
> library. Its single atom is a generic MATLAB Function block (pure user MATLAB code) that
> references the MathWorks Simulink `MATLAB Function` block **by reference**; it embeds no
> MathWorks block implementations and no Simscape blocks. Opening it requires your own
> licensed MATLAB + Simulink. MATLAB® and Simulink® are registered trademarks of The
> MathWorks, Inc.; this is an independent project, not affiliated with or endorsed by MathWorks.

## Why this library is minimal (one atom)

The `foc-ltid` reference plant is an **equation-based linear model** (a hand-built dq
integrator chain — no inverter, no PWM, no Simscape). Consequently:

- **dq transforms** (`Clark` / `Plark` / `Anti_Park`) are reused **by pointer** from the
  read-only base `pmsm_blocks.slx` — not duplicated here.
- the **optional Simscape switched-plant variant** reuses base `PMSM` / `Universal_Bridge` /
  `powergui` / `SVPWM` / `DC_Voltage_Source` / sensors — also **by pointer** from base.
- the **equation-linear plant**, the **speed-PI (with back-calculation anti-windup)**, the
  **dq current-PI + back-EMF decoupling**, and the **feed-forward injection** are 🟢 formula
  (`shared/formulas/pmsm_formulas.md` §0–§7, §A; `formulas/pmsm_foc_ltid_formulas.md` F1, F8) or 🔴
  topology — by rule #4 these are **built from formulas at rebuild time**, never
  shipped as pre-wired subsystems.

What remains as a genuinely method-specific, non-base, non-trivial asset is the **inline
estimator recursion** — shipped here as one atom.

## Block catalog (1 atomic block, no wiring)

| Block name in .slx | Type | Role | Formula | Placeholders |
|---|---|---|---|---|
| `KF_Estimator_EML` | MATLAB Function (Stateflow.EMChart, `SampleTime='-1'`, `ChartUpdate='INHERITED'`) | generic discrete recursive Kalman filter / observer for load-torque estimation | F6.a (recursive KF); steady-state KF = F7; ≡ fixed-gain Luenberger per F7.b | matrices via input ports (no numerics) |

### `KF_Estimator_EML` — interface

```matlab
function [est_load, omega_hat, yhat] = KF_Estimator_EML(y, u, A, B, C, D, Q, R)
```

- **Inputs (8)**:
  - `y` — measured mechanical speed `ω_m,meas` (scalar). Innovation = `y − C·x̂`.
  - `u` — input `Te` (= `Kt·iq`, the electromagnetic torque; scalar).
  - `A,B,C,D` — the **discretized** F2 state-space (`x=[ω_m; T_load]`, `u=Te`, `y=ω_m`).
    Baseline `n=2`: `A` 2×2, `B` 2×1, `C` 1×2, `D` scalar (=0). ⚠️ In R2024b these dims are
    **not** auto-inferred from all-inherited scalar inputs — pin them on the instance (see the
    "R2024b SIZING" note under Usage below); the chart stays `SampleTime='-1'`/INHERITED.
  - `Q,R` — process / measurement noise covariances (`Q` n×n, `R` scalar).
- **Outputs (3)**:
  - `est_load` — `x̂(2)` = load-torque estimate `T̂L` → feeds the F8 feed-forward path.
  - `omega_hat` — `x̂(1)` = speed estimate.
  - `yhat` — `C·x̂ + D·u` (predicted measurement, for diagnostics).
- **Internal state**: `persistent xhat, P`; recursion is the canonical predict/update
  (`P=(I−K·C)·P`, plain form, matching F6.a). No Joseph form, no numeric constants.

### How to realize each estimator variant from this one atom

- **Recursive KF (F6)** — use as-is: feed `Q,R`; the block runs the per-step Riccati `P`
  recursion and a time-varying gain.
- **Steady-state KF (F7)** — same block; `P` converges to `P∞` and the gain to `K∞`
  (or precompute `K∞` and use a fixed-gain observer instead, see next).
- **Pole-placed Luenberger (F4/F5)** — realize as a generic **Discrete State-Space** /
  small EML running `x̂[k]=Ad·x̂[k−1]+Bd·u+Ld·(y−Cd·x̂[k−1])` with a constant `Ld`
  (placed gain). F7.b establishes the steady-state KF is itself an optimally-chosen
  fixed-gain Luenberger, so this atom *and* a fixed-gain observer are two tunings of one
  structure. (No separate atom is shipped for the fixed-gain form — it is a plain linear
  block built from the F4/F5 matrices.)
- **v1 scope: ALL SEVEN variants, 1:1 coverage** (Luenberger, steady
  KF, steady colored KF, recursive KF, recursive colored KF, correlated-noise KF F9.a,
  continuous Kalman-Bucy F10). Variants 1–5 are realized from THIS atom + Discrete State-Space +
  `InitFcn` matrices; variants 6 (F9.a, cross-term `M`) and 7 (F10, continuous Riccati) are
  authored **inline from their documented formulas** at rebuild time — the library ships only this
  one generic atom (the minimal-atom decision). "1:1 = coverage, not copy": reproduce each variant's
  structure/tuning method; never copy the baseline's magic numbers or selector topology.

## Usage

```matlab
% Copy the estimator atom into your model
load_system('foc_ltid_blocks.slx');   % from this method's building_blocks/ (build scripts resolve the path)
add_block('foc_ltid_blocks/KF_Estimator_EML', 'your_model/LoadEstimator');

% Provide the discretized F2 matrices from your InitFcn workspace, e.g. via Constant blocks:
%   Ad,Bd,Cd,Dd = discretize([−B/J −1/J; 0 0],[1/J;0],[1 0],0, ts)   % (F5.a)
%   Qd = diag([q_omega q_load]);  Rd = r_omega;                       % (F6.b design knobs)
% then wire Constants(Ad,Bd,Cd,Dd,Qd,Rd) + y(ω_m,meas) + u(Te) into the 8 input ports.
%
% ⚠️ R2024b SIZING (required step):
%   with all 8 inputs inherited and scalars arriving 1-D, Stateflow cannot statically size
%   `persistent xhat=zeros(size(A,1),1)` → output-size inference FAILS. After add_block:
%   (1) localize the instance link:  set_param(blk,'LinkStatus','inactive')  (library untouched);
%   (2) pin each port Size on the instance — A=[n n], B=[n 1], C=[1 n], D=[1 1], Q=[n n],
%       R=[1 1], y=1, u=1, outputs scalar — keeping DataType = Inherit;
%   (3) keep chart SampleTime='-1', ChartUpdate='INHERITED'.
%   (n=2 baseline; n=3 for the colored-noise augmented variant.)
```

## Usage constraints

### ✅ Allowed
- Copy `KF_Estimator_EML` into your model and wire its 8 inputs.
- Build the plant, PI controllers, decoupling, and FF path from the shared formulas.
- Reuse base `pmsm_blocks.slx` atoms (transforms, optional Simscape plant) by pointer.

### ❌ Not allowed
- Don't expect plant / controller / FF topology here — it is 🟢 formula / 🔴 topology, by design.
- Don't hard-code estimator matrices into the atom — they are workspace-sourced input ports.
- Don't infer any wiring from this library (it is a single unwired atom).

## Compliance & acceptance evidence

- The atom carries **zero numeric values** (matrices are input ports).
- The atom and this doc carry no reference-specific machine/observer literals or filenames (verified by a grep gate: 0 hits).
- Build is reproducible/idempotent via `build_foc_ltid_blocks.m`.
- Reuses the read-only base `pmsm_blocks_manifest.md` by pointer; does not modify base assets.
