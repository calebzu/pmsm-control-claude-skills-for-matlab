---
title: FOC-LTID R2024b Platform API Notes
method: foc-ltid (FOC + load-torque linear estimator + feed-forward compensation)
scope: R2024b Simulink/Stateflow/Control-System-Toolbox facts SPECIFIC to an equation-based linear PMSM plant + linear state-estimators. Generic PMSM/Simscape facts are reused BY POINTER from api_notes.md.
verification: every block path / function / mask-field below was probed live in R2024b (add_block + DialogParameters + exist). No reference-specific numeric values or filenames appear.
---

# R2024b Platform API Notes — `foc-ltid` (equation-linear plant + estimators)

> **License & trademark notice**: this document records **factual, public R2024b API
> conventions** (library paths, mask field names, function names) for interoperability with
> MathWorks products you must license and install yourself. No MathWorks source or model
> files are reproduced. MATLAB® and Simulink® are registered trademarks of The MathWorks,
> Inc.; this project is independent and not affiliated with or endorsed by MathWorks.

## §0 How to use this file

Method-specific companion to the base **`api_notes.md`**. The `foc-ltid` reference plant is an
**equation-based linear model** (hand-built integrator chain — no inverter, no PWM, no
Simscape) and its subject is a **linear load-torque estimator** (Luenberger / Kalman family).
So the platform facts it needs differ from the base file's Simscape-switched-plant focus.

**Reuse the base `api_notes.md` BY POINTER** for everything generic — do not duplicate it:

| Base `api_notes.md` § | Reuse for |
|---|---|
| §1 Simscape Electrical library paths | only the **optional** Simscape-plant variant |
| §2 PMSM mask fields | optional Simscape-plant variant |
| §3 PMSM bus signal names (`ias,ibs,ics,w,theta,Te`) | optional Simscape-plant variant |
| §4 Stateflow / MATLAB Function (`sfroot().find('Stateflow.EMChart')`, `ch.Script`, `ch.SampleTime`, `ch.ChartUpdate`) | the inline estimator atom — **base is authoritative** |
| §5 InitFcn parameter injection | one-click reproducibility |
| §6 Triggered-subsystem sample time | the triggered controllers/estimator |
| §7 `sim` output unpacking | Phase-6 waveform extraction |
| §8 Common errors (incl. θ_e Goto `TagVisibility='global'`) | broken-FOC defense |
| §11 / §11.1 / §11.2 Stateflow chart-config SOP (persistent-state chart `SampleTime='-1'`/`ChartUpdate='INHERITED'`; chart-corruption re-add SOP; column-vector output rule) | the inline KF/observer chart — **base is authoritative** |

The sections below add only what the base file does **not** cover.

---

## §A Equation-based linear plant — Simulink block paths (R2024b, verified)

The linear plant is assembled from generic Simulink math/continuous blocks (no Simscape).
All paths confirmed via `add_block` in R2024b:

| Block | R2024b library path |
|---|---|
| Continuous Integrator (dq currents, ω, θ) | `simulink/Continuous/Integrator` |
| Continuous State-Space (alt. plant realization) | `simulink/Continuous/State-Space` |
| `Fcn` (inline algebraic expr, e.g. Park/Clarke `u(1)*cos(u(3))…`) | `simulink/User-Defined Functions/Fcn` |
| MATLAB Function (inline algorithm) | `simulink/User-Defined Functions/MATLAB Function` |
| Triggered Subsystem (controllers/estimator @ ts) | `simulink/Ports & Subsystems/Triggered Subsystem` |
| Memory (anti-windup back-calc; unit delay of a scalar) | `simulink/Discrete/Memory` |
| Saturation (PI output limit) | `simulink/Discontinuities/Saturation` |
| Discrete-Time Integrator (discrete PI integral term) | `simulink/Discrete/Discrete-Time Integrator` |
| Discrete State-Space (fixed-gain Luenberger / steady-state KF) | `simulink/Discrete/Discrete State-Space` |
| Unit Delay | `simulink/Discrete/Unit Delay` |
| Random Number (measurement noise) | `simulink/Sources/Random Number` |

> The inline estimator atom itself ships in `foc_ltid_blocks.slx` (`KF_Estimator_EML`) — see
> `foc_ltid_blocks_manifest.md`. Copy it with `add_block` rather than rebuilding the recursion.

### §A.1 `Fcn` block function set excludes `mod`/`rem` (R2024b)

⚠️ The Simulink `Fcn` block (`simulink/User-Defined Functions/Fcn`) accepts only a restricted
expression grammar: `+ - * / ^`, parentheses, the relational/logical operators, and a fixed set
of math functions (`abs, acos, asin, atan, atan2, cos, cosh, exp, log, log10, power, sgn, sin,
sinh, sqrt, tan, tanh`). It **does not** include `mod` or `rem`. Writing `mod(u(1), 2*pi)` in an
`Fcn` block fails to parse.

- **Where this bites:** wrapping the electrical angle `θ_e` into `[0, 2π)` (or `[−π, π)`) is the
  natural place to reach for `mod` — and the natural place to put it is an inline `Fcn`. It won't
  compile there.
- **Fix:** use a **`Math Function`** block (`simulink/Math Operations/Math Function`,
  `Function = 'mod'`) for the wrap, or compute the wrap inside a `MATLAB Function` block (whose
  full MATLAB grammar does include `mod`/`rem`). The unbounded-angle alternative — leave `θ_e`
  unwrapped and let downstream `cos`/`sin` absorb it — is valid for short runs but accumulates a
  large argument over long simulations; wrap with `Math Function` for production-length runs.

---

## §B Mask field names (R2024b, probed via DialogParameters)

Exact field names for `set_param`. Inject **workspace variables**, never numeric literals.

### Continuous Integrator — `simulink/Continuous/Integrator`
`InitialCondition`, `ExternalReset`, `InitialConditionSource`, `LimitOutput`,
`UpperSaturationLimit`, `LowerSaturationLimit`, `ShowSaturationPort`.
- dq-current / speed / angle integrators: set `InitialCondition` (start-at-rest → `'0'`).

### Discrete-Time Integrator — `simulink/Discrete/Discrete-Time Integrator`
`gainval`, `IntegratorMethod`, `SampleTime`, `InitialCondition`, `InitialConditionSource`,
`ExternalReset`.
- `gainval` = the integrator gain (use a workspace var, e.g. the PI `Ki`).
- `IntegratorMethod` default is `'Integration: Forward Euler'`; options also include
  `'Integration: Backward Euler'` and `'Integration: Trapezoidal'`.
- `SampleTime` = the controller sample period workspace var (e.g. `'ts'`).

### Discrete State-Space — `simulink/Discrete/Discrete State-Space`
`A`, `B`, `C`, `D`, `X0`, `SampleTime`.
- Use for a **fixed-gain Luenberger observer** or **steady-state KF**: set `A`,`B`,`C`,`D` to
  the *augmented observer* matrices `(A−L·C)`-form (or the prediction/correction realization)
  as **workspace variable names** computed in `InitFcn` (see §C). `X0` = initial state est.

### Memory — `simulink/Discrete/Memory`
`X0` (initial output), `InheritSampleTime`.
- Back-calculation anti-windup on the speed PI uses a `Memory` to break the algebraic loop
  between the saturation output and the integrator correction (base §A speed-PI pattern).

### Random Number — `simulink/Sources/Random Number`
`Mean`, `Variance`, `Seed`, `SampleTime`.
- Measurement-noise injection (speed/current sensors). `Variance` = a workspace var; set a
  fixed `Seed` for reproducible Phase-6 comparisons; `SampleTime` faster than the controller
  (e.g. `'ts/2'`) to approximate white noise — this is the Kalman filter's reason to exist.

---

## §C Control System Toolbox functions for estimator design (R2024b, verified `exist`)

Computed in `InitFcn` (or a build helper) to populate the observer/KF matrices the model
consumes. All confirmed available in R2024b:

| Function | `exist` | Use |
|---|---|---|
| `c2d` | 2 (`.m`) | **preferred**: discretize an `ss(A,B,C,D)` plant — `c2d(sys,ts,'tustin')` / `'zoh'` |
| `c2dm` | 2 (legacy `.m`) | older matrix-form discretizer `c2dm(A,B,C,D,ts,'tustin')`; still present but **prefer `c2d` on an `ss` object** for new code |
| `place` | 2 | Luenberger pole placement: `L = place(A',C',poles)'` (dual form) |
| `lqe` | 2 | steady-state Kalman gain (continuous CARE): `L = lqe(A,G,C,Q,R)` |
| `dlqe` | 2 | discrete steady-state Kalman gain (DARE) — alternative to `lqe`+`c2d` |
| `kalman` | 0 (`@ss` method) | object-form Kalman estimator design (takes an `N` cross-term arg for correlated noise) |

> **Note — `exist` codes:** all six are toolbox `.m` files or class methods, **not** built-ins,
> so `exist` returns `2` (file on path) — or `0` for `kalman`, an `@ss` class method that is
> `which`-resolvable but invisible to `exist`. **All six are available with Control System Toolbox
> installed**; the codes above are R2024b-measured (an earlier draft listed several as `5`).
>
> **Convention note (matches formulas F5/F7):** the reference-style path is *continuous design
> then discretize* (`place`/`lqe` in continuous time → `c2d`/`c2dm` to `ts`). `dlqe` (design
> directly in discrete) is the equivalent alternative. Pin whichever the rebuild uses in its
> `summary.md` design-decision table; both are valid (this resolves the F7 build-time flag).

---

## §D One-click reproducibility for the estimator (method-specific)

The classic failure mode for this method: observer/KF matrices live in the base workspace but
the model's `InitFcn` does **not** compute them, so the `.slx` is not self-contained. Two
compliant fixes (Phase-5 acceptance #2 — pick one):

1. **Inline-EML estimator** (preferred, what `KF_Estimator_EML` enables): the recursion
   computes / receives its matrices through input ports fed by `Constant` blocks whose values
   are workspace vars set in `InitFcn` — no external script, fully one-click.
2. **InitFcn-computed matrices** for a `Discrete State-Space` fixed-gain observer: put the
   `place`/`lqe`/`c2d` calls **inside the `InitFcn` string** so loading the model regenerates
   the matrices. Do not rely on a separately-run design script.

> Either way: **all numeric values originate from `InitFcn` workspace variables**, never
> hard-coded in a block mask or chart literal.

---

## §E Common errors specific to this method

| Symptom | Cause | Fix |
|---|---|---|
| model needs a script run before sim | observer matrices not in `InitFcn` | move `place`/`lqe`/`c2d` into `InitFcn`, or use the inline-EML atom (§D) |
| `Discrete State-Space` size error | `A`,`B`,`C`,`D` dimensions inconsistent (augmented 2- vs 3-state) | confirm the augmented state order `x=[ω_m; T_load]` (F2); colored-noise adds a 3rd state (F9.b) |
| KF estimate biased / drifts | discrete `Q,R` scaled wrong vs continuous intensities | mind `Q≈G·Qc·Gᵀ·ts`, `R≈Rc/ts` (formulas F10.a bridge) when moving between `lqe` and discrete |
| `KF_Estimator_EML` "invalid dimension specified for input 'y'" / output-size inference fails | atom's `persistent xhat=zeros(size(A,1),1)` cannot be statically sized when all 8 inputs are inherited 1-D scalars | apply the **§E.1 SIZING recipe** below (localize link + pin port `Size`) — required for both the `n=2` baseline and the `n=3` colored variants |
| chart `SampleTime` empty after edits | persistent-state chart property corruption | base `api_notes.md` §11.1 re-add SOP; keep `SampleTime='-1'`, `ChartUpdate='INHERITED'` |
| measurement noise not reproducible | `Random Number` seed unset | set a fixed `Seed` workspace var |

### §E.1 `KF_Estimator_EML` R2024b SIZING recipe (required wiring step)

The atom infers its state dimension at runtime via `n = size(A,1)` and allocates
`persistent xhat = zeros(n,1)`. In R2024b, when the instance is added with all 8 inputs left at
the inherited default and the wired signals arrive as 1-D scalars/short vectors, **Stateflow
cannot statically determine `n`** at compile time, so output-size inference fails (typical error:
`port width / dimension error: invalid dimension specified for input y`). This is a platform inference limitation, not a
flaw in the recursion. The working recipe (verified live on the rebuild):

```matlab
% After add_block of the atom instance:
set_param(blk, 'LinkStatus', 'inactive');                 % (1) localize — library source untouched
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk); ch = ch(1);
d  = ch.find('-isa','Stateflow.Data');
% (2) pin each port Size on the INSTANCE (DataType stays Inherit). n=2 baseline shown;
%     use n=3 for the colored-noise augmented variant (x=[ω_m; T_L; n]).
szmap = struct('y','1','u','1', ...
               'A','[2 2]','B','[2 1]','C','[1 2]','D','1','Q','[2 2]','R','1', ...
               'est_load','1','omega_hat','1','yhat','1');
for ii = 1:numel(d)
    nm = d(ii).Name;
    if isfield(szmap, nm), d(ii).Props.Array.Size = szmap.(nm); end
end
% (3) keep the chart itself INHERITED: ch.SampleTime='-1', ch.ChartUpdate='INHERITED'.
```

- Pin **`Size` only** — leave `DataType = Inherit`. Forcing `DataType` re-introduces the base
  `api_notes.md` §11 type-propagation deadlock.
- For the **`n=3`** colored variant, widen to `A=[3 3]`, `B=[3 1]`, `C=[1 3]`, `Q=[3 3]`
  (keep `D=1`, `R=1`, `y=1`, `u=1`, outputs scalar). Use a separate localized instance per `n`.
- This complements the §D one-click rule: the matrices still arrive through input ports fed by
  `Constant` blocks sourced from `InitFcn` — the SIZING step adds no numerics, only port shapes.

---

## §F Acceptance

- Contains **zero reference-specific numeric values** and **zero reference-model filenames** —
  only R2024b platform facts, all probed live.
- Reuses base `api_notes.md` and `pmsm_blocks_manifest.md` **by pointer**; does not modify the
  read-only base assets.
