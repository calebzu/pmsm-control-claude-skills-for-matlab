# CRIT Conditions — dual-vector FCS-MPC (must-pass build invariants)

Violating any of these produces a model that either fails to compile or silently runs broken. Each was hit and resolved in the validated Phase-5 rebuild; see [anti_patterns.md](anti_patterns.md) for the failure signatures.

## §A-CRIT — single `theta_e`, integrated at `Tsc`, global tags
- One `theta_e = ∫ Tsc·(Pn·w)` (persistent, wrapped `atan2(sin,cos)`), feeding **both** `Plark` and the controller vector Park. Do NOT use the PMSM measurement-bus `theta` (R2024b bus-consistency caveat).
- `ThetaSrc` must be **DISCRETE @ Tsc**. At the fast rate it adds `Tsc·we` every `Ts` and over-integrates the angle by `Tsc/Ts` → the motor builds a wrong field, ripple/torque collapse.
- Any Goto/From carrying `theta_e` (or any signal): `TagVisibility='global'`. Default local tag ⇒ From reads nothing ⇒ open-loop lab-frame (classic broken-FOC, base `broken_foc_diagnostics.md`).
- **Override of the base grid:** `motor-pmsm-base/pre_build_grid.md` suggests taking dq feedback from the PMSM internal measurement (`BusSel iqs/ids`). For dual-vector that is WRONG — the controller's vector Park and the current feedback must share the SAME externally-integrated `theta_e`, so feed currents through `Clark → Plark(theta_e)`. Use the external transform chain, not the plant's internal dq.

## §D-CRIT — gate dimension
- `TimeSlicer` output `gate` must be a **6-element column** `[Sa+;Sa−;Sb+;Sb−;Sc+;Sc−]`. A `[1 6]` row gives inferred size `[1 6]` ≠ the Universal_Bridge gate port `[6]` → `Stateflow:cdr:DataBackPropagationSizeSuffix` at compile. (A Constant source auto-orients; a MATLAB-Function output is strict.)

## §G-CRIT — two-rate DISCRETE sample times (the dual-vector-specific rule)
Set via `cfg = get_param(blk,'MATLABFunctionConfiguration'); cfg.UpdateMethod='Discrete'; cfg.SampleTime=<...>;`
**`UpdateMethod` must be `'Discrete'` first — `SampleTime` is silently ignored otherwise (R2024b emits a warning, not an error).**

| Block | UpdateMethod | SampleTime | Why |
|---|---|---|---|
| `DualVecMPC` | Discrete | `s_Tsc` | compute the pair + `t_opt1` once per control period, hold; samples id/iq/theta_e at `Tsc` |
| `ThetaSrc` | Discrete | `s_Tsc` | correct angle integration (see §A) |
| `TimeSlicer` | Discrete | `s_Ts` | must run at the **fast** rate to slice within the period and emit a discrete gate |
| `SimClock` | Digital Clock | `s_Ts` | discrete in-period timer `t` for `mod(t,Tsc)` |

A continuous/inherited gate cannot propagate into the discrete SimPowerSystems network → `Stateflow:SignalTypePropagationError`. This differs from single-vector `motor-fcs-mpc` (which uses INHERITED + dual ZOH and holds one vector the whole period) — dual-vector genuinely needs **two rates** because the slicer subdivides `Tsc`. **This §G rule overrides the base `api_notes §4/§11` INHERITED+ZOH chart default** (correct for long single-rate charts, but it blocks the fast-rate slicer here).

**Setting a MATLAB Function block's code body (R2024b).** The `DualVecMPC`/`TimeSlicer`/`ThetaSrc` bodies (with their `sprintf` K-CRIT literals) are injected via the Stateflow EMChart object, not a block param:
```matlab
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/DualVecMPC'], 'Position', pos);
chart = sfroot().find('-isa','Stateflow.EMChart','Path',[mdl '/DualVecMPC']);
chart.Script = codeStr;                                   % the function text
cfg = get_param([mdl '/DualVecMPC'],'MATLABFunctionConfiguration');
cfg.UpdateMethod = 'Discrete'; cfg.SampleTime = 's_Tsc';  % then the §G rate
```

## §J-CRIT — one-click reproducibility
- Inject EVERY parameter via `set_param(mdl,'InitFcn', strjoin(...))` so a fresh double-click session self-injects. Use `num2str(x,'%.15g')` for full precision.
- Fixed-step **discrete** solver (`FixedStepDiscrete`), `FixedStep = Ts`; `powergui` Discrete @ `Ts`. Discontinuous gate switching forbids variable-step-with-zero-crossings drift.
- Build hygiene: `cd` into the throwaway run dir before `new_system`/`add_block` so `slprj/` + `.slxc` cache do not pollute `shared/`/signed dirs.

## §K-CRIT — chart param consistency (no drift)
- `DualVecMPC` (and `ThetaSrc`) hard-code `Rs/Ld/Lq/psif` (and `Pn/Tsc`) as `sprintf` literals at build time. The literals MUST equal the `InitFcn` values **digit-for-digit** — assert it in the build (`regexp` the chart script vs the formatted `InitFcn` value) and fail the build on mismatch. A chart that predicts with different params than the plant runs silently wrong.
- SPMSM ⇒ `Ld=Lq=Ls` (one value, written into both literals).

## §DC-CRIT — DC bus polarity (power path)
- Wire `DC +(RConn) → UB RConn(1)(+)`, `DC −(LConn) → UB RConn(2)(−)`. Reversed polarity clamps the link via the bridge freewheel diodes → `vds=vqs=0`, zero current, rotor stalls (NOT an obvious error — the model runs to completion producing nothing). Verify on first run that `vds/vqs` are non-zero with an active gate. See [anti_patterns.md](anti_patterns.md) #1.
