# CRIT Conditions — three-vector FCS-MPC (must-pass build invariants)

Violating any of these produces a model that either fails to compile or silently runs broken. Each was hit and resolved in the validated Phase-5 rebuild; see [anti_patterns.md](anti_patterns.md) for the failure signatures.

## §A-CRIT — single `theta_e`, integrated at `Tsc`, global tags
- One `theta_e = ∫ Tsc·(Pn·w)` (persistent, wrapped `atan2(sin,cos)`), feeding **both** `Plark` and the controller vector Park. Do NOT use the PMSM measurement-bus `theta` (R2024b bus-consistency caveat).
- `ThetaSrc` must be **DISCRETE @ Tsc**. At the fast rate it adds `Tsc·we` every `Ts` and over-integrates the angle by `Tsc/Ts` → wrong field, ripple/torque collapse.
- Any Goto/From carrying `theta_e` (or any signal): `TagVisibility='global'`. Default local tag ⇒ From reads nothing ⇒ open-loop lab-frame (classic broken-FOC, base `broken_foc_diagnostics.md`).
- **Override of the base grid:** `motor-pmsm-base/pre_build_grid.md` suggests taking dq feedback from the PMSM internal measurement (`BusSel iqs/ids`). For three-vector that is WRONG — the controller's vector Park and the current feedback must share the SAME externally-integrated `theta_e`, so feed currents through `Clark → Plark(theta_e)`.

## §D-CRIT — gate dimension
- `Slicer` output `gate` must be a **6-element column** `[Sa+;Sa−;Sb+;Sb−;Sc+;Sc−]`. A `[1 6]` row gives inferred size `[1 6]` ≠ the Universal_Bridge gate port `[6]` → `Stateflow:cdr:DataBackPropagationSizeSuffix` at compile. (A Constant source auto-orients; a MATLAB-Function output is strict.)

## §G-CRIT — two-rate DISCRETE sample times + R2024b code injection
Set via `cfg = get_param(blk,'MATLABFunctionConfiguration'); cfg.UpdateMethod='Discrete'; cfg.SampleTime=<...>;`
**`UpdateMethod` must be `'Discrete'` first — `SampleTime` is silently ignored otherwise (R2024b emits a warning, not an error).**

| Block | UpdateMethod | SampleTime | Why |
|---|---|---|---|
| `TVMPCC` | Discrete | `Tsc` | compute the 6-candidate cost + winning `(a,b,ti,tj)` once per control period, hold |
| `ThetaSrc` | Discrete | `Tsc` | correct angle integration (see §A) |
| `Slicer` | Discrete | `Ts` | must run at the **fast** rate to slice three segments within the period |
| `SimClock` | Digital Clock | `Ts` | discrete in-period timer `t` for `mod(t,Tsc)` |

A continuous/inherited gate cannot propagate into the discrete SimPowerSystems network → `Stateflow:SignalTypePropagationError`. The three-vector slicer subdivides `Tsc` into **three** segments, so it genuinely needs the fast rate (same reason as dual-vector, one more segment).

**Overrides base `api_notes §11`.** The base note prescribes INHERITED + bilateral ZOH for *long* charts (it was reacting to the older `sfroot().find(...).Script` + `ch.SampleTime` route, which deadlocks type-propagation). For this skill set the code **and** the rate through the SAME `MATLABFunctionConfiguration` config object (`FunctionScript` + `UpdateMethod='Discrete'`, below) — this runs the long `TVMPCC` chart DISCRETE @ `Tsc` without the §11 deadlock (verified in rebuild). Follow §G here, not base §11, for the controller/slicer charts. Keep `Tsc/Ts` an **integer** so the slicer's `mod(t,Tsc)` lands on segment boundaries.

**Setting a MATLAB Function block's code body (R2024b) — use the config object, NOT the Stateflow class.** The block's chart class in R2024b is `Stateflow.EMChart` (the older `Stateflow.EMLChart` lookup returns 0 charts). The release-stable route writes the code through the **same** configuration object used for the §G rate:
```matlab
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/TVMPCC'], 'Position', pos);
cfg = get_param([mdl '/TVMPCC'],'MATLABFunctionConfiguration');
cfg.FunctionScript = codeStr;                             % the function text (R2024b)
cfg.UpdateMethod   = 'Discrete'; cfg.SampleTime = 'Tsc';  % then the §G rate
```
(If a release lacks `FunctionScript`, fall back to the Stateflow `EMChart` object: `sfroot().find('-isa','Stateflow.EMChart','Path',blk).Script = codeStr` — note `EMChart`, not `EMLChart`.)

## §J-CRIT — one-click reproducibility
- Inject EVERY parameter via `set_param(mdl,'InitFcn', strjoin(...))` so a fresh double-click session self-injects. Use `%.10g`/`%.15g` for full precision.
- Fixed-step **discrete** solver (`FixedStepDiscrete`), `FixedStep = Ts`; `powergui` Discrete @ `Ts`. Discontinuous gate switching forbids variable-step-with-zero-crossings drift.
- Build hygiene: `cd` into the throwaway run dir before `new_system`/`add_block` so `slprj/` + `.slxc` cache do not pollute `shared/`/signed dirs.
- **`add_block` signature**: the 2nd argument is the destination **path** string `[mdl '/Name']`, NOT a position array. Set position via `'Position',[...]`. (Passing the position array as the destination → "invalid destination block".)

## §K-CRIT — chart param consistency (no drift)
- `TVMPCC` (and `ThetaSrc`) hard-code `Rs/Ld/Lq/psif/Vdc` (and `Pn/Tsc`) as `sprintf` literals at build time. The literals MUST equal the `InitFcn` values **digit-for-digit** — write them with the SAME formatter (`%.10g`) and assert equality in the build. A chart that predicts with different params than the plant runs silently wrong.
- SPMSM ⇒ `Ld=Lq=Ls` (one value, written into both literals).

## §DC-CRIT — DC bus polarity (power path)
- Wire `DC +(RConn) → UB RConn(1)(+)`, `DC −(LConn) → UB RConn(2)(−)`. Reversed polarity clamps the link via the bridge freewheel diodes → `vds=vqs=0`, zero current, rotor stalls (NOT an obvious error — the model runs to completion producing nothing). Verify on first run that `vds/vqs` are non-zero with an active gate. See [anti_patterns.md](anti_patterns.md) #6.

## §TV-CRIT — three vectors must stay live
- The T5 three-case feasibility clamp must **drop** an out-of-range vector and renormalize, not silently pin everything to one index. In steady state confirm `t_i, t_j, t_z` are all meaningfully nonzero (the controller is genuinely using three segments). If one is pinned at 0/`Tsc` every period, the build has degenerated to dual/single-vector — a finding, not success. See [anti_patterns.md](anti_patterns.md) #5.
