# Anti-Patterns — three-vector FCS-MPC

Concrete failures hit (and fixed) building the validated reference, with the exact symptom so you recognize them fast. R2024b. #1–#4 are the four build-blockers actually hit in the Phase-5 rebuild (in order); #5+ carry over from the FCS-MPC family.

## 1. Inherited/continuous Slicer sample time into the SPS bridge → won't compile  ⭐ (G-CRIT)
**Symptom:** `Stateflow:cdr:ErrorCompilingDataProperties` / `Stateflow:translate:SignalTypePropagationError` ("error propagating data type 'double' through Slicer").
**Cause:** the `Slicer` was left **Inherited**; a continuous/inherited gate cannot propagate into the discrete powergui/Universal_Bridge network.
**Fix:** §G-CRIT — `cfg.UpdateMethod='Discrete'` THEN `cfg.SampleTime='Ts'`. The slicer must run at the fast rate to sequence its three segments. Each MATLAB Function compiles fine in isolation; the conflict only appears at the SPS boundary — isolate-compile to localize.

## 2. Wrong Stateflow class for code injection → "0 charts" (R2024b)  ⭐
**Symptom:** `Could not locate EMLChart for <blk> (found 0 charts total)` — `sfroot().find('-isa','Stateflow.EMLChart',...)` returns empty.
**Cause:** in R2024b a MATLAB Function block's chart is class **`Stateflow.EMChart`**, not `Stateflow.EMLChart`.
**Fix:** write the body through the configuration object: `get_param(blk,'MATLABFunctionConfiguration').FunctionScript = code` (release-stable, same object as the §G rate). If you must use the Stateflow API, match `'-isa','Stateflow.EMChart'`. See §G-CRIT.

## 3. `add_block` called with a position array as the destination  ⭐
**Symptom:** `Invalid destination block` at the first stock `add_block`.
**Cause:** `add_block('simulink/Sources/Step', [x y w h], 'Name','S', ...)` — the 2nd arg must be the destination **path** string, not a position.
**Fix:** `add_block(src, [mdl '/Name'], 'Position',[x y w h], ...)`. (A small `addStk(mdl,src,pos,varargin)` wrapper that derives the path from the `'Name'` pair makes every stock-block add uniform.)

## 4. `gate` as a row vector `[1 6]`  ⭐ (D-CRIT)
**Symptom:** `Stateflow:cdr:DataBackPropagationSizeSuffix` — inferred `[1 6]` ≠ back-propagated `[6]`.
**Cause:** `gate = [s1, 1-s1, ...]` (row). The Universal_Bridge gate port is a 6-element vector and a MATLAB-Function output is strict (a Constant would auto-orient, hiding this).
**Fix:** emit a **column**: `gate = [s1; 1-s1; s2; 1-s2; s3; 1-s3];`.

## 5. Treating it like dual/single-vector — three vectors collapse (TV-CRIT)
**Symptom:** ripple no better than dual-vector, or `t_z`/`t_i`/`t_j` pinned at 0/`Tsc` every period.
**Cause:** the T5 three-case clamp implemented as a hard "pin to one vector", or the slicer run at `Tsc` (only one segment effectively applies), or only the active-θ sector evaluated (single candidate).
**Fix:** evaluate all 6 expected vectors (T6b); slicer DISCRETE @ Ts with `mod(t,Tsc)`; T5 drops the out-of-range vector and renormalizes. Confirm `t_i,t_j,t_z` all nonzero in steady state.

## 6. Reversed DC bus polarity → silent stall (DC-CRIT)
**Symptom:** compiles + runs to completion, but speed=0, all currents=0, `vds=vqs=0`, gate active, DC amplitude correct. Nothing errors.
**Cause:** `DC −(LConn)` to `UB +(RConn1)` and `DC +` to `UB −` — reversed DC forward-biases the freewheel diodes, clamping the link to ≈0.
**Fix:** `+→+`, `−→−`. On first run verify `vds/vqs` non-zero with an active gate. (Diode clamp gives *exactly* zero — looks like "nothing connected", not "wired backwards".)

## 7. Running `ThetaSrc` at the fast rate (A-CRIT)
**Symptom:** wrong electrical frequency in `i_abc`, ripple/torque don't match, motor spins wrong.
**Cause:** the persistent integrator `th += Tsc·we` executed every `Ts` integrates `theta_e` `Tsc/Ts`× too fast.
**Fix:** `ThetaSrc` DISCRETE @ Tsc (§A-CRIT).

## 8. Copying a library block by file path
**Symptom:** `Simulink:Commands:InvBlockSpecifier` + `NameTooLong` (>63 chars) at the first `add_block`.
**Cause:** `add_block('<...>/pmsm_blocks.slx/powergui', ...)` — a full path is not a valid specifier, and the library was never loaded.
**Fix:** `load_system('<...>/pmsm_blocks.slx')` once, then `add_block('pmsm_blocks/<Name>', ...)` (diagram name). Stock blocks (e.g. `Digital Clock`) come from `simulink/Sources/...`.

## 9. Using a torque-MAD script as "the ripple metric"
**Symptom:** ripple numbers that don't compare to Table 4.
**Cause:** the §E metric is **RMS-of-deviation of dq current** (= `std(i,1)`); a `mean(abs(Te−mean(Te)))` torque-MAD is a different statistic on a different variable.
**Fix:** `Δi = std(i,1)` on `id/iq` over a settled window, sampled at the fast rate.

## 10. Batching all chart code injection after wiring multi-output blocks
**Symptom:** `Invalid Simulink object name: 'TVMPCC/3'` (or `ThetaSrc/2`, etc.) when wiring an output port that doesn't yet exist.
**Cause:** `add_block` on a MATLAB Function creates a default 1-in/1-out signature. If you wire port 2, 3, or 4 before injecting the correct function signature, those ports don't exist yet. Batching all `setChartCode` calls after all `add_line` calls triggers this on any multi-output chart.
**Fix:** inject a **signature stub** (minimal 1-line function with the correct number of inputs/outputs) immediately after each `add_block`, BEFORE any wiring. The full code body is then injected at any later point (it just overwrites the stub). Example:
```matlab
addStk('simulink/User-Defined Functions/MATLAB Function', 'TVMPCC', pos);
setChartCode([mdl '/TVMPCC'], sprintf('function [a,b,ti,tj]=tvmpcc(id_ref,iq_ref,id,iq,theta_e,we)\na=1;b=2;ti=0;tj=0;\nend'));
% ... now safe to wire TVMPCC/1..6 inputs and TVMPCC/1..4 outputs ...
```

## 11. Claiming the ripple "matches" Table 4
**Symptom:** over-claiming fidelity by reporting a Table-4 match.
**Cause:** Table 4 is **experimental**; an ideal sim gives *lower* ripple (validated: 0.085/0.158 vs 0.21/0.24). Tuning `Vdc`/`Ts` to hit it = fit-to-paper.
**Fix:** report honestly — method + trend reproduced, ideal-sim ripple lower than the experimental anchor (expected). See [acceptance_criteria.md](acceptance_criteria.md).
