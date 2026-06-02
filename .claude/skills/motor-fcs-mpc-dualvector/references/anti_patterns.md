# Anti-Patterns — dual-vector FCS-MPC

Concrete failures hit (and fixed) building the validated reference, with the exact symptom so you recognize them fast. R2024b.

## 1. Reversed DC bus polarity → silent stall  ⭐ most common
**Symptom:** model compiles and runs to completion, but speed=0, all currents=0, `vds=vqs=0`, the gate is active and the DC amplitude resolves correctly. Nothing errors.
**Cause:** `DC −(LConn)` wired to `UB +(RConn(1))` and `DC +` to `UB −`. Reversed DC across the IGBT bridge forward-biases the freewheel diodes, which clamp the DC link to ≈0 → the machine sees no voltage.
**Fix:** `add_line(dc.RConn(1), ub.RConn(1))` (+→+), `add_line(dc.LConn(1), ub.RConn(2))` (−→−). On first run, verify `vds/vqs` are non-zero with an active gate.
**Why it's sneaky:** reversed/clamped ≠ "reversed current"; the diode clamp gives *exactly zero*, which looks like "nothing connected" rather than "wired backwards." Probe terminal voltage to localize (a valid constant gate on a live bus MUST produce current).

## 2. Inherited/continuous chart sample time into the SPS bridge → won't compile
**Symptom:** `Stateflow:cdr:ErrorCompilingDataProperties` / `Stateflow:translate:SignalTypePropagationError` ("error propagating data type 'double' through TimeSlicer").
**Cause:** the `TimeSlicer` (or controller) had inherited/continuous sample time; a continuous gate cannot propagate into the discrete powergui/Universal_Bridge network.
**Fix:** §G-CRIT — `cfg.UpdateMethod='Discrete'` THEN `cfg.SampleTime`. Each custom MATLAB Function compiles fine in isolation; the conflict only appears at the SPS boundary, so isolate-compile to localize.

## 3. `SampleTime` set but ignored (UpdateMethod still Inherited)
**Symptom:** a warning "SampleTime has no effect when UpdateMethod is 'Continuous' or 'Inherited'", and the §2 propagation error persists.
**Cause:** `cfg.SampleTime` was set without first setting `cfg.UpdateMethod='Discrete'`.
**Fix:** set `UpdateMethod='Discrete'` first, then `SampleTime`. (Order matters; the warning is easy to miss in batch output.)

## 4. `gate` as a row vector `[1 6]`
**Symptom:** `Stateflow:cdr:DataBackPropagationSizeSuffix` — inferred `[1 6]` ≠ back-propagated `[6]`.
**Cause:** `gate = [sa, 1-sa, ...]` (row). The Universal_Bridge gate port is a 6-element vector and a MATLAB-Function output is strict (a Constant would auto-orient, hiding this).
**Fix:** emit a **column**: `gate = [sa; 1-sa; sb; 1-sb; sc; 1-sc];`.

## 5. `DataLogging` on the line handle
**Symptom:** `Simulink:Commands:ParamUnknown` for `DataLogging`.
**Cause:** in R2024b `DataLogging` is a property of the block **output port handle**, not the line. The line carries only the signal `Name`.
**Fix:** `set_param(porthandle,'DataLogging','on')`. For Bus Selector outputs, wrap the line-`Name` rename in try/catch (their labels are inherited and cannot be renamed).

## 6. Running `ThetaSrc` at the fast rate
**Symptom:** wrong electrical frequency in `i_abc`, ripple/torque don't match, motor may spin at the wrong speed or fight itself.
**Cause:** the persistent integrator `th += Tsc·we` executed every `Ts` integrates `theta_e` `Tsc/Ts`× too fast.
**Fix:** `ThetaSrc` DISCRETE @ Tsc (§A-CRIT). If you want a smooth fast-rate angle instead, integrate `th += Ts·we` at `Ts` — but then sample `theta_e` at `Tsc` into the controller.

## 7. Substituting the paper's volt-second voltage directly (no ÷Tsc)
**Symptom:** dq currents predicted off by a `Tsc` factor; selection degraded, ripple inflated.
**Cause:** the paper's eqs 12-13 are volt-seconds; substituting them into a prediction already carrying `Tsc/L` double-counts `Tsc`.
**Fix:** N4 duty-weighted **average** voltage (÷Tsc).

## 8. Using a torque-MAD script as "the ripple metric"
**Symptom:** ripple numbers that don't compare to Table 2.
**Cause:** the §E metric is **RMS-of-deviation of dq current** (= std); a `mean(abs(Te−mean(Te)))` torque-MAD is a different statistic on a different variable (the paper's eq 18-19 has a missing-square typo).
**Fix:** `Δi = std(i,1)` on `id/iq` over a settled window.

## 10. Copying a library block by file path
**Symptom:** `Simulink:Commands:InvBlockSpecifier` + `Simulink:LoadSave:NameTooLong` ("name > 63 characters") at the first `add_block`.
**Cause:** `add_block('<...>/pmsm_blocks.slx/powergui', ...)` — a full file path is not a valid block specifier, and the library was never loaded.
**Fix:** `load_system('<...>/pmsm_blocks.slx')` once, then `add_block('pmsm_blocks/<Name>', ...)` (diagram name, not path). Stock blocks not in the library (e.g. `Digital Clock`) come from `simulink/Sources/...`.

## 9. Treating it like single-vector (one held vector, INHERITED+ZOH)
**Symptom:** no ripple reduction vs single-vector, or `Top` pinned at 0/Tsc.
**Cause:** collapsing the two-rate structure — running the slicer at `Tsc` (so only one vector effectively applies) or never sequencing the second vector.
**Fix:** slicer DISCRETE @ Ts with `mod(t,Tsc)`; confirm in acceptance that `Top ∈ (0,Tsc)` and multiple vectors are used.
