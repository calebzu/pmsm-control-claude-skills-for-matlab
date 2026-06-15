# Build SOP — MFPCC-ESO model construction

One-click-reproducible build of the full drive: speed PI → MFPCC-ESO deadbeat → SVPWM → Universal
Bridge → SPMSM. Library blocks come **by reference** from `shared/building_blocks/pmsm_blocks.slx`
(read-only — never fork/edit the library); method-specific glue is authored fresh.

## 1. Block inventory

Load the library first, copy by block-diagram name (full file paths are invalid block specifiers):

```matlab
load_system('<ws>/shared/building_blocks/pmsm_blocks.slx');
add_block('pmsm_blocks/<Name>', [mdl '/<Name>'], 'Position', pos);
```

| From `pmsm_blocks.slx` (by reference) | Role |
|---|---|
| `powergui` | Discrete solver env, `SampleTime='Ts'` |
| `DC_Voltage_Source` | DC bus, Amplitude=`Vdc_val` |
| `Universal_Bridge` | IGBT inverter, gate = Simulink inport 1 |
| `PMSM` | SPMSM plant (`Ld=Lq=Ls`), MeasurementBus on |
| `BusSelector_PMSM` | bus unpack `ias,ibs,ics,w,theta,Te` |
| `Clark` | abc → αβ (amplitude-invariant) |
| `SVPWM` | (Vα,Vβ) → 6-bit gate @ fixed fsw |

| Stock Simulink (authored fresh) | Role |
|---|---|
| 2 × MATLAB Function | `MFPCC_ESO` + `SpeedPI` charts (chart_contract.md) |
| 6 × Zero-Order Hold @ `Tsc` | input glue |
| 2 × Unit Delay @ `Tsc`, IC=0 | output glue (NOT ZOH — chart_contract.md §3) |
| 2 × From Workspace | `wref`, `TL` (inline expressions, §3) |
| 2 × Gain (`Pn`) | mech → electrical θe / ωe |
| Constant 0 | `id_ref` |
| Mux ×2, To Workspace ×10, Scope | logging (§6) |

No `Plark`/`Anti_Park` in this build — the chart does its own frame math internally.

## 2. Power-stage wiring (exact)

```
DC/RConn1 → UB/RConn1 (+)          DC/LConn1 → UB/RConn2 (−)      % DC link polarity
UB/LConn1..3 → PMSM/LConn1..3                                      % AC phases
SVPWM/1 → UB/1                                                     % 6-bit gate (Simulink inport)
TL (From Workspace) → PMSM/1                                       % load torque (Simulink inport)
```

Gotcha: on UB, `RConn*` = DC side, `LConn*` = AC side — the R/L prefix is NOT polarity. Crossed DC
polarity freewheels the link to zero volts: zero current, stalled rotor, **no error message**.

## 3. Signal wiring (exact)

```
PMSM/1 → BusSel/1     with OutputSignals = 'ias,ibs,ics,w,theta,Te'  (fixes outport order 1..6)
BusSel/1,2,3 → Clark/1,2,3
Clark/1 → ZOHi_ial → MFPCC_ESO/1          Clark/2 → ZOHi_ibe → MFPCC_ESO/2
BusSel/5 (theta, MECH) → Gain_Pn_th → ZOHi_the → MFPCC_ESO/3
BusSel/4 (w, MECH)     → Gain_Pn_w  → ZOHi_ome → MFPCC_ESO/4
wref (From Workspace)  → SpeedPI/1        BusSel/4 → SpeedPI/2      (both MECH rad/s, no Pn gain)
SpeedPI/1 → ZOHi_iqr → MFPCC_ESO/5        Constant(0) → ZOHi_idr → MFPCC_ESO/6
MFPCC_ESO/1 (ua) → UnitDelay_ua → SVPWM/1
MFPCC_ESO/2 (ub) → UnitDelay_ub → SVPWM/2
```

`wref`/`TL` are From-Workspace blocks whose **VariableName holds an inline expression** (evaluated
against InitFcn scalars — keeps one-click without pre-building timeseries):

```
wref: [0 0; ramp_t wref_radps; StopTime wref_radps]      % ramp then hold
TL:   [0 0; TL_t 0; TL_t TL_val; StopTime TL_val]        % step at TL_t
```

## 4. SVPWM contract + sector-7 startup fix

- Internal constants read workspace **`Tpwm`** and **`Vdc_val`** — InitFcn MUST define both names.
- Carrier (Repeating Sequence) is hardcoded `t=[0 Tpwm/2 Tpwm]`, `y=[0 Tpwm/2 0]` for
  `Tpwm = 1e-4`. If you change `Tpwm`, update the carrier on the local instance too.
- **Sector-7 fix (REQUIRED)**: at t=0, `Vα=Vβ=0` resolves to invalid sector 7 → hard error. Apply on
  the local instance after wiring (edits only this copy, library untouched):

```matlab
set_param([mdl '/SVPWM'],'LinkStatus','inactive');
ms = find_system([mdl '/SVPWM'],'LookUnderMasks','all','FollowLinks','on', ...
                 'BlockType','MultiPortSwitch');
for k = 1:numel(ms), set_param(ms{k},'DiagnosticForDefault','None'); end
```

## 5. Solver + InitFcn + re-commit guard

- Model: `SolverType='Fixed-step'`, `Solver='FixedStepDiscrete'`, `FixedStep='Ts'`. powergui
  `Discrete` @ `Ts`. All-discrete is deliberate (faster, and exact for this digital law) — this is
  the documented exception to the "variable-step with SPS" default (`api_notes.md` §6).
- **InitFcn template** (every parameter; model must run from a double-click in a fresh session):

```matlab
init = sprintf(['Rs=%.6g; Ld=%.6g; Lq=%.6g; flux=%.6g; Pn=%d; J=%.6g; F=%.6g; ' ...
    'Vdc_val=%.6g; Ts=%.6g; Tsc=%.6g; Tpwm=%.6g; ' ...
    'alpha=%.6g; z12=%.6g; omega0=%.6g; beta01=%.6g; beta02=%.6g; ' ...
    'iq_max=%.6g; Kp_w=%.6g; Ki_w=%.6g; ' ...
    'wref_radps=%.6g; ramp_t=%.6g; TL_val=%.6g; TL_t=%.6g; StopTime=%.6g;'], ...
    p.Rs, p.Ls, p.Ls, p.psi_f, p.Pn, p.J, p.B, ...      % PMSM mask: F = viscous friction B
    p.Vdc, p.Ts, p.Tsc, p.Tsc, ...                       % Tpwm = Tsc (control = switching)
    p.alpha, p.z12, omega0, beta01, beta02, ...
    p.iq_max, p.Kp_w, p.Ki_w, ...
    p.wref_radps, p.ramp_t, p.TL_val, p.TL_t, p.StopTime);
set_param(mdl, 'InitFcn', init);
```

  with the asserted closed-form gains computed in the build script:

```matlab
omega0 = (1 - p.z12)/p.Tsc;
beta01 = 2*omega0*p.Tsc;  beta02 = omega0^2*p.Tsc;
assert(abs(beta01 - 2*(1-p.z12)) < 1e-12 && abs(beta02 - (1-p.z12)^2/p.Tsc) < 1e-9);
```

- **Chart parameters are build-time literals, NOT workspace reads** (chart_contract.md §1/§2): the
  two chart bodies embed `alpha/beta01/beta02/Tsc` and `Kp_w/Ki_w/iq_max/Tsc` via the same
  `sprintf('%.6g',...)` values used in the InitFcn — assert the literals equal the InitFcn string
  digit-for-digit. The InitFcn still carries all of them (one-click traceability + SVPWM/plant use
  `Tpwm/Vdc_val/...` from workspace, which DOES work for ordinary blocks — the restriction is
  charts only).

- **Re-commit guard**: after all wiring, re-apply both charts' `FunctionScript` + sample-time config
  once more (chart_contract.md §5).
- Hygiene: `cd` into the run workspace BEFORE `new_system` so `slprj/` + `.slxc` don't pollute the
  read-only `shared/` library.

## 6. Logging + Scope

10 To-Workspace channels (timeseries), each at its block's natural rate:
`log_w` (BusSel/4), `log_Te` (BusSel/6), `log_abc` (Mux of BusSel/1,2,3), `log_iq`/`log_id`
(chart 3/4), `log_Fa`/`log_Fb` (chart 5/6), `log_iqref` (SpeedPI/1), `log_uab` (Mux of delayed
ua/ub), `log_TL`. Plus one Scope on speed for the human visual check.

⚠ **Ts/Tsc mixed-rate trap**: channels live on different time grids. In analysis, index every signal
by **its own** `.time` — never reuse another channel's time axis (classic false-"broken" diagnosis).

## 7. Validation-case defaults (paper Table I plant + supplied mechanics)

| Param | Value | Status | Param | Value | Status |
|---|---|---|---|---|---|
| `Rs` | 2.34 Ω | paper | `alpha` | 50 | paper |
| `Ls` | 19.36 mH | paper | `z12` | 0.15 | paper |
| `psi_f` | 0.402 Wb | paper | `iq_max` | 6 A | supplied |
| `Pn` | 4 | paper | `Kp_w, Ki_w` | 0.6, 8 | supplied |
| `Vdc` | 540 V | paper | `J` | 3e-3 kg·m² | supplied |
| `Ts` | 1e-6 s | — | `B` (mask `F`) | 1e-3 N·m·s | supplied |
| `Tsc=Tpwm` | 1e-4 s (10 kHz) | paper | scenario | 1000 rpm ramp 0.1 s; TL 5 N·m @ 0.25 s; stop 0.4 s | validation case |

`wref_radps = 1000·2π/60 = 104.72` (mechanical). Vdc headroom note: at the 1500 rpm rated point
`Udc/√3 ≈ 311 V ≈` rated phase peak — zero overmodulation margin by design (paper plant); the
validation case runs at 1000 rpm where margin ≈ 1.85×.
