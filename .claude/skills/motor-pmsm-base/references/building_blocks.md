# Building Blocks Library SOP

## Atomic Block Library

`shared/building_blocks/pmsm_blocks.slx` provides 14 verified atomic blocks (parameter placeholders, shareable across methods). Authoritative list with full mask configs: `shared/building_blocks/pmsm_blocks_manifest.md`.

- **Transforms** — `Clark` (abc → αβ, amplitude-invariant), `Plark` (αβ → dq, original Park; θ_e as explicit inport), `Anti_Park` (dq → αβ inverse Park; reads θ_e internally via `From "The"`). Conventions match [plant_modeling.md](plant_modeling.md)
- **PMSM block** (R2024b SPS wrapper with explicit mask settings — `RefAngle` pre-set to original Park to match `shared/formulas/pmsm_formulas.md §1`; the `sps_lib` bare default is modified Park, 90° offset — see RefAngle CRIT below and F-CRIT 6)
- **SVPWM** (project-built SubSystem inside `pmsm_blocks.slx`, block name `SVPWM` — 5 sub-subsystems: Sector_Caculate / T1T1_Caculate / Tcm_Caculate / XYZ_Caculate / PWM)
- **Power stage** — `powergui` (Discrete), `DC_Voltage_Source`, `Universal_Bridge` (IGBT/Diodes inverter)
- **Measurement & routing** — `BusSelector_PMSM` (OutputSignals `'ias,ibs,ics,w,Te'`), `VoltageMeasurement`, `CurrentMeasurement`
- **Rate utilities** — `ZOH_Tsc`, `UnitDelay_Tsc` (SampleTime = Tsc), `RateLimiter` (ω_ref ramp)

## Reuse Discipline

Subagent build scripts must use `add_block` to copy from the shared library — do **not** rebuild blocks from memory:

```matlab
add_block('<project>/shared/building_blocks/pmsm_blocks.slx/PMSM', ...
          [mdl '/PMSM'], 'Position', [...]);
```

This guarantees:
- Frame conventions match
- Mask defaults are battle-tested
- Future library updates propagate to all skills

## G-CRIT — Goto / From `TagVisibility='global'`

Anti_Park internally reads θ_e from the main model via a `From` block tagged `"The"`. The corresponding `Goto` in the main model **must** have `TagVisibility='global'`:

```matlab
% In build script: explicitly set global visibility
add_block('simulink/Signal Routing/Goto', [mdl '/Goto_The'], ...
          'GotoTag', 'The', ...
          'TagVisibility', 'global');     % MUST be 'global'
```

**Why this matters**: the default `'local'` is a silent failure mode. The `From` inside Anti_Park's subsystem cannot see a `local` Goto in the parent — it returns 0. Anti_Park then computes `inv-Park(θ_e=0)` which is just identity in the lab frame. The whole FOC closed-loop becomes silently open-loop. Symptoms: motor stalled, abc currents DC-locked at single angle, iq permanently at ±iq_max.

A self-test in Phase 9 should assert this:

```matlab
goto_blocks = find_system(mdl, 'BlockType', 'Goto', 'GotoTag', 'The');
if ~isempty(goto_blocks)
  assert(strcmp(get_param(goto_blocks{1}, 'TagVisibility'), 'global'), ...
    'G-CRIT: Goto_The TagVisibility must be global');
end
```

## RefAngle CRIT — PMSM `RefAngle` Frame Alignment

The R2024b `sps_lib/.../Permanent Magnet Synchronous Machine` block defaults `RefAngle = '90 degrees behind phase A axis (modified Park)'`. The project's chart Park convention (`d = α·cos(θe) + β·sin(θe)`, per `shared/formulas/pmsm_formulas.md §1`) is the original Park. The two must match — otherwise the plant dq frame is rotated 90° relative to the chart dq, a silent failure mode (see [broken_foc_diagnostics.md](broken_foc_diagnostics.md) F-CRIT 6).

**Required**: copy from `shared/building_blocks/pmsm_blocks.slx/PMSM` (library instance is pre-set), or set explicitly when adding bare from `sps_lib`:

```matlab
set_param([mdl '/PMSM'], 'RefAngle', 'Aligned with phase A axis (original Park)');
```

A self-test in Phase 9 should assert this:

```matlab
pmsm_blocks = find_system(mdl, 'MaskType', 'Permanent Magnet Synchronous Machine');
if ~isempty(pmsm_blocks)
  assert(strcmp(get_param(pmsm_blocks{1}, 'RefAngle'), ...
                'Aligned with phase A axis (original Park)'), ...
    'RefAngle CRIT: PMSM RefAngle must be original Park');
end
```

DTC αβ-frame methods are exempt (no Park transform).

## SVPWM Block

- Source: **project-built SubSystem inside `pmsm_blocks.slx`** (block name `SVPWM`) — 5 sub-subsystems: Sector_Caculate / T1T1_Caculate / Tcm_Caculate / XYZ_Caculate / PWM + Repeating Sequence triangle carrier + 2 Constants. Copy via `add_block` from the library; do not rebuild from memory.
- Output: 6-bit pair-adjacent pulse `[Sa_up, Sa_dn, Sb_up, Sb_dn, Sc_up, Sc_dn]` direct-fit to `Universal_Bridge` inport 1. Internal Constants reference `Tpwm` + `Vdc_val` workspace vars (set in InitFcn).
- **Sector=7 startup fix**: at t=0 transient `Vα=Vβ=0` makes `Sector_Caculate` output sector=7 (`sign(X)=sign(Y)=sign(Z)=+1` → `4+2+1=7`, out of valid 1..6), and the internal MultiPort Switch blocks (`DiagnosticForDefault='Error'`) throw. Fix on the **local instance** (break library link, set MultiPortSwitch default to None):

```matlab
set_param([mdl '/SVPWM_blk'], 'LinkStatus', 'inactive');
ms_blks = find_system([mdl '/SVPWM_blk'], 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'BlockType', 'MultiPortSwitch');
for k = 1:numel(ms_blks); set_param(ms_blks{k}, 'DiagnosticForDefault', 'None'); end
```

> The older `StartSector` parameter applies **only** to the MathWorks official *Discrete SV PWM Generator* block, which this project does **not** use — the project SVPWM is a hand-built SubSystem with no `StartSector` parameter.

## Cross-Decoupling Feedforward (FF) — Dimensional Correctness

dq decoupling FF (amplitude-invariant convention):

- `Vd_ff = -ω_e · Lq · iq`     (note **ω_e** in rad/s, NOT θ_e in rad)
- `Vq_ff =  ω_e · (Ld · id + ψ_f)`

**Build script must use a dedicated block** to compute `ω_e = Pn · ω_m`:

```matlab
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Pn_omega'], ...
          'Gain', 'Pn');
% Connect ω_m → Gain_Pn_omega → FF Mux input
```

Do NOT route θ_e directly into the FF Mux. The dimensions will appear superficially valid (same units as ω_e if you forget time), but the physical behavior will be wrong.
