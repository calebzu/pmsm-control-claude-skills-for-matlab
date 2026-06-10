# Measurement, Logger, Build Script Skeleton, Self-Tests

## Standard 4 Scopes (Mandatory)

Every PMSM build script (`build_template.m` by convention) must add these four Scopes. Method-specific Scopes are added on top, never as a replacement:

```matlab
add_block('simulink/Sinks/Scope', [mdl '/Scope_wm_RPM'], 'Position', [...]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_iq'],     'Position', [...]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_abc'],    'Position', [...]);  % 3 channels: ia, ib, ic
add_block('simulink/Sinks/Scope', [mdl '/Scope_Te'],     'Position', [...]);  % 2 channels: Te, TL+B·ω overlay
```

Method-specific examples:
- DTC: `Scope_psi_alphabeta` (αβ XY phase plot — circular vs hexagonal reveals 8-state pollution)
- SMC: `Scope_sliding` (`s` vs `t` — convergence vs divergence)
- FCS-MPC: `Scope_cost` (cost function vs `t`)

## Logger Channels (Minimum 14)

Use `To Workspace` blocks writing to `logsout`. Always include these:

| # | Channel | Source |
|---|---|---|
| 1 | `t` | Clock |
| 2 | `omega_ref` | step input |
| 3 | `omega_m` | PMSM bus / Integrator |
| 4 | `omega_e` | `Gain_Pn_omega` output |
| 5 | `theta_e` | Anti_Park / main model |
| 6 | `iq_ref` | outer controller output |
| 7 | `iq_meas` | PMSM bus `BusSel/7` |
| 8 | `id_meas` | PMSM bus `BusSel/8` |
| 9 | `Te` | PMSM bus |
| 10 | `TL` | TL step input |
| 11 | `Vdc` | constant |
| 12–14 | `ia`, `ib`, `ic` | abc measurement |

Method-specific channels added beyond #14: DTC adds `psi_alpha`, `psi_beta`, `sector`; SMC adds `s`, `sgn(s)`, `u_eq`, `u_sw`; FCS-MPC adds `V_k`, `cost`. See each method skill.

## Build Script Skeleton

```matlab
function build_pmsm_method(params)
  %% Phase 0 — params validation + sanity grid
  p = validate_pmsm_params(params);     % Required: Rs, Ld, Lq, psi_f, Pn, J, B, Vdc, Tsc, omega_ref, TL_max
  check_vdc_headroom(p);                % see pre_build_grid.md
  % Method-specific param checks (e.g., iq_max for FOC, psi_ref for DTC, K1/K2 for SMC)

  %% Phase 1 — model setup
  mdl = sprintf('pmsm_%s', method_name);
  if bdIsLoaded(mdl), close_system(mdl, 0); end
  new_system(mdl); open_system(mdl);

  %% Phase 2 — InitFcn injection (see plant_modeling.md)
  init_str = build_init_fcn(p);
  set_param(mdl, 'InitFcn', init_str);

  %% Phase 3 — PMSM plant
  add_pmsm_plant_block(mdl, p);

  %% Phase 4 — control loop (method-specific, layered on this base)
  % Outer: omega_ref → omega_meas → Te_ref / iq_ref / V_k (method-dependent)
  % Inner: depends on method
  %   - FOC / FCS-MPC: Vd/Vq → SVPWM → Universal_Bridge → 3-phase
  %   - DTC: αβ flux + hysteresis + switching table → V_k → gate
  %   - SMC: PD-type sliding + STA → iq_ref → cascaded PI inner → SVPWM → UB

  %% Phase 5 — modulation (FOC-based methods only; DTC skips)
  add_anti_park_block(mdl, p);          % G-CRIT: Goto_The TagVisibility='global' (see building_blocks.md)
  add_svpwm_block(mdl, p);              % see building_blocks.md
  add_universal_bridge_block(mdl, p);

  %% Phase 6 — measurement + feedback
  add_measurement_block(mdl, p);

  %% Phase 7 — Scopes + Logger
  add_scopes(mdl);
  add_logger(mdl, p);

  %% Phase 8 — solver + arrange + save
  set_param(mdl, 'StopTime', sprintf('%g', p.sim_time), ...
                 'Solver', 'ode23tb', ...
                 'ZeroCrossControl', 'DisableAll');
  try
    Simulink.BlockDiagram.arrangeSystem(mdl, 'FullLayout', 'true');
  catch e
    warning('arrangeSystem failed (%s) — fix layout manually; T7 still enforces block_overlaps == 0', e.message);
  end
  save_system(mdl);

  %% Phase 9 — self-tests (idempotent)
  run_self_tests(mdl, p);
end
```

## Position Bands (Non-Overlapping Wiring)

```
X bands:
   40–200   sources (omega_ref / TL / params)
  200–400   outer control (PI / SMC outer / DTC outer)
  400–600   inner / chart / decision logic
  600–800   modulation (Anti-Park / SVPWM / Universal Bridge) + plant
  800–940   measurement + Clarke + Park + Logger + Scopes

Y bands:
    0–300   speed loop (top horizontal lane)
  300–500   chart main flow
  500–700   measurement + Clarke + Park
  700–800   time + log
```

All `add_block` calls must pass `Position`.

**The layout gate is MANDATORY, not advisory** — an advisory band scheme alone gets silently skipped, leaving auto-built diagrams unreadable. After build:

- **HARD — `block_overlaps == 0`**: enforced by self-test T7 (build fails otherwise). Always achievable on a 2-D plane.
- **HARD — human visual sign-off**: export a screenshot and have the user confirm it is not messy. Same human-in-loop gate model as the Visual 4-check; the build cannot self-pass.
- **SOFT — line-line crossings / line-through-block (report only, NEVER a numeric gate)**: run a layout-tidy pass (arrange + de-overlap + planarity diagnosis) to minimize them, but a non-planar signal graph (K3,3/K5) cannot reach zero crossings (Kuratowski). To truly reach zero, restructure (encapsulate fan-in groups into a Subsystem, route long/shared signals via Goto/From), not by moving blocks.

`Simulink.BlockDiagram.arrangeSystem(mdl, 'FullLayout', 'true')` fixes overlaps + compactness but can *worsen* crossings — measure before/after and never accept a net crossing regression.

## Self-Tests (Phase 9, Idempotent)

```matlab
function run_self_tests(mdl, p)
  % T1: InitFcn populated
  init = get_param(mdl, 'InitFcn');
  assert(count(init, ';') >= 10, 'InitFcn underfilled');

  % T2: 4 Scopes present
  scopes = find_system(mdl, 'BlockType', 'Scope');
  assert(numel(scopes) >= 4, 'Need 4 Scopes minimum');

  % T3: Stateflow chart Script inline (not a 1-line wrapper)
  chart_blocks = find_system(mdl, 'MaskType', 'Stateflow');
  for i = 1:numel(chart_blocks)
    sf = sfroot().find('-isa', 'Stateflow.EMChart');
    if ~isempty(sf)
      assert(numel(splitlines(sf.Script)) >= 20, 'chart Script too short');
    end
  end

  % T4: Solver settings
  assert(strcmp(get_param(mdl, 'ZeroCrossControl'), 'DisableAll'), 'ZC not disabled');

  % T5: Goto_The TagVisibility='global' (G-CRIT)
  goto_blocks = find_system(mdl, 'BlockType', 'Goto', 'GotoTag', 'The');
  if ~isempty(goto_blocks)
    assert(strcmp(get_param(goto_blocks{1}, 'TagVisibility'), 'global'), ...
      'G-CRIT: Goto_The TagVisibility must be global');
  end

  % T6: reload + sim 1 cycle (no NaN/Inf)
  out = sim(mdl, 'StopTime', '0.1');
  assert(~any(isnan(out.logsout.find('omega_m').Values.Data)), 'NaN in omega_m');

  % T7: zero block-block overlaps (HARD layout gate — stops unreadable auto-built
  % diagrams; an advisory band scheme alone gets silently skipped). Block overlap
  % is always eliminable on a 2-D plane, so this CAN be a hard assert.
  lb = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
  XY = cell2mat(get_param(lb, 'Position'));
  for a = 1:size(XY, 1)
    for b = a+1:size(XY, 1)
      assert(XY(a,3) <= XY(b,1) || XY(b,3) <= XY(a,1) || ...
             XY(a,4) <= XY(b,2) || XY(b,4) <= XY(a,2), ...
        'T7 layout gate: blocks "%s" and "%s" overlap', lb{a}, lb{b});
    end
  end
  % NOTE: line-line crossings and line-through-block hits are SOFT (report only,
  % NEVER asserted). A non-planar signal graph (a K3,3/K5 — e.g. N estimators each
  % fanning to a selector AND a logger) cannot reach zero crossings on a 2-D plane
  % (Kuratowski); hard-gating crossings would reject valid models. Minimize them
  % with a layout-tidy pass + human screenshot sign-off, never a numeric gate.

  fprintf('All self-tests PASS (incl. T7 zero block overlaps).\n');
end
```
