---
name: motor-pmsm-base
description: PMSM Simulation Modeling Base — entry-point skill for building three-phase voltage-source-inverter-driven PMSM (SPMSM/IPMSM) control simulations in Simulink. Provides plant build standard, dq frame conventions, building blocks library SOP, sanity-check templates, broken-FOC defense checklist, and visual review standards. Method-specific skills (motor-fcs-mpc / motor-dtc-pmsm / motor-smc-pmsm) layer their control law on this base. Use when starting a new PMSM control method study, designing a build script skeleton, or debugging broken-FOC symptoms (motor stalled, abc DC-locked). Skip for non-PMSM motors (induction / BLDC / SRM) or pure theory questions.
metadata:
  version: "1.0"
---

# motor-pmsm-base — PMSM Simulation Modeling Base

Reusable plant + frame conventions + building blocks + sanity-check templates + broken-FOC defense + visual review standards for three-phase voltage-source-inverter-driven PMSM (SPMSM / IPMSM via parameterization). Method-specific control laws (FOC / FCS-MPC / DTC / SMC / sensorless / observer / MTPA / weak-field) layer on top of this base.

## Must-Follow Rules

- **Use `shared/*` assets, do not duplicate.** `shared/formulas/pmsm_formulas.md` has signed-off plant equations and control law derivations. `shared/building_blocks/` provides verified atomic blocks. Reference them by path; do not re-derive or copy content.
- **Run pre-build sanity grid before any build.** See [references/pre_build_grid.md](references/pre_build_grid.md). Fail-fast when Vdc/BEMF tight, Goto TagVisibility default-local, FF dimensional incorrect, or sector=7 startup unhandled.
- **Visual 4-check before trusting numerical metrics**: motor rotates / iq tracks reference / abc AC sinusoidal / Te energy balance. If any check fails, fix the implementation before computing 5-metric scores.
- **Layout gate after build (MANDATORY, not advisory — prevents silent skip)**: `block_overlaps==0` is a HARD self-test (T7; build fails otherwise) + a mandatory human screenshot sign-off (same human-in-loop model as the Visual 4-check). Line-line crossings and line-through-block hits are SOFT (reported, NEVER numeric-gated): non-planar signal graphs (K3,3/K5) cannot reach zero crossings on a 2-D plane (Kuratowski), so reach zero only by restructuring (Subsystem encapsulation / Goto/From), never by hard-gating a count. See [references/measurement_logger.md](references/measurement_logger.md).
- **One-click reproducibility**: model must Run from `.slx` double-click without prior script execution. Inject all parameters via `set_param(mdl, 'InitFcn', ...)`.
- **Don't guess conventions.** dq is amplitude-invariant (factor 2/3); Anti_Park's internal `From "The"` requires `Goto_The TagVisibility='global'`. See [references/plant_modeling.md](references/plant_modeling.md) and [references/building_blocks.md](references/building_blocks.md).
- **PMSM block `RefAngle` must be `'Aligned with phase A axis (original Park)'`.** R2024b `sps_lib` default is `'90 degrees behind phase A axis (modified Park)'`, which rotates the plant dq frame 90° relative to the chart Park convention — a silent failure mode (motor drifts to wrong operating point, abc non-clean sinusoidal, `id` large drift, `iq` cannot track `iq_ref`). Either copy PMSM from `shared/building_blocks/pmsm_blocks.slx` (pre-set), or `set_param([mdl '/PMSM'], 'RefAngle', 'Aligned with phase A axis (original Park)')`. DTC αβ-frame methods exempt. See [references/broken_foc_diagnostics.md](references/broken_foc_diagnostics.md) F-CRIT 6.

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate params + sanity grid | [pre_build_grid.md](references/pre_build_grid.md) |
| 1 | Theory anchor (sign or AI-self-audit formulas) | [theory_anchor.md](references/theory_anchor.md) |
| 2 | PMSM plant + InitFcn injection | [plant_modeling.md](references/plant_modeling.md) |
| 3 | Building blocks (Park / Clarke / Anti_Park / SVPWM) | [building_blocks.md](references/building_blocks.md) |
| 4 | Control loop | method-specific skill |
| 5 | Modulation (FOC-based methods only; DTC skips) | [building_blocks.md](references/building_blocks.md) |
| 6 | Measurement + Logger | [measurement_logger.md](references/measurement_logger.md) |
| 7 | Sanity check + Visual 4-check | [sanity_visual.md](references/sanity_visual.md) |
| 8 | Solver + layout gate (T7 zero-overlap + screenshot sign-off) + save | [measurement_logger.md](references/measurement_logger.md) |
| 9 | Idempotent self-tests | [measurement_logger.md](references/measurement_logger.md) |

If broken-FOC symptoms appear (motor stuck, abc DC-locked, iq permanently bang-bang), jump to [broken_foc_diagnostics.md](references/broken_foc_diagnostics.md).

## When to Use

| ✅ Use this skill | ❌ Skip |
|---|---|
| Starting a new PMSM control method study (sensorless, DTC-SVM, MP-DTC, deadbeat, MTPA, weak-field) | Non-PMSM motors (induction / BLDC trapezoidal / SRM / DC) |
| Designing your build script (conventionally named `build_template.m`) for a PMSM method | Pure theory questions (dq frame, Park transform math) |
| Broken-FOC debugging (motor stalled, abc DC-locked, iq saturated) | Method already covered by `motor-fcs-mpc` / `motor-dtc-pmsm` / `motor-smc-pmsm` (use the specific skill) |
| Need Phase 1.5 theory-anchor or Phase 4.5 sanity-check templates | MATLAB performance tuning (use `matlab-performance-optimizer`) or unit tests (use `matlab-test-creator`) |

## Known Limitations

- PMSM only (SPMSM / IPMSM). Induction motor / BLDC trapezoidal / SRM / DC are different plants.
- dq amplitude-invariant convention. Power-invariant requires extension.
- Three-phase 2-level VSI. Multilevel inverter / matrix converter out of scope.
- Nominal parameters. No robust / adaptive / online parameter ID.
- Ideal sensors. No encoder quantization or current sampling delay.
- No fault tolerance, no multi-motor coordination.

## Method-specific Skills (Layered on This Base)

| Skill | Method |
|---|---|
| [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) | Single-vector finite-control-set MPC current control |
| [motor-dtc-pmsm](../motor-dtc-pmsm/SKILL.md) | Direct Torque Control with hysteresis switching table (Takahashi / Sutikno 6-state) |
| [motor-smc-pmsm](../motor-smc-pmsm/SKILL.md) | Sliding Mode Control speed-loop (PD-type sliding + super-twisting) |

## External References

- Krause, Wasynczuk, Sudhoff (2013). *Analysis of Electric Machinery and Drive Systems* — PMSM dq modeling reference.
- Bose (2002). *Modern Power Electronics and AC Drives* — VSI + SVPWM reference.
- Slotine, Li (1991). *Applied Nonlinear Control* — Lyapunov-based design reference.
