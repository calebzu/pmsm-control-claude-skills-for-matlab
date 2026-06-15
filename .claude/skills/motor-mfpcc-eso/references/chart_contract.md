# Chart Contracts — MFPCC_ESO + SpeedPI (method-specific API)

Both controllers are MATLAB Function blocks. Set their code via the R2024b-robust route
(`get_param(blk,'MATLABFunctionConfiguration').FunctionScript = code`), never via the wrong
Stateflow class. Generic chart-configuration SOP: `shared/building_blocks/api_notes.md`.

## 1. `MFPCC_ESO` chart — 6 in / 6 out (FIXED contract)

| # | Input | Unit | Source |
|---|---|---|---|
| 1 | `ialpha` | A | Clark/1 → ZOH @ Tsc |
| 2 | `ibeta` | A | Clark/2 → ZOH @ Tsc |
| 3 | `theta_e` | rad (electrical) | BusSel `theta` × `Pn` → ZOH @ Tsc |
| 4 | `omega_e` | rad/s (electrical) | BusSel `w` × `Pn` → ZOH @ Tsc |
| 5 | `iq_ref` | A | SpeedPI/1 → ZOH @ Tsc |
| 6 | `id_ref` | A | Constant 0 → ZOH @ Tsc |

| # | Output | Unit | Sink |
|---|---|---|---|
| 1 | `ua` | V | Unit Delay @ Tsc → SVPWM/1 |
| 2 | `ub` | V | Unit Delay @ Tsc → SVPWM/2 |
| 3 | `iq_meas` | A | logger only (recovered internally from iα,iβ,θe) |
| 4 | `id_meas` | A | logger only |
| 5 | `Fa_o` | A/s | logger only (ESO `F̂` α-channel) |
| 6 | `Fb_o` | A/s | logger only (ESO `F̂` β-channel) |

**Config**: `UpdateMethod = 'INHERITED'`, `SampleTime = '-1'` — the effective rate comes from the
six input ZOHs @ `Tsc`. (Contrast: `SpeedPI` is explicitly DISCRETE @ `Tsc`.)

**Parameters**: `alpha, beta01, beta02, Tsc` ONLY — **embedded as numeric literals** in the chart
body via `sprintf` at build time, digit-for-digit equal to the InitFcn values (assert it in the
build). ⛔ Do NOT reference them as bare workspace names: a MATLAB Function chart does **not**
resolve base-workspace identifiers — undefined names make Stateflow fail with "cannot determine
output size/type" plus a MISLEADING "invalid dimension for input <name>" on an innocent input
(Simulink itself flags the message as possibly inaccurate). Same convention as the FCS-family
controller charts.
⛔ **Model-free boundary**: no `Rs`, no `Ls`, no `psi_f`, no `Pn` inside this chart. If a draft
needs a machine parameter to "work", the implementation is wrong — re-read the formulas.

### Per-step algorithm (equation numbers = pmsm_formulas.md §G)

```
persistent: z1a, z1b   % î_s   (current estimate, αβ)
            z2a, z2b   % F̂     (lumped disturbance estimate, αβ)
            ua_prev, ub_prev   % u(k-1) = what the plant is receiving THIS cycle
all initialized to 0.

1. Logging frame math (output only): rotate measurement to dq
     id_meas =  ialpha·cos(theta_e) + ibeta·sin(theta_e)
     iq_meas = -ialpha·sin(theta_e) + ibeta·cos(theta_e)
2. ESO update (eq 13), driven by the APPLIED voltage u(k-1) = (ua_prev, ub_prev):
     ea = z1a - ialpha;            eb = z1b - ibeta
     z1a_next = z1a + Tsc·z2a + Tsc·alpha·ua_prev - beta01·ea     (β-channel analogous)
     z2a_next = z2a - beta02·ea                                    (β-channel analogous)
3. Reference in αβ, advanced TWO steps (eq 21):
     i_ref(k)   = (id_ref + j·iq_ref)·e^{j·theta_e}
     i_ref(k+2) = i_ref(k)·e^{j·2·omega_e·Tsc}
4. Deadbeat with one-step delay compensation (eq 20), using the k+1 estimates:
     u = (1/alpha)·[ (i_ref(k+2) - z1_next)/Tsc - z2_next ]        (complex; split α/β)
5. Outputs: ua, ub = real/imag(u);  Fa_o, Fb_o = z2a_next, z2b_next.
   Update persistents: z1 ← z1_next, z2 ← z2_next, ua_prev ← ua, ub_prev ← ub.
```

Step-2-before-step-5 ordering matters: the ESO must consume `ua_prev` **before** it is overwritten,
so observer and plant see the same applied voltage. This is what keeps the observer consistent with
the Unit-Delay output path (§3).

## 2. `SpeedPI` chart — 2 in / 1 out

| I/O | Signal | Unit | Notes |
|---|---|---|---|
| in 1 | `omega_ref` | rad/s (mechanical) | From-Workspace ramp profile |
| in 2 | `omega_meas` | rad/s (mechanical) | BusSel `w` (NO Pn gain — both sides mechanical) |
| out 1 | `iq_ref` | A | → ZOH @ Tsc → MFPCC_ESO/5 |

**Config**: `UpdateMethod = 'DISCRETE'`, `SampleTime = 'Tsc'` (short chart — this works; for the
long MFPCC chart use INHERITED per §1).
PI with output clamp to `±iq_max` and **anti-windup** (conditional integration: do not commit the
integrator update while saturated). Parameters `Kp_w, Ki_w, iq_max, Tsc` — embedded as numeric
literals at build time, same rule and same failure mode as §1.

## 3. Sample/hold glue — why Unit Delay on the output, ZOH on the inputs

- **Inputs**: six `Zero-Order Hold` @ `Tsc` downsample the `Ts`-rate plant signals to the control
  grid. Plain rate sync — ZOH is correct here.
- **Outputs**: two `Unit Delay` @ `Tsc` (IC = 0) between the chart and SVPWM. They REALIZE the
  one-step digital delay that eq (20) compensates: the plant receives `u(k-1)`, and the chart's
  internal `ua_prev/ub_prev` equals the same `u(k-1)`.
  ⛔ Replacing them with ZOH applies `u(k)` in the SAME cycle → the compensated horizon is off by
  one step → the loop diverges. (`shared/building_blocks/api_notes.md` §6, deadbeat glue entry.)

## 4. ⚠ Double-`F` name collision

| Context | `F` means | Set where |
|---|---|---|
| PMSM mask variable `F` | viscous friction coefficient **B** (N·m·s) | InitFcn `F = <B value>` |
| ESO state `F` / outputs `Fa_o, Fb_o` | lumped disturbance `(−Rs·i_s − jωe·ψr)/Ls` (A/s) | chart internal |

Same letter, unrelated quantities, both unavoidable (mask name is fixed by the library block; `F` is
the paper's symbol). Never pass one where the other is expected; in build scripts name the friction
value `p.B` and assign `F = p.B` only in the InitFcn string.

## 5. Re-commit guard

After ALL wiring is done, re-apply both charts' `FunctionScript` + update-method/sample-time once
more. R2024b can silently drop chart sample-time settings when blocks are edited after the chart is
configured (`api_notes.md` chart-corruption caveat).
