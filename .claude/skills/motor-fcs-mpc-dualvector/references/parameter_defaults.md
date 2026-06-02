# Parameter Defaults — dual-vector FCS-MPC

Ask the user for machine + scenario. If they want the **paper reproduction**, use the Table-1 SPMSM set below; otherwise these are sensible starting points to be overridden.

## Paper SPMSM (Xu Yanping 2017, Table 1) — reproduction set

| Param | Value | Note |
|---|---|---|
| `Rs` | 0.2 Ω | stator resistance |
| `Ld = Lq = Ls` | 8.5e-3 H | SPMSM (non-salient) |
| `psif` | 0.24 V·s | PM flux linkage |
| `Pn` | 4 | pole pairs |
| `J` | 0.0012 kg·m² | rotor inertia |
| `B` (`F`) | 0 | viscous friction (not given ⇒ 0) |
| `Vdc` | 300 V | BEMF margin (~100 V peak BEMF @1000 rpm); co-sets ripple — see [design_decisions.md](design_decisions.md) |
| `Tsc` | 100e-6 s | control period (10 kHz, paper) |
| `Ts` | 1e-6 s | plant solver step (`Tsc/Ts = 100`) |
| `iq_max` | 9.4 A | q-axis current clamp |
| `id_ref` | 0 | SPMSM / mild-saliency IPMSM |
| Cost | L1 unweighted | paper eq 7 |

## Derived speed-PI gains (Symmetric Optimum, a=4)
```
Kt   = 1.5*Pn*psif            % = 1.44 N·m/A for the set above
T_eq = 5*Tsc                  % predictive current loop ~ deadbeat-fast; (2..5)*Tsc acceptable
Kp   = J/(a*Kt*T_eq)          % a = 4
Ki   = J/(a^3*Kt*T_eq^2)
```
Saturate `iq_ref` to `±iq_max` with back-calculation anti-windup; `1.5*Pn*psif*iq_max ≥ 1.3*TL_max`. Recompute with [scripts/speed_pi_design.m](../scripts/speed_pi_design.m).

## Validated reproduction scenarios
- **Scenario A (dynamic)**: no-load, `omega_ref` 800 rpm → step to 400 rpm @ 0.3 s, `StopTime` 0.6 s. Visual 4-check.
- **Scenario B (steady, for Table-2 ripple)**: `omega_ref` 1000 rpm constant, no-load, `StopTime` ~0.3 s; compute §E ripple on `id/iq` over a settled window (e.g. 0.2-0.3 s).

## Generalization sanity bounds (any machine)
- `Tsc/Ts ≥ 50` (slicer needs fine intra-period resolution; the reference uses 100).
- `Vdc` ≥ ~1.5× peak line BEMF at top speed (base Vdc/BEMF rule) — too low saturates the reachable vector set and the deadbeat clamp (N5) saturates constantly.
- `Lq/Ld ≤ 1.5` for `id_ref=0` (mild saliency); beyond that consider MTPA (out of v1 scope).
