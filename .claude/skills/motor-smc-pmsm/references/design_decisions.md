# Design Decisions Checklist

For each decision, present the user with options + recommended default, then get explicit confirmation.

| ID | Decision | Recommended | Alternatives |
|---|---|---|---|
| D01 | Sliding surface type | **PD-type `s = e + λ·de/dt`** (Filtered Derivative `s/(Tf·s+1)`) | PI-type `s = e + λ·∫e` ❌ wind-up + AW changes manifold geometry, Lyapunov void; Zero-order `s = e` ❌ no robustness margin; Higher-order ❌ overkill for v1 |
| D02 | Reaching law | **STA**: `u = K1·\|s\|^0.5·sgn(s) + K2·∫sgn(s)` | classic sgn ❌ chattering by design; boundary-layer sat ❌ degenerates to sgn when φ << \|s\|; Gao exponential ❌ chattering still present |
| D03 | λ_pd choice | **0.01 s** (settling time constant; sliding settling 4·λ = 40 ms) | Smaller (5 ms): faster but tighter inner-PI BW required; Larger (20 ms): slower but more robust |
| D04 | STA gains | **Auto-computed**: `K1 ≥ max(200, 1.5·√M·1.5)`, `K2 ≥ max(8000, 1.1·M·1.3)` where `M = (TL_max + B·ω_max)/J` | User-tuned ❌ Lyapunov violation invisibly |
| D05 | Filtered Derivative `Tf` | **`Tf = Tsc`** | Smaller ❌ HF gain too high → derivative kick; Larger ❌ phase lag erodes margin |
| D06 | iq_ref Saturation | **ON, ±iq_max** (B-CRIT mandatory) | OFF ❌ STA transient `\|s\|^0.5` term may exceed iq_max during reaching phase |
| D07 | id_ref | **`0`** (SPMSM / mild-saliency IPMSM v1 baseline) | MTPA `(ψf - sqrt(ψf² + 8(Lq-Ld)²·iq²))/(4(Lq-Ld))` for strong-saliency IPMSM (out of v1) |
| D08 | Inner PI design | **PZC**: `Kp_iq = ωc·Lq, Ki_iq = ωc·Rs` with `ωc_inner = 2000 rad/s` (≈ 5× SMC bandwidth) | SO ❌ current plant is RL (one-pole), not integral; PZC is natural choice |
| D09 | Cross-decoupling FF | **ON** (`Vd_ff = -Lq·ω_e·iq`, `Vq_ff = Ld·ω_e·id + ψ_f·ω_e`) | OFF ❌ PI must compensate BEMF + cross-coupling via integrator → saturation under load |
| D10 | Inner PI saturation + AW | **`LimitOutput='on' ±Vdc/√3` + `AntiWindup='clamping'`** | OFF ❌ uq command unbounded; AW='back-calculation' ❌ extra Kb tuning unneeded for v1 |
| D11 | Inverter modulation | **Anti_Park + SVPWM library blocks** | Hand-rolled inv-Park + Mux+Fcn ❌ verbose; MATLAB Function chart ❌ overhead |
| D12 | `Goto_The TagVisibility` | **`'global'`** (G-CRIT mandatory) | `'local'` (default) ❌ silent failure: Anti_Park internal From invisible → FOC degenerates to lab-frame open-loop |
| D13 | FF Mux input | **Port 1 = ω_e from `Gain_Pn_omega`** (independent block) (H-CRIT mandatory) | Port 1 = θ_e from `Gain_Pn` ❌ dimensional bug: `Vq_ff = θ_e·ψ_f` instead of `ω_e·ψ_f` |
| D14 | Plant friction `B` | **B > 0 in v1 baseline**, default 0.008 (C-CRIT envelope guard) | B = 0 not a theoretical blocker (STA convergence is damping-independent), but **outside v1 validated envelope** → re-validate gains if B≈0 |
| D15 | Vdc | **≥ 1.5·√3·ω_e_max·ψ_f** (`Vdc/√3` ≥ 1.5× peak phase BEMF headroom) | Tight (Vdc/BEMF ~1.13×) ❌ PI saturation continuous → tracking degrades |
| D16 | Solver | **Fixed-step `ode3` + `ZeroCrossControl='DisableAll'`** (E-CRIT) | Variable-step ❌ adaptive sub-stepping at sgn flips; ZC ON ❌ same |
| D17 | Step size | **1 µs** (= Tsc/50) | Tsc/100 OK; larger ❌ aliasing on PWM edges |
| D18 | Tsc | **50 µs** | 100 µs OK (slower chattering visualization); 25 µs OK (more compute) |
| D19 | ω_ref shape | **Linear ramp [0, ramp_time, sim_time] via From Workspace** | Step input ❌ saturates Saturation_iq throughout reaching phase, transient unobservable |
| D20 | ramp_time | **0.5 s** | Shorter (0.1 s) ❌ transient too aggressive; Longer (1.0 s) ❌ wastes sim time |
| D21 | Logger | **14-channel Mux + To Workspace, 4 SMC-specific (s/u_sta/e_w/de_w/dt)** | 17-ch with `sgn(s)`/`u_eq`/`u_sw` ❌ STA architecture has no separate switch/equiv terms |
| D22 | Scopes | **4 mandatory** (wm_RPM / iq / s / Te) | Logger only ❌ requires plot scripts; loses visual health check |
| D23 | Parameter injection | **InitFcn `set_param` + mask field var-name refs** | Hard-coded literals ❌ drifts |
| D24 | sim_time | **≥ ramp_time + 5·λ_pd + transient observation window**, typical 1.0 s | Shorter ❌ sliding-phase steady not observable |
| D25 | TL_max prior knowledge | **Always asked, no default** (C-CRIT) | Guessing TL_max ❌ Lyapunov bound becomes meaningless |
