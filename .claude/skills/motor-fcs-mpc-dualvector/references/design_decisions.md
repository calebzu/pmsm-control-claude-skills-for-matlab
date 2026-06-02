# Design Decisions — dual-vector FCS-MPC

Each row: the option, the choice taken, and why. "Faithful to the paper" ≠ "best for production"; where they differ, the **reproduction** default follows the paper and the production alternative is noted.

| # | Decision | Choice | Rationale / alternative |
|---|---|---|---|
| 1 | Cost function | **L1 unweighted** `|Δid|+|Δiq|` (paper eq 7) | Reproduction fidelity. The single-vector `motor-fcs-mpc` uses **L2 weighted** `λd·ed²+λq·eq²` — a legitimate production option (lets you trade d vs q ripple). Offer L2 as an opt-in; default L1 when reproducing the paper. Changing it changes vector selection, so it is NOT a free swap. |
| 2 | Inductance form | **general `(Ld,Lq)`**, with `Ld=Lq=Ls` injected for SPMSM | Matches the repo IPMSM-default plant and supports mild-saliency IPMSM. The paper is the zero-fidelity-cost special case `Ld=Lq` (N2 design note). Adopting the general form costs nothing for SPMSM reproduction. |
| 3 | Vector candidate set | **7 distinct** `{V0,V1..V6}` (V7≡V0 dropped) | 6 active + 1 zero; V7 duplicates V0, so 7 evals cover the set without double-counting. |
| 4 | Search structure | **two-stage 7+7** (fix `V_opt1`, sweep `V_j`) | Avoids the 7×7=49 pair search. A vector that is sub-optimal as a standalone is rarely the better *first* vector, so the staged search reaches the same pair at ~1/3 the cost. Full 49-pair search is an option if you want provable global optimality. |
| 5 | `t_opt1` solve | **N3 q-axis deadbeat**, `1e-10` divide guard, **N5 clamp `[0,Tsc]`** | Forces `iq` to its reference in one period; clamp handles the unreachable-in-one-period case (saturation). |
| 6 | Equivalent voltage | **duty-weighted AVERAGE** `(u_opt1·Top + u_j·(Tsc−Top))/Tsc` | N4 dimensional fix: the paper's volt-second eqs 12-13 substituted into a prediction that already carries `Tsc/L` double-counts `Tsc`. The ÷Tsc average form is the dimensionally-correct realization. |
| 7 | `theta_e` source | **inline `∫Tsc·(Pn·w)`, persistent, wrapped** | Avoids the R2024b PMSM bus-`theta` trap; one angle for Plark + controller. DISCRETE @ Tsc (§A-CRIT). |
| 8 | Chart sample-time | **two-rate DISCRETE** (controller/theta @ Tsc, slicer @ Ts) | Validated structure. The slicer subdivides `Tsc`, so it must run at the fast rate — unlike single-vector, which holds one vector for the whole period and can use INHERITED+ZOH. An INHERITED+ZOH variant is conceivable but was not validated and still needs the slicer at the fast rate; prefer the DISCRETE form. |
| 9 | Speed PI | **Symmetric Optimum, a=4** (B=0 ⇒ integrator+delay plant), sat `±iq_max`, back-calc anti-windup | `Kt=1.5·Pn·psif`, `T_eq≈(2..5)·Tsc` (predictive loop is near-deadbeat-fast). See [scripts/speed_pi_design.m](../scripts/speed_pi_design.m). |
| 10 | PI domain | **rad/s** (convert both ref and meas) | SO gains are rad/s-native; converting both sides avoids the 9.55× RPM-domain scaling pitfall. |
| 11 | Solver | **fixed-step discrete**, `FixedStep=Ts`, powergui Discrete | Discontinuous gate switching needs fixed-step. |
| 12 | Modulator | **none** (time-slicer applies vectors directly) | Dual-vector is modulator-free; no SVPWM, no Anti_Park. The gate is the time-slice of two switch states. |

## Startup / out-of-v1-scope notes
- **Startup speed overshoot** (~37-45% on a no-load step-to-speed start): the current-limited start winds up the SO integrator faster than back-calculation unwinds it. Acceptable for the reproduction; if undesired, add a reference slew/ramp limiter (use From-Workspace inline matrix, NOT a continuous RateLimiter — base H-CRIT), detune SO (larger `a`), or lower the startup `iq` clamp.
- **`Vdc` co-sets ripple.** Switching-cycle ripple scales ~`Vdc·Tsc/L`. The paper's `Vdc` was not published, so a Table-2 ripple match is partly conditioned on the chosen `Vdc` (the validated rebuild used 300 V). State the `Vdc` whenever quoting a ripple number.
- **`id` ripple > `iq` ripple** is expected: the time allocation is q-axis-deadbeat, so the d-axis is not deadbeat-controlled.
- L2-weighted cost, full 49-pair search, MTPA `id_ref` for strong saliency, 3-vector / multi-step horizon — all deferred.
