# Design Decisions — three-vector FCS-MPC

Each row: the option, the choice taken, and why. "Faithful to the paper" ≠ "best for production"; where they differ, the **reproduction** default follows the paper and the production alternative is noted. Items tagged *(production alt)* are alternatives the reproduction does NOT adopt (they would break the Table-4 yardstick); they are valid production options.

| # | Decision | Choice | Rationale / alternative |
|---|---|---|---|
| 1 | Cost function | **L1 unweighted** `\|Δid\|+\|Δiq\|` (paper eq 7) | Reproduction fidelity. *(production alt)* an **L2-weighted** cost (`λ=20`) is a legitimate production option; offer as opt-in, default L1 when reproducing. Changing it changes vector selection. |
| 2 | Inductance form | **general `(Ld,Lq)`**, `Ld=Lq=Ls` injected for SPMSM | Matches the repo IPMSM-default plant; supports mild-saliency IPMSM. The paper is the zero-fidelity-cost special case `Ld=Lq` (T2 design note). The d-axis deadbeat (T3) genuinely uses `Ld`. |
| 3 | Candidate set | **6 synthesized expected vectors** (one per sector) | T6(b): each sector's adjacent active pair is synthesized into one optimal-duty expected vector; the cost ranks the 6. NOT the 8 basic vectors (that is single-vector FCS-MPC). |
| 4 | Time allocation | **dual-axis deadbeat 2×2 solve** (T3) | Forces BOTH `id` and `iq` to reference in one period — the three-vector novelty over dual-vector's q-only. *(production alt)* an equivalent **3×3 Cramer** system for `(ti,tj,tz)` is **algebraically identical** (verified symbolically) — a cleaner-to-port writing, not a different result. Either is acceptable; the 2×2 (eliminate `tz` first) is the minimal form. |
| 5 | Feasibility clamp | **T5 literal 3-case** (drop the out-of-range vector, renormalize) | Graceful degradation to dual/single vector. *(production alt)* a **proportional-scaling** clamp (scale `ti,tj` to preserve the voltage angle) is ≈ equivalent to T5 Case 1, not clearly better — left as a production option. |
| 6 | Zero-vector choice | **`u0=[000]` default** | Either zero vector is electrically zero. A loss-minimizing `u0/u7` pick (the one sharing most legs with `u_j`) reduces switching transitions — an optional refinement, not required for reproduction. |
| 7 | Equivalent voltage | **duty-weighted AVERAGE** `(u_i·ti + u_j·tj)/Tsc` (T4①) | The paper's own eqs 26-27 are already the clean `/Tsc` form (unlike the dual-vector paper, which needed a `/Tsc` correction). No dimensional fix required here. |
| 8 | `theta_e` source | **inline `∫Tsc·(Pn·w)`, persistent, wrapped** | Avoids the R2024b PMSM bus-`theta` trap; one angle for Plark + controller. DISCRETE @ Tsc (§A-CRIT). |
| 9 | Chart sample-time | **two-rate DISCRETE** (controller/theta @ Tsc, slicer @ Ts) | Validated. The slicer subdivides `Tsc` into 3 segments → must run at the fast rate. **Overrides base `api_notes §11`** (which prescribes INHERITED+ZOH for long charts): the R2024b `MATLABFunctionConfiguration.FunctionScript` + `UpdateMethod='Discrete'` config-object route avoids the §11 type-propagation deadlock (verified in rebuild). |
| 10 | Speed PI | **Symmetric Optimum, a=4** (B=0 ⇒ integrator+delay plant), sat `±iq_max`, back-calc anti-windup | `Kt=1.5·Pn·psif`, `T_eq≈(2..5)·Tsc`. For the paper machine → `Kp=0.4167, Ki=52.08` (same as dual-vector: same machine + Tsc + SO). See [scripts/speed_pi_design.m](../scripts/speed_pi_design.m). |
| 11 | PI domain | **rad/s** (convert both ref and meas) | SO gains are rad/s-native; avoids the 9.55× RPM-domain scaling pitfall. |
| 12 | Solver | **fixed-step discrete**, `FixedStep=Ts`, powergui Discrete | Discontinuous gate switching needs fixed-step. |
| 13 | Modulator | **none** (3-segment slicer applies vectors directly) | No SVPWM, no Anti_Park. The gate is the time-slice of three switch states. |
| 14 | Machine / freq / Vdc (reproduction) | paper Table-3 SPMSM, 10 kHz (`Tsc=100 μs`), `Vdc=300 V` | *(production alt)* alternative implementations may use a different machine / switching frequency — none adopted; reproduction targets the paper. `Vdc` unpublished → 300 V chosen (documented). |

## Out-of-v1-scope notes
- **Ripple is LOWER than the paper's Table 4, not matching.** Paper Table 4 (`Δi_d=0.21, Δi_q=0.24` @1000 rpm) is **experimental**; the ideal simulation (no deadtime, ideal switches, fine `Ts`, no sensor noise) gives lower ripple (validated rebuild: `Δi_d≈0.085, Δi_q≈0.158` at `Vdc=300, Tsc=100 μs`). This is the expected sim-vs-experiment direction. Reproduce the **method + trend** (TV ≪ ODC ≪ traditional), do NOT tune parameters to hit an experimental absolute. See [acceptance_criteria.md](acceptance_criteria.md).
- **`Vdc` co-sets ripple** (~`Vdc·Tsc/L`). State `Vdc` whenever quoting a ripple number.
- **Startup speed overshoot** under the current-limited SO start is expected/acceptable; add a reference slew (inline From-Workspace matrix, NOT a continuous RateLimiter — base H-CRIT) or detune SO (larger `a`) if undesired.
- L2-weighted cost, 3×3 Cramer form, proportional clamp, loss-optimal zero, MTPA `id_ref` for strong saliency, multi-step horizon — all deferred / optional.
