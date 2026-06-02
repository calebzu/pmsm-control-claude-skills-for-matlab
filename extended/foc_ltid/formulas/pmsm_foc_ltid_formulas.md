> **Note:** the state-estimation theory (F2–F10) is cross-checked against the cited academic sources, with corrections noted inline. The base PMSM plant physics is reused by pointer from `pmsm_formulas.md`, not duplicated here.

# PMSM FOC + Load-Torque Linear Estimator (Luenberger / Kalman) + Feed-Forward Compensation — `foc-ltid` Formulas

Controller- and estimator-side formulas for the `foc-ltid` method: standard cascaded FOC
(dq current-loop PI + speed-loop PI) **augmented** with a linear **load-torque state
estimator** (continuous + discrete Luenberger observer, recursive + steady-state Kalman
filter) and a **feed-forward load-torque compensation** path into the torque/current
reference. The PMSM plant physics is **reused by pointer** from `pmsm_formulas.md` §0–§7
(dq voltage/flux/torque/mechanical/kinematic) and the speed-PI from §A — never duplicated
here.

> **Scope note:** F9.a (correlated-noise KF), F9.b (colored/Gauss-Markov
> noise KF via state augmentation) and F10 (continuous-time Kalman-Bucy filter) are **v1
> IN-SCOPE** advanced estimator variants.
> They are derived from academic literature only and reuse F2/F6/F7 **by pointer** (the predict
> step and the white-noise recursion are unchanged; only the gain/`P` update or the model is
> generalised).

**Conventions (inherited from `pmsm_formulas.md`):**
- Amplitude-invariant Park (2/3) → the `1.5·pn` torque factor (§1, §5).
- Rotor reference frame, d-axis aligned with ψf; motor convention (current into motor +).
- Symbols: `wm` mechanical speed [rad/s], `we = pn·wm`, `B` viscous friction, `ψf`/`psif`.
- `Kt = 1.5·pn·ψf` (SPMSM) — the torque constant, per §5 / §A.1.

---

## Coverage check vs `pmsm_formulas.md` (Step 1 result)

| Equation the method needs | Status | Where |
|---|---|---|
| dq voltage equations (with `we·Lq·iq`, `we·Ld·id`, `we·ψf` coupling/BEMF terms) | ✅ reuse | §2, §3 |
| dq flux `ψd = Ld·id+ψf`, `ψq = Lq·iq` | ✅ reuse | §4 |
| Torque `Te = 1.5·pn·[ψf·iq + (Ld−Lq)·id·iq]`; `Kt = 1.5·pn·ψf` | ✅ reuse | §5, §A.1 |
| Mechanical `J·dwm/dt = Te − B·wm − TL` | ✅ reuse | §6 |
| Kinematics `θe = pn·θm`, `we = pn·wm` | ✅ reuse | §7 |
| Outer speed-PI (SO/PZC/IMC tuning) | ✅ reuse | §A |
| **F1 dq current-loop PI tuning (IMC) + cross-coupling decoupling FF** | ➕ add | this doc |
| **F2 augmented mechanical state-space with `TL` as a state** | ➕ add | this doc |
| **F3 observability matrix + rank condition** | ➕ add | this doc |
| **F4 continuous Luenberger observer + pole placement of L** | ➕ add | this doc |
| **F5 discrete Luenberger observer (Euler/Tustin/ZOH)** | ➕ add | this doc |
| **F6 discrete recursive Kalman filter (predict/update)** | ➕ add | this doc |
| **F7 steady-state / stationary KF (DARE, `lqe`-style constant gain)** | ➕ add | this doc |
| **F8 feed-forward load-torque compensation law + DOB intuition** | ➕ add | this doc |
| **F9.a correlated process/measurement-noise KF (cross-term `M`)** | ➕ add | this doc |
| **F9.b colored/Gauss-Markov noise KF via state augmentation** | ➕ add | this doc |
| **F10 continuous-time Kalman-Bucy filter (CARE limit)** | ➕ add | this doc |

---

## F1 — Inner dq current-loop PI (IMC tuning) + cross-coupling decoupling feed-forward

### F1.a Plant seen by each current loop (declared dependence)
**Plant: `pmsm_formulas.md` §2 (d-axis), §3 (q-axis).** After the cross-coupling terms are
cancelled by the decoupling feed-forward (F1.c), each axis reduces to a **first-order RL
plant**:
```
d-axis:  Ld·(d id/dt) = Vd' − Rs·id      ⇒   id(s)/Vd'(s) = 1 / (Ld·s + Rs)
q-axis:  Lq·(d iq/dt) = Vq' − Rs·iq      ⇒   iq(s)/Vq'(s) = 1 / (Lq·s + Rs)
```
where `Vd' = Vd + we·Lq·iq`, `Vq' = Vq − we·Ld·id − we·ψf` are the **decoupled** axis voltages
(the rotational/BEMF terms moved to the feed-forward path, F1.c).

### F1.b PI gains by Internal Model Control (pole-zero cancellation of the RL pole)
Design knob = desired closed-loop current-loop bandwidth `α_c` [rad/s]. IMC inverts the
first-order plant and cancels its pole, giving the closed loop `id/id_ref = α_c/(s+α_c)`
(a clean first-order lag of bandwidth `α_c`):
```
d-axis:   Kp_d = α_c · Ld      Ki_d = α_c · Rs
q-axis:   Kp_q = α_c · Lq      Ki_q = α_c · Rs
```
- **PI form:** `Vd'(s) = (Kp_d + Ki_d/s)·(id_ref − id)`, likewise q.
- **Zero/pole cancellation check:** PI zero at `−Ki/Kp = −Rs/L` cancels the plant pole at
  `−Rs/L`; open loop becomes `α_c/s`; closed loop `α_c/(s+α_c)`. ✔
- **Bandwidth guidance:** `α_c` is set 5–20× below the PWM/sampling angular frequency; the
  outer speed loop then treats the current loop as a lag with `T_eq ≈ 1/α_c` (ties to
  `pmsm_formulas.md` §A.1 `T_eq`).
- **Units:** `Kp` in V/A (= Ω), `Ki` in V/(A·s) (= Ω/s). `α_c` in rad/s.

### F1.c Cross-coupling decoupling feed-forward (the dq de-coupling terms)
Added to the PI outputs so the two axes become independent first-order loops:
```
Vd = Vd_PI − we·Lq·iq                       (cancels the +we·Lq·iq term in §2)
Vq = Vq_PI + we·Ld·id + we·ψf               (cancels −we·Ld·id and the BEMF −we·ψf in §3)
```
- Sign convention: each FF term is the **negative** of the corresponding coupling term in the
  plant voltage equation (§2/§3), so it cancels in closed loop. `+we·ψf` on q is the **BEMF
  compensation** (lets `iq` build without the PI fighting back-EMF).
- **Role:** turns the genuinely coupled dq plant into two SISO RL loops so F1.b tuning is exact.

### F1 audit
- **Role in method:** the FOC inner loop `iq` tracks `iq_ref` (from speed PI + F8 FF) fast
  enough that the speed/estimator loop can treat `iq ≈ iq_ref` (`pmsm_formulas.md` §A.1 A7).
- **Plant dependence:** §2, §3 (voltage), §4 (flux), §5 (`Kt`).
- **Sources (≥2 independent):**
  - **S1 — Harnefors & Nee 1998**, IEEE Trans. Ind. Appl. 34(1):133–141, DOI
    <https://doi.org/10.1109/28.658735>. IMC current control of AC machines → `Kp = α·L`,
    `Ki = α·R`. Anchor: the IMC-tuned PI result is
    the paper's central contribution (the "model-based" gains in machine parameters + `α`).
  - **S5 — Ogata, *Modern Control Engineering* 5th ed.** (ISBN 978-0-13-615673-4),
    Ch. on root-locus / PI design — pole-zero cancellation of a first-order plant pole.
  - **Corroboration:** MathWorks *PMSM Current Controller* doc states the same `Kp = α_c·L`,
    `Ki = α_c·R`.
- **Confidence:** HIGH (formula multiply-corroborated). No reference-specific numerics.

---

## F2 — Augmented mechanical state-space with load torque as a state

### F2.a Continuous model
Take the mechanical equation **Plant: `pmsm_formulas.md` §6** `J·dwm/dt = Te − B·wm − TL`
and **augment** the unknown load torque `TL` as a second state with a *slowly-varying /
constant-disturbance* model `dTL/dt ≈ 0` (random-walk in the stochastic view):
```
state    x = [ wm ; TL ]                         (units: [rad/s ; N·m])
input    u = Te    (= Kt·iq, the controllable electromagnetic torque; §5)
output   y = wm    (measured mechanical speed)

ẋ = A·x + B·u + G·w
y = C·x + v

A = [ −B/J   −1/J ;        B = [ 1/J ;        C = [ 1   0 ]     D = 0
       0      0   ]              0   ]
G = [ 0 ; 1 ]   (process-noise enters the TL state — the random-walk driving term)
```
Component form:
```
ẇm  = −(B/J)·wm − (1/J)·TL + (1/J)·Te
ṪL  = 0  (+ w, process noise in the stochastic/KF view)
```

### F2.b The `dTL/dt ≈ 0` modeling assumption (literature standpoint)
- Deterministic (Luenberger) view: the load is modelled as **piecewise-constant**; between
  step changes `ṪL = 0` exactly, and at a step the observer error transient re-converges
  (this is the classic "constant-disturbance observer" / Lorenz servo-drive model, S2).
- Stochastic (Kalman) view: `ṪL = w` with `w` zero-mean white → `TL` is a **random walk**
  (Wiener process); the process-noise intensity `Q_TL` encodes how fast the load may drift.
  Larger `Q_TL` → faster but noisier `TL` tracking. (S3, S6.)
- Why valid: mechanical load dynamics are *much slower* than the electrical/observer
  bandwidth, so a zeroth-order (constant) internal model captures the disturbance well enough
  for the observer's integral action to drive steady-state estimation error to zero for
  constant `TL`.

### F2 audit
- **Role:** the plant the F4/F5/F6/F7 estimators run on; its second state `T̂L` is what F8
  feeds forward.
- **Plant dependence:** §6 (mechanical), §5 (`Te = Kt·iq`).
- **Sources:**
  - **S2 — Lorenz & Van Patten 1991**, IEEE Trans. Ind. Appl. 27(4):701–705, DOI
    <https://doi.org/10.1109/28.85485> — canonical disturbance-torque-as-state servo observer.
  - **S3 — Horváth 2024**, *Automatika* 65(3):1201–1212, DOI
    <https://doi.org/10.1080/00051144.2024.2354643> — augments load torque as a state for
    SPMSM. Abstract first sentence (verbatim): "The paper introduces novel position sensorless
    state estimators designed to improve robustness against permanent magnet flux linkage
    variations in PMSMs."
- **Confidence:** HIGH (architecture canonical). No reference-specific numerics.

---

## F3 — Observability of the augmented system

### F3.a Observability matrix and rank condition
For the 2nd-order `(A, C)` of F2 with `y = wm`:
```
O = [ C ;            =  [ 1      0   ;
      C·A ]              −B/J   −1/J ]

det(O) = (1)·(−1/J) − (0)·(−B/J) = −1/J
```
- **Rank condition:** `rank(O) = 2 = n` **iff** `J` is finite and nonzero (`1/J ≠ 0`), which is
  always true for a real rotor. Independent of `B`.
- **Interpretation:** measuring only speed `wm` is sufficient to reconstruct the unmeasured
  load torque `TL`, because `TL` enters `ẇm` with coefficient `−1/J ≠ 0` — the disturbance is
  *visible* through the acceleration. This is why a single speed sensor suffices for the LTID
  estimator.

### F3 audit
- **Role:** licenses F4–F7 (an observer/KF only exists if the pair is observable).
- **Plant dependence:** uses F2's `A, C` (built from §6).
- **Sources:**
  - **S5 — Ogata 5th ed.**, observability matrix `O = [C; CA; …; CAⁿ⁻¹]`, rank-n test
    (Ch. 9).
  - **S3 — Horváth 2024** — "includes detailed observability analyses for each model used in
    state estimation" (abstract, verbatim phrase), corroborating observability of the
    load-torque-augmented model. DOI <https://doi.org/10.1080/00051144.2024.2354643>.
- **Confidence:** HIGH (closed-form `det(O) = −1/J`, trivially nonzero).

---

## F4 — Continuous Luenberger observer + pole placement of L

### F4.a Observer equation
```
x̂̇ = A·x̂ + B·u + L·(y − ŷ),     ŷ = C·x̂
   = (A − L·C)·x̂ + B·u + L·y          ← compact "[B  L]·[u; y]" form
```
with observer gain `L = [ l1 ; l2 ]` (`l1` → speed-correction [1/s], `l2` → load-torque
correction [N·m·s/rad]). Estimation error `e = x − x̂` obeys `ė = (A − L·C)·e`, so `L` is
chosen to place the eigenvalues of `(A − L·C)`.

### F4.b Pole placement by characteristic-polynomial matching
```
A − L·C = [ −B/J − l1    −1/J ;
            −l2           0    ]

char. poly:  det(sI − (A−L·C)) = s² + (B/J + l1)·s − (l2/J)
desired:     s² + 2·ζo·ωo·s + ωo²        (double-real or damped pair, observer poles)
⇒   l1 = 2·ζo·ωo − B/J
    l2 = −J·ωo²
```
> **[Correction]** The original draft had the constant term as `+(l2/J)` (⇒ `l2=+J·ωo²`),
> which yields an unstable observer pole pair `s²+2ζoωo·s−ωo²`. Independent re-derivation of
> `det(sI−(A−L·C))` with `A−L·C=[−B/J−l1, −1/J; −l2, 0]` gives the constant term `−(l2/J)`
> (⇒ `l2=−J·ωo²`); `l1` is unaffected. The build uses numeric `place()` (F5), so simulations
> were never affected — this corrects the closed-form documentation only.
- **Speed/damping guidance:** place observer poles **3–5× faster** than the closed-loop
  control bandwidth (speed-loop crossover `ωc`), i.e. `ωo ≈ (3…5)·ωc`, with `ζo ≈ 0.7–1.0`
  (real/critically-damped preferred to avoid estimation overshoot). Too fast → noise
  amplification (motivates the KF, F6, which optimises this trade-off statistically).
- **Ackermann form** (equivalent, for the dual): `Lᵀ = place(Aᵀ, Cᵀ, desired_poles)`.

### F4 audit
- **Role:** deterministic estimator of `[ŵm; T̂L]` from `wm` measurement; `T̂L` → F8.
- **Plant dependence:** F2 `(A, B, C)`.
- **Sources:**
  - **S5 — Ogata 5th ed.**, Ch. 10 — full-order state observer `x̂̇ = (A−LC)x̂ + Bu + Ly`,
    pole placement, Ackermann's formula.
  - **S2 — Lorenz & Van Patten 1991**, DOI <https://doi.org/10.1109/28.85485> — observer-based
    disturbance/load-torque estimation for servo feed-forward (the application anchor).
  - **S4 — Dursun & Özçıra 2025**, *Energies* 18(4):883, DOI
    <https://doi.org/10.3390/en18040883> — filtering observer estimating disturbance torque
    with **arbitrary pole placement** in a PMSM drive (independent corroboration of F4.b).
- **Confidence:** HIGH for structure/pole-placement; `3–5×` rule is standard engineering
  guidance (flag: rule-of-thumb, tune per plant).

---

## F5 — Discrete Luenberger observer (sampled at Ts)

### F5.a Discretization of the estimator matrices
Sample at `Ts`. Three standard maps (use ZOH for fidelity, forward-Euler for simplicity):
```
ZOH (exact for piecewise-constant u):   Ad = e^{A·Ts},   Bd = (∫₀^{Ts} e^{A·τ} dτ)·B
Forward-Euler (1st-order):              Ad ≈ I + A·Ts,   Bd ≈ B·Ts
Tustin / bilinear:                      Ad = (I + A·Ts/2)(I − A·Ts/2)⁻¹,  Bd from same map
Cd = C,   Dd = D = 0
```

### F5.b Discrete observer update (prediction-correction form)
```
predict:   x̂⁻[k] = Ad·x̂⁺[k−1] + Bd·u[k−1]
correct:   x̂⁺[k] = x̂⁻[k] + Ld·( y[k] − Cd·x̂⁻[k] )
```
or the one-line current-estimator form `x̂[k] = Ad·x̂[k−1] + Bd·u[k−1] + Ld·(y[k−1] − Cd·x̂[k−1])`.
- **Discrete pole placement:** place eigenvalues of `(Ad − Ld·Cd)` inside the unit circle at
  `z = e^{s_pole·Ts}` (map the continuous F4.b poles). Stability requires `|eig| < 1`.
- **Guidance:** keep `Ts` small enough that `ωo·Ts ≲ 0.1–0.3` so Euler discretization error
  stays negligible; otherwise use ZOH/Tustin.

### F5 audit
- **Role:** the implementable (digital) form of F4 for the build.
- **Plant dependence:** F2 `(A,B,C)` discretized.
- **Sources:** **S5 — Ogata 5th ed.** (discrete-time state-space: forward-Euler / ZOH /
  bilinear discretization; discrete observers). Corroborated by **S6 — Simon**
  (discrete-time state model is the basis of the discrete KF, F6).
- **Confidence:** HIGH (standard discretization).

---

## F6 — Discrete recursive Kalman filter (predict / update)

Stochastic optimal estimator for the **same** F2 model discretized (F5.a) to `(Ad, Bd, H)`,
with `H = Cd = [1 0]`, process noise covariance `Q` (n×n, here 2×2), measurement noise
variance `R` (scalar, on the speed measurement).

### F6.a Recursion
```
— Time update (predict) —
x̂⁻[k] = Ad·x̂⁺[k−1] + Bd·u[k−1]
P⁻[k]  = Ad·P⁺[k−1]·Adᵀ + Q

— Measurement update (correct) —
S[k]   = H·P⁻[k]·Hᵀ + R                         (innovation covariance, scalar here)
K[k]   = P⁻[k]·Hᵀ · S[k]⁻¹                      (Kalman gain, n×1)
x̂⁺[k] = x̂⁻[k] + K[k]·( y[k] − H·x̂⁻[k] )        (innovation = measured − predicted speed)
P⁺[k]  = (I − K[k]·H)·P⁻[k]                      (Joseph form for numerical robustness:
                                                 P⁺ = (I−KH)P⁻(I−KH)ᵀ + K·R·Kᵀ)
```

### F6.b Q and R as tuning knobs
- **R** = variance of the speed-measurement noise (sensor/encoder-difference noise). Larger R
  → filter trusts the model more → smoother but slower `T̂L`.
- **Q** = process-noise covariance; its `(2,2)` entry `Q_TL` is the **dominant LTID knob**: it
  models how fast `TL` may change (random-walk intensity, F2.b). Larger `Q_TL` → faster
  `T̂L` response to load steps, more noise passed through. `Q_wm` (the `(1,1)` entry) absorbs
  mechanical-model mismatch (e.g. uncertain `B/J`).
- **Ratio intuition:** only the ratio `Q/R` shapes the steady-state gain (F7); scaling both by
  the same factor leaves `K∞` unchanged.

### F6 audit
- **Role:** statistically-optimal version of F4/F5 — chooses the correction gain to minimise
  estimation-error variance under the `Q,R` noise model; `T̂L = x̂⁺[2]` → F8.
- **Plant dependence:** F2 `(A,B,C)` discretized (F5.a).
- **Sources:**
  - **S6 — Simon, *Optimal State Estimation*** (ISBN 978-0-471-70858-2), discrete KF
    predict/update recursion + Joseph form.
  - **S3 — Horváth 2024**, DOI <https://doi.org/10.1080/00051144.2024.2354643> — KF-family
    load-torque estimation for SPMSM (application anchor). Abstract first sentence (verbatim):
    "The paper introduces novel position sensorless state estimators designed to improve
    robustness against permanent magnet flux linkage variations in PMSMs."
- **Confidence:** HIGH (textbook-standard recursion).

---

## F7 — Steady-state / stationary Kalman filter (DARE, `lqe`-style constant gain)

### F7.a Algebraic-Riccati (constant-gain) form
For LTI `(Ad, H)` with constant `Q, R`, the error covariance converges to a fixed `P∞` and the
gain to a constant `K∞`. `P∞` solves the **discrete algebraic Riccati equation (DARE)**:
```
P∞ = Ad·P∞·Adᵀ + Q − Ad·P∞·Hᵀ·(H·P∞·Hᵀ + R)⁻¹·H·P∞·Adᵀ
K∞ = P∞·Hᵀ·(H·P∞·Hᵀ + R)⁻¹
```
Steady-state filter (drop the `P[k]` recursion; run with constant `K∞`):
```
x̂⁻[k] = Ad·x̂⁺[k−1] + Bd·u[k−1]
x̂⁺[k] = x̂⁻[k] + K∞·( y[k] − H·x̂⁻[k] )
```
- **MATLAB:** `K∞` from `kalman`/`lqe` (continuous `lqe(A,G,C,Q,R)` → `L`; discrete
  `dlqe`/`kalman` on `(Ad,…)`). The continuous-time analogue solves the **CARE**
  `A·P + P·Aᵀ − P·Cᵀ·R⁻¹·C·P + G·Q·Gᵀ = 0`, `L = P·Cᵀ·R⁻¹`.

### F7.b When valid & equivalence to a fixed-gain Luenberger observer
- **Valid when:** `(Ad, H)` observable (F3 ✔), `(Ad, G√Q)` has no unobservable unstable mode,
  and `Q, R` constant. Then `P[k] → P∞` from any `P[0] ≥ 0`.
- **Reduction:** with constant `K∞`, the update is **structurally identical to the discrete
  Luenberger observer (F5.b)** with `Ld := K∞`. So the steady-state KF *is* a Luenberger
  observer whose gain was chosen optimally (by `Q/R`) instead of by manual pole placement —
  the two F-families converge to the same fixed-gain estimator, differing only in how `L` is
  picked. This is the bridge that lets the build expose "Luenberger" and "Kalman" as two
  tunings of one estimator block.

### F7 audit
- **Role:** cheap (no per-step Riccati) optimal estimator; the production estimator when
  `Q,R` are time-invariant; demonstrates Luenberger↔Kalman equivalence.
- **Plant dependence:** F2 discretized; F3 observability is the validity precondition.
- **Sources:**
  - **S6 — Simon, *Optimal State Estimation*** (ISBN 978-0-471-70858-2) — steady-state KF via
    DARE, constant-gain filter.
  - **S5 — Ogata 5th ed.** — duality observer↔regulator and Riccati-based gain (LQ/observer
    duality), supports the `lqe` reduction.
- **Confidence:** HIGH for DARE/equivalence. Flag (MEDIUM): the exact `dlqe` vs `lqe`
  current-vs-prediction-form convention should be pinned to the chosen MATLAB function at
  build time (implementation-time verification item).

---

## F8 — Feed-forward load-torque compensation law

### F8.a The compensation law
The speed-PI (**Plant/control: `pmsm_formulas.md` §A**) produces a feedback torque demand
`Te_fb`. Add the **estimated load torque** `T̂L` (from F4/F5/F6/F7) as a feed-forward term, then
convert to the current reference via the torque constant:
```
Te_ref  = Te_fb + T̂L                          (feed-forward injection)
iq_ref  = Te_ref / Kt = (Te_fb + T̂L) / Kt ,   Kt = 1.5·pn·ψf   (SPMSM, §5/§A.1)
id_ref  = 0                                    (SPMSM baseline; IPMSM-MTPA out of v1 scope)
```
(Optionally also feed-forward the friction `B·ŵm` if `B` is known; the estimator's `T̂L`
already absorbs un-modelled friction when `B` is set to 0 in the estimator model.)

### F8.b Why it improves disturbance rejection (DOB intuition)
- **Physical:** at a load step, the PI must first *accumulate speed error* before its integral
  ramps the torque to match the new load → a speed dip/overshoot. The FF path supplies the
  matching torque `T̂L` **immediately** (as soon as the estimator tracks it), so the PI sees a
  near-cancelled disturbance and the speed dip shrinks.
- **Transfer-function intuition:** the load-to-speed disturbance transfer `wm/TL` is reduced by
  roughly the factor `(1 − Ĝ)` where `Ĝ` is the estimator's tracking transfer for `TL` (the
  observer/KF bandwidth). Inside the estimator bandwidth `Ĝ ≈ 1` → disturbance rejection is
  near-ideal; outside it, the PI handles the residual. This is the standard **disturbance-
  observer (DOB)** result: FF compensation of an *estimated matched disturbance* attenuates the
  closed-loop sensitivity to that disturbance up to the estimator bandwidth, **without**
  retuning or de-stabilising the speed PI (the FF is outside the feedback loop).
- **Limit:** only the *matched* mechanical disturbance `TL` (acting in the torque channel) is
  compensated; sensor noise on `wm` propagates through the estimator (the Q/R or pole choice
  sets the noise–speed trade-off).

### F8 audit
- **Role:** the payoff of the whole method — turns `T̂L` into improved speed-loop disturbance
  rejection.
- **Plant dependence:** §A (speed PI), §5/§A.1 (`Kt`), §6 (where `TL` acts).
- **Sources:**
  - **S2 — Lorenz & Van Patten 1991**, DOI <https://doi.org/10.1109/28.85485> — observed
    disturbance/load torque used as **feed-forward** in the servo torque command (canonical).
  - **S4 — Dursun & Özçıra 2025**, *Energies* 18(4):883, DOI
    <https://doi.org/10.3390/en18040883> — disturbance-torque estimate used to improve FOC
    PMSM performance (feed-forward compensation), independent corroboration.
- **Confidence:** HIGH for the law/intuition. Flag (MEDIUM): the exact `(1−Ĝ)` sensitivity
  reduction is an *intuition/approximation*, not an exact closed-form for this 2nd-order plant
  — mark for implementation-time if a rigorous sensitivity bound is needed.

---

## F9.a — Kalman filter with correlated process & measurement noise

> **v1 IN-SCOPE**. Advanced
> variant of F6; selected when the speed measurement is derived by *differencing* a state.

### F9.a.1 The standard-KF assumption that this relaxes
F6 assumes the discrete process noise `w[k]` and measurement noise `v[k]` are **mutually
uncorrelated**: `E[w[k]·v[j]ᵀ] = 0` for all `k, j`. F9.a relaxes the *same-step* case to a
nonzero cross-covariance:
```
E[ w[k]·v[k]ᵀ ] = M        (n×m;  here n=2, m=1  ⇒  M is 2×1)
E[ w[k]·v[j]ᵀ ] = 0        for k ≠ j   (still white across steps)
```
(`M` is also written `S` or `N` in the literature; Simon uses `M`.)

### F9.a.2 Modified gain and covariance update (Simon form)
The time-update (predict) of F6.a is **unchanged**:
```
x̂⁻[k] = Ad·x̂⁺[k−1] + Bd·u[k−1]
P⁻[k]  = Ad·P⁺[k−1]·Adᵀ + Q
```
The measurement-update gain picks up the cross-term `M` in **both** the numerator and the
innovation-covariance denominator:
```
K[k]   = ( P⁻[k]·Hᵀ + M ) · ( H·P⁻[k]·Hᵀ + H·M + Mᵀ·Hᵀ + R )⁻¹
x̂⁺[k] = x̂⁻[k] + K[k]·( y[k] − H·x̂⁻[k] )
P⁺[k]  = P⁻[k] − K[k]·( H·P⁻[k] + Mᵀ )          (≡ (I−K·H)·P⁻ − K·Mᵀ)
```
- **Units / shapes:** `M` [ (state-unit)·(meas-unit) ] = here `[ (rad/s)·(rad/s) ; (N·m)·(rad/s) ]`;
  `K` [ state-unit / meas-unit ]; the inverse argument is the scalar innovation covariance
  `S[k] = H·P⁻·Hᵀ + H·M + Mᵀ·Hᵀ + R`.
- **Reduction:** set `M = 0` → `K = P⁻·Hᵀ·(H·P⁻·Hᵀ+R)⁻¹`, `P⁺ = (I−K·H)·P⁻` — **exactly F6.a**.
  So F9.a *is* F6 with `M` carried through; F6 is the `M→0` special case (declared by pointer).
- **Equivalent "decorrelation" route (Brown & Hwang / Gelb):** instead of the modified gain,
  one defines a transformed measurement that subtracts a multiple of the dynamics so the
  redefined process noise is uncorrelated with `v`, then applies the *standard* F6 recursion to
  the transformed model. Both routes give the identical estimate; the modified-gain form above
  is the more direct.

### F9.a.3 When correlation arises physically (the LTID trigger)
- **Differenced-speed measurement.** If `wm` is not measured directly but obtained by
  *differencing position* (`wm ≈ (θ[k]−θ[k−1])/Ts`) or by a discrete-derivative/observer-difference,
  the measurement noise inherits the **same** sample of mechanical/quantisation disturbance that
  also drives the process equation — so `w[k]` and `v[k]` share a common term ⇒ `M ≠ 0`. This is
  the canonical "measurement derived by differencing a state" case (Simon, Gelb).
- **Discretisation of a continuous model with one white source** feeding both the dynamics and
  the output (e.g. ZOH-sampling a plant whose continuous process noise also appears in the
  sampled measurement) induces `M ≠ 0` even when the continuous noises were independent.
- **Decision for the build:** if the speed signal is a clean encoder/resolver reading, keep
  `M = 0` (use F6). Only switch to F9.a when the speed feedback is a *differenced* quantity and
  the resulting bias/variance inflation is measurable.

### F9.a audit
- **Role:** corrects the F6 gain when speed feedback is a differenced/derived signal so the
  estimate stays optimal (minimum-variance) and unbiased; `T̂L = x̂⁺[2]` → F8 as before.
- **Plant dependence:** **F2** `(A,B,C)` discretised (F5.a); **F6** recursion **by pointer**
  (only the gain/`P⁺` lines change). Does not re-derive predict step.
- **Sources (≥2 independent):**
  - **S6 — Simon, *Optimal State Estimation*** (ISBN 978-0-471-70858-2) — Ch. 7
    "Kalman filter generalizations", *correlated process and measurement noise* section: the
    gain `K = (P⁻Hᵀ+M)(HP⁻Hᵀ+HM+MᵀHᵀ+R)⁻¹` and matching `P⁺` update.
  - **S7 — Brown & Hwang 4th ed.** (ISBN 978-0-470-60969-9, Wiley 2012) — generalising the
    discrete KF: correlated measurement & process noise handled by the decorrelation /
    measurement-transformation route. ISBN verified (Wiley).
  - **S9 — Khalid 2024**, *Applications and Optimizations of Kalman Filter and Their Variants*
    (IntechOpen), DOI <https://doi.org/10.5772/intechopen.1005079> — peer-reviewed chapter,
    cross-covariance gain `T = S·R⁻¹` with `S` the process/measurement cross-covariance
    (verbatim eqs cached). Independent corroboration of the cross-term structure.
- **Confidence:** HIGH for the modified gain/`P⁺` (textbook-standard, multiply-corroborated).
  Flag (MEDIUM): the exact `P⁺` algebraic form (`P⁻−K(HP⁻+Mᵀ)` vs an equivalent Joseph-style
  rearrangement) and whether the build's KF API exposes an `M`/`N` argument should be pinned at
  implementation-time (`kalman`/`dlqe` in MATLAB take an `N` cross-term argument — verify sign convention).

---

## F9.b — Colored / band-limited (pink) noise Kalman filter via state augmentation

> **v1 IN-SCOPE**. Strict generalisation of F2's random-walk disturbance
> model; selected when the load/disturbance noise is time-correlated rather than white.

### F9.b.1 Why the white-noise KF is *wrong* under coloured noise
F6/F7 are optimal **only** when `w` and `v` are white (uncorrelated in time). If the real
disturbance is **coloured** (its samples are autocorrelated — a band-limited / "pink" process
with a finite correlation time), feeding it to a white-noise KF violates the KF's core
assumption: the innovations are no longer white, the filter under-/over-weights measurements,
and the estimate is **biased and sub-optimal** (error covariance no longer minimal, and the
reported `P` is optimistic). The remedy is to **model the colour explicitly** with a shaping
filter and let the augmented KF "whiten" it internally.

### F9.b.2 First-order Gauss-Markov shaping filter (the extra state)
Model the coloured disturbance component `n(t)` as the output of a first-order linear filter
driven by **white** noise `ζ` (this is the standard band-limited / exponentially-correlated
"pink-ish" model; PSD rolls off at `−20 dB/dec` above `a`):
```
ṅ = −a·n + ζ ,      ζ ~ white,  E[ζ²]=q_ζ                (continuous Gauss-Markov)
⇒ autocorrelation  R_n(τ) = σ_n²·e^{−a|τ|},   σ_n² = q_ζ/(2a),   τ_corr = 1/a
```
- `a` [1/s] = inverse correlation time (bandwidth of the colour); `a→∞` (with `q_ζ` scaled) → white limit; `a→0` → random walk (integrator).

### F9.b.3 Augmented state-space (append the shaping state to F2)
Take **F2** `x = [wm; TL]` and append the coloured-noise state `n`. Two physically-distinct
placements (pick per where the colour enters):

**(A) Coloured *load/process* disturbance** — the load torque itself is coloured rather than a
pure random walk. Replace F2's `ṪL = w` (white) by a Gauss-Markov `TL`:
```
state  x_aug = [ wm ; TL ]      with    ṪL = −a·TL + ζ      (TL now IS the shaping state)
A_aug  = [ −B/J   −1/J ;        B_aug = [ 1/J ; 0 ] ,   G_aug = [ 0 ; 1 ]  (ζ enters TL)
           0      −a   ]
C_aug  = [ 1   0 ]              (speed still measured)
```
This is the **minimal** coloured-disturbance model and the most relevant LTID case: F2 is
recovered exactly as **`a → 0`** (`ṪL = ζ` random walk). Larger `a` → the estimator expects the
load to mean-revert faster.

**(B) Coloured *measurement* noise on the speed** — the speed-sensor noise is band-limited.
Keep F2's `[wm; TL]` and append a 3rd state `n` for the coloured measurement noise; the
measurement then reads speed **plus** the coloured component:
```
state  x_aug = [ wm ; TL ; n ] ,     ṅ = −a·n + ζ
A_aug  = [ −B/J  −1/J   0  ;        B_aug = [ 1/J ; 0 ; 0 ]
            0     0     0  ;        G_aug = [ 0 ; w-port ; 1 ]   (ζ drives n)
            0     0    −a ]
C_aug  = [ 1   0   1 ]              (measurement picks up the coloured component n)
                                    ← the "[1 0 1]-type" structure
```
With C_aug = [1 0 1] the *direct* white measurement noise `v` can be made small/zero (its colour
is now a state), but a tiny `R` is kept for numerical conditioning (the known "perfect
measurement" ill-conditioning, Brown & Hwang).

### F9.b.4 Run the standard KF on the augmented model (no new filter math)
Discretise `(A_aug, B_aug, G_aug, C_aug)` by F5.a and run the **standard F6 predict/update**
(or F7 steady-state) **verbatim** on the augmented state. The augmentation is the whole trick:
the KF now sees a *white*-driven model (`ζ` is white), so its optimality is restored; the
coloured statistics are captured by the extra state's dynamics rather than violated inside a
white-noise filter. The disturbance/coloured estimate is the relevant augmented component
(`T̂L` for case A → F8; the speed estimate is read from the `wm` state, with the coloured
sensor-noise state subtracted in case B).

### F9.b audit
- **Role:** restores KF optimality when the disturbance or sensor noise is time-correlated;
  generalises F2/F6/F7 without changing the filter equations — only the model is augmented.
- **Plant dependence:** **F2** (base `[wm; TL]` state-space) extended; **F6/F7** recursion reused
  **by pointer** (unchanged, run on `(A_aug, C_aug)`). White-noise F2/F6 recovered as `a → 0`
  (case A) — the limit is declared, not re-derived.
- **Sources (≥2 independent):**
  - **S6 — Simon, *Optimal State Estimation*** (ISBN 978-0-471-70858-2) — Ch. 7, *coloured /
    correlated noise* via state augmentation with a shaping filter; recovery of the white case
    as a limit.
  - **S7 — Brown & Hwang 4th ed.** (ISBN 978-0-470-60969-9) — shaping filters for coloured
    process & measurement noise; first-order Gauss-Markov model; augmented-state KF and the
    perfect-measurement conditioning caveat. ISBN verified.
  - **S8 — Gelb (ed.), *Applied Optimal Estimation*** (ISBN 978-0-262-57048-0, MIT Press 1974)
    — Ch. 4, exponentially-correlated (Gauss-Markov) processes and state augmentation for
    coloured noise. ISBN verified (MIT Press).
- **Confidence:** HIGH for the augmentation method and the `a→0` recovery of F2 (canonical).
  Flag (MEDIUM): the exact `G_aug` port for case A (whether the build injects `ζ` only into the
  `TL` row) and the `R→0` conditioning in case B are implementation-time implementation choices.

---

## F10 — Continuous-time Kalman-Bucy filter (continuous Riccati form)

> **v1 IN-SCOPE**. Continuous-time analogue of F6/F7; the limit the
> discrete KF approximates as `Ts → 0`.

### F10.a Filter equations (Kalman-Bucy)
For the **continuous** F2 model `ẋ = A·x + B·u + G·w`, `y = C·x + v` with white
`w ~ (0, Qc)`, `v ~ (0, Rc)` (intensities, [unit²·s]), uncorrelated:
```
gain:        K(t) = P(t)·Cᵀ·Rc⁻¹
estimator:   x̂̇ = A·x̂ + B·u + K(t)·( y − C·x̂ )
covariance:  Ṗ = A·P + P·Aᵀ − K·Rc·Kᵀ + G·Qc·Gᵀ
                = A·P + P·Aᵀ − P·Cᵀ·Rc⁻¹·C·P + G·Qc·Gᵀ      (substitute K = P·Cᵀ·Rc⁻¹)
```
- The two `Ṗ` forms are identical: `K·Rc·Kᵀ = (P·Cᵀ·Rc⁻¹)·Rc·(Rc⁻¹·C·P) = P·Cᵀ·Rc⁻¹·C·P`.
- **Units:** `K` [state·(meas·s)⁻¹] (continuous gain, no `Ts`); `P` [state²]; `Qc, Rc` are
  spectral intensities (per-Hz / ×s), **not** the discrete covariances `Q, R` of F6 — they
  relate by `Q ≈ G·Qc·Gᵀ·Ts`, `R ≈ Rc/Ts` for small `Ts` (the bridge to F6).
- There is **no separate predict/correct split**: a single matrix ODE for `(x̂, P)` is integrated
  forward in time (contrast F6's two-stage discrete recursion).

### F10.b Steady-state limit → CARE (ties to F7)
For LTI `(A, C)` with constant `Qc, Rc` and the pair observable (F3 ✔), `P(t) → P∞` as the
transient dies, so `Ṗ → 0` and the differential Riccati equation collapses to the **continuous
algebraic Riccati equation (CARE)**:
```
A·P∞ + P∞·Aᵀ − P∞·Cᵀ·Rc⁻¹·C·P∞ + G·Qc·Gᵀ = 0
L  =  P∞·Cᵀ·Rc⁻¹          (constant Kalman-Bucy gain)
```
This is **exactly the CARE already stated in F7.a** (`A·P + P·Aᵀ − P·Cᵀ·R⁻¹·C·P + G·Q·Gᵀ = 0`,
`L = P·Cᵀ·R⁻¹`) — declared there **by pointer**; MATLAB `lqe(A,G,C,Qc,Rc)` returns this `L`.
So F10 is the *time-varying transient* whose steady state is F7's continuous constant-gain
observer; F7 ⊂ F10 (steady-state slice).

### F10.c When continuous vs discrete is the right choice
- **Use F10 (continuous)** when: the design/analysis is in continuous time (e.g. `lqe`-based
  gain for an analog or very-high-rate loop), or when deriving the transient covariance flow,
  or when `Ts` is so small that the discrete recursion is just approximating the ODE.
- **Use F6/F7 (discrete)** for the *implementable* digital estimator (the actual build runs at a
  fixed `Ts` on a processor) — F5.a/F6 are the sampled realisation. The continuous gain `L`
  from F10.b can seed the discrete design (discretise, or map to `Ld`), but the running filter
  is discrete.
- **Bridge:** as `Ts → 0`, the discrete F6 recursion converges to the F10 ODE (with the `Q,R`↔`Qc,Rc`
  scaling above); F7's DARE solution → F10.b's CARE solution. The build exposes the *discrete*
  estimator; F10 is the continuous reference it approximates.

### F10 audit
- **Role:** continuous-time optimal estimator and the theoretical limit of F6/F7; supplies the
  `lqe`/CARE gain and the covariance-flow ODE; `T̂L` (continuous) → F8.
- **Plant dependence:** **F2** continuous `(A,B,C,G)` directly (no discretisation); steady-state
  **= F7.a CARE by pointer** (not re-derived).
- **Sources (≥2 independent):**
  - **S10 — Kalman & Bucy 1961**, "New Results in Linear Filtering and Prediction Theory,"
    *J. Basic Eng.* 83(1):95–108, DOI <https://doi.org/10.1115/1.3658902> — the foundational
    paper deriving the Riccati "variance equation" and the continuous optimal gain. DOI resolves
    (ASME Digital Collection). The 1961 original's equations could not be transcribed verbatim (scanned
    PDF, corrupt text layer); the gain `K=P Cᵀ Rc⁻¹` and `Ṗ = AP+PAᵀ−PCᵀRc⁻¹CP+GQcGᵀ` are
    reproduced identically by S11 and the corroborating notes below.
  - **S11 — Anderson & Moore, *Optimal Filtering*** (ISBN 978-0-486-43938-9, Dover 2005 reprint
    of Prentice-Hall 1979) — continuous-time (Kalman-Bucy) filter, gain and differential Riccati
    equation, steady-state CARE. ISBN verified.
  - **S8 — Gelb (ed.), *Applied Optimal Estimation*** (ISBN 978-0-262-57048-0) — Ch. 3,
    continuous Kalman-Bucy filter and the matrix Riccati equation. ISBN verified.
  - **Corroboration (non-anchor):** Goddard Consulting "Kalman-Bucy Filter" and Stengel
    (Princeton MAE 546) lecture notes both print `Ṗ = FP+PFᵀ−PHᵀR⁻¹HP+GQGᵀ` — identical form.
- **Confidence:** HIGH for the Kalman-Bucy form and the F7-CARE equivalence (canonical, multiply
  corroborated). Flag (LOW): 1961 paper's *verbatim* equation could not be transcribed from the scan
  (publisher/scan limitation, not fabrication) — form confirmed by ≥3 independent reproductions.

---

## Summary table (formula → status → primary anchor)

| F | Formula | New? | Primary source(s) | Confidence |
|---|---|---|---|---|
| F1 | dq current-loop PI (IMC `Kp=α_c·L, Ki=α_c·R`) + decoupling FF | ➕ | S1 (DOI 10.1109/28.658735), S5 | HIGH |
| F2 | augmented `[wm; TL]` state-space, `ṪL≈0` | ➕ | S2 (10.1109/28.85485), S3 (10.1080/00051144.2024.2354643) | HIGH |
| F3 | observability `O=[C;CA]`, `det=−1/J≠0` | ➕ | S5, S3 | HIGH |
| F4 | continuous Luenberger `x̂̇=(A−LC)x̂+Bu+Ly`, pole placement | ➕ | S5, S2, S4 (10.3390/en18040883) | HIGH |
| F5 | discrete Luenberger (Euler/ZOH/Tustin) | ➕ | S5, S6 | HIGH |
| F6 | discrete recursive KF (predict/update, Joseph) | ➕ | S6 (ISBN 978-0-471-70858-2), S3 | HIGH |
| F7 | steady-state KF (DARE, `lqe`-style `K∞`) ↔ Luenberger | ➕ | S6, S5 | HIGH (MED on MATLAB form pin) |
| F8 | feed-forward `Te_ref=Te_fb+T̂L`, DOB intuition | ➕ | S2, S4 | HIGH (MED on exact sensitivity) |
| F9.a | correlated-noise KF, gain `K=(P⁻Hᵀ+M)(HP⁻Hᵀ+HM+MᵀHᵀ+R)⁻¹` | ➕ | S6 (Ch.7), S7 (ISBN 978-0-470-60969-9), S9 (10.5772/intechopen.1005079) | HIGH (MED on MATLAB `N`-arg) |
| F9.b | colored-noise KF via Gauss-Markov shaping state; `a→0`→F2 | ➕ | S6 (Ch.7), S7, S8 (ISBN 978-0-262-57048-0) | HIGH |
| F10 | continuous Kalman-Bucy `K=PCᵀRc⁻¹`, `Ṗ=AP+PAᵀ−PCᵀRc⁻¹CP+GQcGᵀ`; CARE = F7 | ➕ | S10 (10.1115/1.3658902), S11 (ISBN 978-0-486-43938-9), S8 | HIGH (LOW on 1961 verbatim) |

**Low-confidence / implementation-time flags:** (1) F7 `dlqe`-vs-`lqe` current/prediction-form convention
to pin at build time; (2) F8 `(1−Ĝ)` sensitivity reduction is intuition, not an exact bound
for this plant; (3) F9.a exact `P⁺` form + MATLAB `kalman`/`dlqe` `N` cross-term sign to pin at
implementation-time; (4) F9.b `G_aug` port (case A) and `R→0` conditioning (case B) are implementation-time choices;
(5) F10 — Kalman & Bucy 1961 *verbatim* equation could not be transcribed from the scanned original
(form confirmed by ≥3 independent reproductions: Anderson & Moore, Gelb, Goddard/Stengel).
**No reference-specific numeric values or filenames appear in this document.**
