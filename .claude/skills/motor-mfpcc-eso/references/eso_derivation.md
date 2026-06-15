# ESO Derivation + Tuning (standalone)

Standalone derivation of the linear Extended State Observer used by MFPCC-ESO, and the deadbeat
voltage solve built on it. Equation numbers track [pmsm_formulas.md §G](../../../../shared/formulas/pmsm_formulas.md), which
in turn tracks the source paper (Zhang 2021, *IEEE TIE* 68(2), §III–§IV). Read this when you need the
*why* behind the gains; the formulas doc remains the single source of truth for the equations.

## 1. Ultralocal model — what "model-free" means

The SPMSM αβ current dynamics (pmsm_formulas §8):

```
di_s/dt = (1/Ls)·u_s + (−Rs·i_s − jωe·ψr)/Ls
```

are matched to the first-order **ultralocal model** (eq 4 / eq 7):

```
di_s/dt = F + α·u_s
```

- `α` = input gain. Nominally `1/Ls`, but treated as a **free design constant** (paper: α = 50 for
  all sampling rates — it is NOT identified from Ls). The α-vs-1/Ls mismatch is absorbed into `F`.
- `F` = lumped disturbance. Swallows resistance drop, back-EMF, and ALL parameter error — anything
  the model doesn't know. This is why the control law needs no machine parameters.

The controller's only job: estimate `F` fast enough, then cancel it.

## 2. Continuous-time linear ESO

Take states `z1 = î_s` (current estimate) and `z2 = F̂` (disturbance estimate), driven by the
estimation error `ε = z1 − i_s` (eq 8):

```
ż1 = z2 + α·u_s − β1·ε
ż2 =            − β2·ε
```

Error dynamics characteristic polynomial (eq 10): `s² + β1·s + β2`.
**Bandwidth parameterization** (Gao): place both poles at `−ω0` (eq 11/12):

```
β1 = 2·ω0,    β2 = ω0²
```

One knob (`ω0`) sets the whole observer. Larger ω0 → faster `F̂` convergence → better disturbance
cancellation, but more noise amplification and less discrete-time robustness.

## 3. Discrete ESO + the z12 tuning rule

Forward-Euler discretization at the control period `Tsc` (eq 13), with discrete gains
`β01 = Tsc·β1`, `β02 = Tsc·β2`:

```
ε(k)     = î_s(k) − i_s(k)
î_s(k+1) = î_s(k) + Tsc·F̂(k) + Tsc·α·u_s(k) − β01·ε(k)
F̂(k+1)  = F̂(k)                              − β02·ε(k)
```

The discrete characteristic equation (eq 15) `z² + (β01−2)·z + (1−β01+β02·Tsc) = 0` has a
**double pole** (eq 16) when the bandwidth parameterization is substituted:

```
z1,2 = 1 − ω0·Tsc      ⇔      ω0 = (1 − z12)/Tsc        (eq 17)
```

**Tuning rule**: pick the double pole `z12 ∈ (0,1)`:
- `z12 → 1` (small ω0): sluggish observer, poor disturbance rejection.
- `z12 → 0` (large ω0): fast but fragile — noise amplification, divergence risk.
- Paper default: **`z12 = 0.15`** (fast current loop, validated).

Closed-form chain used by the build (assert all three):

```
omega0 = (1 − z12)/Tsc;   beta01 = 2·omega0·Tsc;   beta02 = omega0²·Tsc
```

Numeric anchor (validation case, `Tsc = 1e-4`, `z12 = 0.15`):
`omega0 = 8500`, `beta01 = 1.7`, `beta02 = 7225`.

## 4. Deadbeat voltage solve + delay compensation

Invert the ultralocal model for the voltage, replacing `F` by `F̂` (eq 18) and forcing
`i_s(k+1) = i_s,ref` (eq 19 — deadbeat). In a real digital loop the computed voltage is applied one
period late, so the paper compensates one step ahead (eq 20):

```
u_s,ref = (1/α)·[ (i_s,ref(k+2) − î_s(k+1))/Tsc − F̂(k+1) ]
```

using the ESO's (k+1) predictions, and rotates the current reference forward two steps (eq 21):

```
i_s,ref(k+2) = i_s,ref(k)·e^{j·2·ωe·Tsc}
```

The 1-step delay being compensated is REAL in the build: it is the output Unit Delay between the
chart and SVPWM (chart_contract.md §3). Remove or ZOH-ify that delay and eq (20) compensates a
delay that no longer exists — the loop diverges.

`u_s,ref` is a **continuous** voltage vector handed to SVPWM — this is where the method's identity
(deadbeat + SVM, NOT FCS) is decided. No candidate vectors, no cost function.

## 5. Model-based cross-checks (for acceptance, not for control)

The controller never uses these, but the *verifier* should:

- **F̂ steady-state magnitude** ≈ back-EMF term `ωe·ψf/Ls` (the Rs·i term is small).
  Validation case @ 1000 rpm: `ωe = 104.72×4 = 418.9 rad/s` → `418.9×0.402/0.01936 ≈ 8700 A/s`.
  Logged `|F̂| = sqrt(Fa² + Fb²)` should match within ~10%.
- **ESO digital pole**: from logged step responses the observer error should settle like
  `(1 − ω0·Tsc)^k` — no ringing (double real pole).
- **id residual bias**: grows ∝ ωe² (finite-bandwidth tracking of the ROTATING `F`) — expected,
  bounded, documented in known_limitations.md #1. Do not tune it away during acceptance.
