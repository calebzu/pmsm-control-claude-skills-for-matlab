# Known Limitations — characterized, not bugs

## 1. `id` steady-state bias grows ∝ ωe² (ESO finite-bandwidth vs rotating disturbance)

**Symptom**: a small positive `id` offset that grows quadratically with speed.
Validation plant: 0.034 / 0.133 / 0.297 A at 500 / 1000 / 1500 rpm (`id/ωe²` constant within ~1.3%);
at the 1500 rpm rated point ≈ 7% of rated current.

**Mechanism**: the lumped disturbance `F` contains the back-EMF, a vector **rotating** at ωe. A
finite-bandwidth ESO tracks a rotating target with amplitude error ∝ ωe and phase lag ∝ ωe — the
product shows up as a residual ∝ ωe², landing on the d-axis. Verified by exclusion: NOT an α
mismatch (quadratic fit has ≈ 0 constant term), NOT the angle extrapolation (linear term negligible).

**Disposition**: expected physics of the linear-ESO design — accept within the acceptance band.
Mitigations, with costs:
- raise `ω0` (lower `z12`): shrinks the bias, but amplifies noise and erodes discrete robustness
  (eso_derivation.md §3);
- feed forward the back-EMF: removes the bias but requires `ψf`/`ωe` in the law — **breaks the
  model-free identity**. Out of v1 scope.

## 2. One-step output latency is structural (Unit Delay, not removable)

The deadbeat law compensates a 1-step digital delay that is physically present as the output Unit
Delay (chart_contract.md §3). Consequences: (a) the applied voltage always lags the computation by
one `Tsc`; (b) "optimizing" the delay away — or swapping in a ZOH — breaks the compensated horizon
and diverges. The latency is part of the method, not an implementation artifact.

## 3. SPMSM only

The αβ ultralocal premise rides on `Ld = Lq = Ls` (pmsm_formulas §8). Saliency (`Ld ≠ Lq`)
introduces 2θe-dependent terms that the single constant `α` cannot represent; the rotating-frame
content of `F` grows and limitation #1 worsens. IPMSM is out of scope — do not parameterize around it.

## 4. `J`, `B`, speed-loop PI are NOT from the paper

The paper tabulates the electrical plant only. `J = 3e-3`, `B = 1e-3`, `Kp_w = 0.6`, `Ki_w = 8` are
supplied values for the validation case. The current loop is insensitive to them (model-free), but
speed-loop transients (dip %, settle time) are NOT paper-traceable — re-tune per machine.

## 5. Parameter-robustness claims are paper evidence, not a re-verified envelope

The robustness sweeps (inductance 50–200%, flux 50–200%, Rs 10–1000%) are the paper's experiments
(its Figs. 4/5/7/12–14). This skill documents them as source claims; the validated envelope here is
the nominal validation case only. Re-verify before relying on extreme mismatch operation.
