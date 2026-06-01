# Acceptance gates — tiered by reachability

The cardinal rule: **gate only what is always achievable; report (don't gate) what topology may forbid.** Setting "zero crossings" as a hard gate would reject mathematically-valid models and is forbidden.

| Criterion | Gate type | Rationale |
|---|---|---|
| block-block overlaps `== 0` | **hard assert** | Overlap is always removable by moving blocks. `tidy_layout.m` asserts this and a residual de-overlap pass guarantees it. |
| line-through-block hits `== 0` (or tiny threshold) | **hard assert** | A line cutting through an unrelated block is always re-routable. Keep a small threshold only for pathological dense models. |
| line-line crossings | **soft report + soft gate** | Planar graph → expect ≈ 0. Non-planar graph → only require `≤ L1 floor` (no worse than the achievable minimum) **or** resolved via L2. Never require absolute zero. |
| compactness / extent | report only | Compactness trades off against readability; show before→after, do not gate. |
| screenshot "not ugly" | **human-in-the-loop** | The final gate is a human eyeballing the exported PNG. No metric substitutes for it. |

## How to apply the crossing gate honestly

1. Run `planarity_check.py` on the extracted graph.
2. **Planar** → it is legitimate to push for crossings near 0. If L1 leaves crossings, that is a routing-quality issue, not a topological wall; report the residual and the screenshot.
3. **Non-planar** (K3,3/K5 proven) → state plainly: *zero crossings is impossible by block moves alone* (Kuratowski). The L1 success criterion becomes "crossings not worse than before, overlaps and line-block hits at zero, screenshot acceptable." Offer L2 if the user wants true top-level zero.
4. **Inconclusive** (Euler fallback, no networkx) → say so; recommend `pip install networkx` before making any zero-crossing claim.

## No-regression invariant + idempotency

`tidy_layout` measures the arrange cost on a throwaway copy and keeps the minimal-move layout (original routing + just-enough de-overlap) whenever arrange is strictly worse on crossings. So **the reported `after` crossings never exceed `before`** (`selftest.m` asserts `after ≤ before`), and re-running keeps overlaps at 0. arrange is applied to the real model only when it genuinely wins, so crossings do not fluctuate run-to-run.

## Anti-pattern (red line)

> ❌ "Layout tidied — **0 crossings achieved**." on a model whose graph contains a K3,3.

This is either false or was bought by silently deleting a connection. Never claim or hard-require zero crossings without a planarity proof that the graph is planar.
