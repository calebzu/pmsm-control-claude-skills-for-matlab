# L2 contract — reaching true top-level zero crossings (opt-in)

L1 only moves blocks and re-routes lines, so on a **non-planar** graph it cannot reach zero line-line crossings. L2 removes the offending edges from the *top-level drawing* without changing the model's behavior, by two logically-equivalent transformations. L2 is **opt-in** and the user must explicitly request it.

## What L2 is allowed to do

1. **Goto/From substitution.** Replace a long-range or multi-consumer signal line with a `Goto` block at the source and `From` blocks at each consumer (matching tag). The signal is identical; the visible wire disappears, so it can no longer cross anything.
2. **Subsystem encapsulation.** Wrap a functional cluster of blocks into a `Subsystem`. Internal crossings move inside the subsystem; the top level sees one tidy block. Behavior is unchanged.

Both change the **block set** (Goto/From/Subsystem are new blocks), which is why L2 needs a re-test that L1 does not.

## Mandatory L2 re-test

After any L2 transformation you MUST:

1. Run one smoke simulation (a short `sim(mdl)` or existing test harness).
2. Confirm no `NaN`/`Inf` in the outputs.
3. Confirm behavior is unchanged versus the pre-L2 baseline (compare a key logged signal, not just "it ran").

*Why:* unlike L1's byte-identical guarantee, L2 edits the block set — a mistyped Goto tag or a mis-scoped `TagVisibility` silently breaks the signal. The smoke sim is the safety net.

## Goto/From pitfalls

- **TagVisibility.** Default is `local`. For a From to see a Goto across subsystem boundaries it must be `global` (or `scoped` with matching scope). A default-local tag that crosses a boundary yields an unconnected From → silent break. This is the single most common L2 failure.
- **Tag uniqueness.** A global tag name must be unique model-wide; a collision cross-wires unrelated signals.
- **Readability cost.** Goto/From trades visible wires for invisible coupling. Use it for genuinely long/many-consumer signals, not everywhere — a model that is all From blocks is *less* readable, not more.

## When L2 is worth it

- The planarity check proves non-planar AND the user wants a publication-grade, crossing-free top-level figure.
- A few signals (clocks, enables, a shared bus) fan out to many consumers and dominate the crossing count — ideal Goto/From candidates.
- A self-contained cluster (e.g. a controller, an observer) can be boxed without splitting tightly-coupled logic — ideal Subsystem candidate.

If none of these hold, stay at L1 and report the honest crossing floor.
