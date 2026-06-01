# Reference-Model Skill Distillation: A Methodology

> **Version**: 1.1 (layout-readability gate added; initial public release 1.0, 2026-05-18)
> **Author**: Zong Chuhang <ZONG0008@e.ntu.edu.sg>
> **License**: Apache 2.0
>
> **Purpose**: When you have a reference model — your own baseline, a peer's published implementation, or an open-source repository — and want to (a) learn the methodology behind it and (b) distill that knowledge into a reusable Claude Code skill, this workflow tells you how. **Its central novelty is a verification protocol that prevents the rebuild subagent from silently reproducing the reference's structure instead of independently deriving it** — see Appendix F.
>
> **Validation**: Applied to three independent control families (one-step finite-control-set MPC, hysteresis-table direct torque control, super-twisting sliding mode) targeting surface-mounted and interior-permanent-magnet synchronous machines. Appendix C gives anonymized **narrative** case studies (problem / diagnosis / resolution). Raw simulation artifacts (build scripts, `.slx`, waveforms, logs) live in the author's private development repository and are **not** included in this public release — see "Reproducibility & Evidence" in the [README](README.md).

---

## TL;DR

Build a skill from a reference model in **eleven phases** (Phase 0 through Phase 10) under **seven non-negotiable disciplines**:

1. **Learn–build separation** — understanding a model is not the same as being able to rebuild it; verify both abilities separately.
2. **Anti-cheating** — during the rebuild phase, the agent must not read the reference model or any document derived from it. This is enforced operationally via subagent isolation (Phase 5) and reference physical relocation (Appendix F §9).
3. **Two-round validation** — round one reproduces (you learned it); round two generalizes (the skill is reusable).
4. **Confidentiality boundary** — methodology and reusable templates travel into the skill; specific numerical parameters and reference filenames do not.
5. **Failure exit** — at most two reverse-corrections before archival. A skill that needs a third revision is signaling the methodology needs revising, not the skill.
6. **Anti-contamination** ⭐ — fresh verification subagents require an explicit must-NOT-read list (three layers: confidential / historical / meta) plus a main-session audit protocol (grep / mathematical consistency / structural consistency). This is the core contribution. See Appendix F.
7. **Theory-first priors** — derive plant and controller equations before opening the reference `.slx`. The reference must not seed the main session's mental model; theory pre-derivation also unlocks a bug-detection capability (a mismatch between equations and implementation is a *finding*, not a hallucination).

The phase-by-phase workflow is in §"Phases (0–10)"; the seven disciplines are detailed in §"Core Principles"; the anti-contamination SOP is in Appendix F; failure budgets and role-transition protocols are in §"Refinement Loop on FAIL".

---

## Core Principles (Seven Non-Negotiable Disciplines)

| # | Discipline | Why |
|---|---|---|
| 1 | **Learn–build separation** | Understanding a model is not the same as being able to rebuild it. Both abilities must be verified independently — one alone yields a fragile skill. |
| 2 | **Anti-cheating** | If the agent peeks at the reference during rebuild, the rebuild degenerates into module-by-module copying. The resulting skill captures syntax, not methodology. |
| 3 | **Two-round validation** | First round: can you reproduce the reference's behavior (you learned it). Second round: can the distilled skill solve a new problem in the same domain (the skill generalizes). |
| 4 | **Confidentiality boundary** | The reference's intellectual property (specific parameter values, filenames, identifying topology) stays out of the skill. Only methodology and parameterized templates travel. |
| 5 | **Failure exit** | At most two reverse-corrections. If the methodology cannot drive a passing rebuild within that budget, archive the attempt — do not force a third revision pretending the methodology held. |
| 6 | **Anti-contamination** ⭐ | Fresh verification subagents require an explicit must-NOT-read list (three layers: confidential / historical / meta) plus a main-session audit protocol (grep / mathematical consistency / structural consistency). Without this, the subagent can "cheat" via documentation osmosis, magic-number reproduction, or failure-path memorization. **Core contribution — see Appendix F.** |
| 7 | **Theory-first priors** | Derive the plant equations and controller fundamentals *before* opening the reference `.slx`. The reference must not seed the main session's mental model. A bonus: a mismatch between pre-derived equations and the reference's implementation becomes a **finding** (the reference may be buggy, unconventional, or use a higher-order formulation) rather than a model hallucination. |

---

## Three-Tier Shareability Classification

A binary "methodology shareable / implementation confidential" split is too coarse. In practice, the rebuild subagent needs **platform-level API facts** that are not reference-specific (e.g., "the R2024b SimPowerSystems library path is `sps_lib/...`") but also not pure methodology. Without these, the rebuild fails at the physical layer despite a sound methodology document.

Three tiers:

| Tier | Content | Provided to rebuild subagent? | Example |
|---|---|---|---|
| 🟢 **Methodology** | Minimal skeleton, conventions, enhancement checklist, common-error checklist, design decisions | ✅ | "Cost function: `λ_d · e_d² + λ_q · e_q²`"; "Use amplitude-invariant Park transform"; "Prediction horizon N=1, forward-Euler discretization" |
| 🟡 **Platform API know-how** | R2024b platform facts independent of any specific reference | ✅ | "SimPowerSystems library path"; "Universal Bridge mask field is `Arms`"; "Stateflow chart type `Stateflow.EMChart`" |
| 🔴 **Reference-specific** | Parameter values, filenames, fully-wired subsystems, identifying topology | ❌ | `Rs = 0.9585 Ω`; `<reference_model>.slx`; complete plant wiring diagram |

**Separation rule**: Tier 🟡 captures what R2024b literally requires; Tier 🟢 captures what *good* design chooses. The distinction matters at rebuild time: a subagent with 🟢 alone may know to apply Park transformation but cannot construct the block because it doesn't know the R2024b mask field names.

**Border cases**:
- `Universal Bridge mask 'Arms' = '3'` → 🟡 (the field exists in R2024b); `DC Voltage Source Amplitude = 300` → 🔴 (specific numerical value).
- A Clark subsystem (formula + amplitude-invariant convention) → 🟢 / 🟡 hybrid (you can share the building block); the Clark → Park → controller wiring path → 🔴 (topology encodes design).

---

## Workflow Parameters

Each application of the workflow fills the following parameters:

| Parameter | Meaning |
|---|---|
| `{reference_model}` | The reference model file being studied |
| `{domain}` | Domain (e.g., motor type + control method family) |
| `{target_skill_name}` | Name of the skill being distilled |
| `{generalization_test}` | A novel problem within the same domain, **specified during Phase 1** before any rebuild has been attempted, used by Phase 8. Specifying late risks reverse-engineering the test to be easy. |
| `{acceptance_criteria}` | Per-signal numerical equivalence thresholds (see Equivalence Grades below) |

---

## Equivalence Grades

Three nested grades, used at different phases:

| Grade | Definition | Where it gates |
|---|---|---|
| **Structural** | Identical block count, wiring topology, subsystem hierarchy | **Never gates** — implementation freedom is desirable; deviations are not bugs |
| **Behavioral** | Under the same scenario, the primary waveforms (e.g., speed, torque, currents) agree pointwise within ε or overlay hit rate ≥ 95% | **Phase 6 gates here** |
| **Robust** | Under perturbations (load step, ±10% parameter variation, measurement noise), error metrics remain within ±20% of reference | **Phase 8 gates here** |

Per-signal thresholds (ε) are domain-specific. For PMSM speed control as an example: `ω_m ε = 1 rad/s`, `i_q / i_d ε = 0.5 A`, `T_e ε = 0.2 N·m`, `i_abc ε = 2 A`. Pick thresholds that reflect the measurement noise floor and the application's tolerance — not the precision of double-precision floats.

---

## Phases (0 – 10)

### Phase 0 — Acquiring the Reference Model

- The user obtains the reference model (own work, supervisor, paper supplement, or cloned repository).
- Declare the IP class up front:
  - **Proprietary** — must not be transmitted outside the local environment.
  - **Licensed** — respect license terms (citation, redistribution, derivative use).
  - **Open** — public reuse permitted.

The IP class determines which artifacts may live in the skill (methodology only) versus which must stay outside it (filenames, identifying topology, specific numerical parameters).

---

### Phase 1 — Kickoff

**Owner**: user.

**Actions**:
1. Place the model under `references/` or note its path.
2. **Specify the `{generalization_test}`** now. Specifying it after the rebuild has been attempted invites reverse-engineering the test to make it easy.
3. Declare `{acceptance_criteria}` (defaults from §"Equivalence Grades" or custom thresholds).

**Output**: A bookmark in session memory recording Phase 1 completion and the parameter table.

---

### Phase 1.5 — Theory Pre-Derivation ⭐

**Owner**: Claude (main session, leading) + user review.
**Location**: Main session — critically, **before opening any `.slx`**. The main session is in a clean state, free of reference-specific implementation context.

**Why this phase exists**: A naive workflow reads the reference first, then back-derives theory. Under that ordering the main session's first artifact (a deep-dive document) is already contaminated with reference-specific choices, and contamination does not unwind. Phase 1.5 reverses the order: theory first, verify the model against it. Two benefits:

1. The main session accumulates a shareable theoretical anchor *before* contamination.
2. A mismatch between pre-derived equations and the reference's implementation becomes a **finding**, not a hallucination — the reference may have a bug, an unconventional choice, or a higher-order formulation the pre-derivation missed.

**Trigger**: First-time application of a new method (e.g., a control-law family the user has not previously distilled). If the method's core equations are already complete in `shared/formulas/`, skip to the coverage check; otherwise add incrementally.

**Procedure**:

1. **Coverage check.** List the equations the method requires (plant dynamics, control-law fundamentals, observers / estimators, switching / decision logic). Compare against existing `shared/formulas/`. Diff: ✅ already present, ➕ to add, ⊘ not needed.

2. **Independent research (literature only, no implementation).**
   - Must **not** read any file in `references/<method>/` (vendor slides, tutorial packs, `.slx`, accompanying `.png`) or any reference `.slx` under `shared/reference_models/`.
   - Collect ≥ 3 academic primary sources per formula. Primary sources only — vendor tutorials may misrepresent textbook conventions to fit their packaging. DOI and BibTeX must be complete and verifiable.

3. **Write the shareable formulas document.**
   - Path: `shared/formulas/<plant>_<method>_formulas.md` (new file) or appended section to `<plant>_formulas.md`.
   - Header: `SHAREABLE — Phase 1.5 artifact, usable by Phase C rebuild subagent`.
   - Each formula records: definition + units + sign and amplitude convention + primary source (academic, *not* the reference model) + role in the method.
   - Mirror academic anchors in `methods/<m>/reference_study/literature_anchors.md` (DOI / BibTeX).

4. **User review.** Sign off on each formula one by one — correctness, coverage, source authority.

**Output**:
- `shared/formulas/<plant>_<method>_formulas.md` (or appended section)
- `methods/<m>/reference_study/literature_anchors.md`

**Confidentiality**: `SHAREABLE — Phase 1.5 artifact, usable by Phase C rebuild subagent`.

**Acceptance — five hard checks**:

- [ ] Coverage diff complete (existing vs to-add vs not-needed).
- [ ] New formulas contain no reference filename and no reference-specific numerical value (`grep` returns zero matches).
- [ ] Academic reference list has ≥ 3 sources with verifiable DOI / BibTeX.
- [ ] **Formula-by-formula sign-off.** One formula → user approves → written → next. Batch submission of multiple formulas for review is forbidden.
- [ ] **Each formula carries four anti-fabrication anchors**: (i) DOI resolvable from a browser, (ii) the abstract's first sentence quoted verbatim, (iii) exact page and equation number (e.g., `p. 247, Eq. (3)`), (iv) declared dependence on already-signed formulas (which assumptions are inherited).

**Why anchors 4 and 5**: LLM-generated formula collections fail in two common modes — (a) batch submission overwhelms the reviewer until subtle errors slip through, and (b) plausible-looking but fabricated DOIs, authors, or journals enter the record. Per-formula sign-off addresses (a); the four anchors address (b) by making any single anchor independently spot-checkable by the user.

**Contamination guarantee (hard rule)**: During Phase 1.5, the main session must not read any subdirectory under `methods/<m>/reference_study/` (with the single exception of `literature_anchors.md` created *within* Phase 1.5) and must not read any reference `.slx`. Violation invalidates Phase 1.5 — it must be redone in a fresh main session, because contamination does not wash out.

#### Sub-condition: Signing Authority by Domain Expertise

**Trigger**: The user lacks domain expertise to review formulas independently in this method — for example, a motor-control practitioner reviewing sliding-mode formulas without prior SMC background, or a PMSM specialist reviewing induction-machine formulas.

**Switch**: From the default mode (user has domain expertise) to **AI-self-audit with verifiable sources**.

| Default mode | AI-self-audit mode |
|---|---|
| User signs each formula | AI signs each formula with ≥ 2 independent cross-check sources |
| Four anchors (DOI / abstract / page / dependence) spot-checked by user | Same four anchors, plus a hard requirement: ≥ 1 resolvable DOI link *and* ≥ 1 PDF cached locally |
| Optional post-Phase-6 reference-comparison sanity if the reference is unlocked | The optional reference comparison + Phase 8 Gate 4 user visual review act as a **fail-safe** substitute for formula sign-off |

**Mode declaration is explicit.** At Phase 1.5 kickoff, the main session asks: "Do you have domain expertise to review formulas in this method independently?" If the user answers no, switch to AI-self-audit mode, record the switch in session memory, and prepend each formula's header with `AI-self-audit with verifiable sources, user domain knowledge gap declared YYYY-MM-DD`.

**Anti-pattern**: A user signing formulas in a domain they don't understand creates a false sense of security — the user has no capacity to catch AI hallucinations in formulas, DOIs, or citations. The reviewer-pattern fails silently.

**Validated case**: In one Phase 1.5 application, the user declared no prior background in the target method. The workflow switched to AI-self-audit mode: each of the six method-specific formulas was anchored by ≥ 2 independent academic sources (one peer-reviewed paper + one textbook); all DOIs resolvable; all PDFs cached locally to defend against fabricated references. Phase 8 user visual review served as the fail-safe and confirmed the resulting skill produced correct tracking behavior, demonstrating that AI-self-audit mode preserves the strictness of the verification gate. See Appendix C, Case 3.

**Relation to Phase 4.5**:

| Phase | Layer | Output |
|---|---|---|
| **Phase 1.5** | Plant-level and control-law-level formulas (open-loop static and transient equations) | Plant equations + control-law fundamentals |
| **Phase 4.5** | Closed-loop stability + time constants (must build on Phase 1.5) | Transfer functions + poles + slowest time constant |

Phase 1.5 is a hard prerequisite for Phase 4.5 — without plant equations the closed-loop transfer function cannot be derived.

---

### Phase 2 — Deep Read of the Reference (Phase A)

**Owner**: Claude.
**Location**: Main session (full context available — this is the last opportunity to read the reference before the rebuild lockdown of Phase 5).

**Actions**:

1. **Top-level signal-flow diagram (textual).** For every signal line, record: name / units / sample rate / physical meaning / source block / destination block. Make the loop boundaries explicit — outer loop, inner loop, plant, sensors.

2. **Block enumeration.** For every block, record: Simulink path / `BlockType` / mask parameters (every field) / functional role / whether reference-specific (🟢 / 🟡 / 🔴 per the Three-Tier Shareability Classification).

3. **Algorithm core, transcribed in full.** MATLAB Function blocks and Stateflow charts: copy verbatim with inline comments documenting the physical / mathematical meaning of every intermediate variable and its units.

4. **InitFcn variable traceback.** For each variable defined in `InitFcn` or workspace: list every block × mask field consuming it. Flag dead variables.

5. **Solver and sample-rate audit.** Main solver configuration, `powergui` `Ts`, every Zero-Order Hold, every Discrete block, every Rate Transition.

6. **Scenario and test inputs — complete record.** Read the entire time–value sequence of every Timer / Step / Signal Builder, not just the first row of the mask. A common trap: a Timer feeding a Rate Limiter produces ramp behavior that the Timer's first mask row does not reveal.

7. **Note-taking organized by learning granularity** — anti-cheating discipline #2 operationalized as four categories of what to learn versus what to discard:

   | Category | What to learn | Goes into the skill? |
   |---|---|---|
   | ① **Plant parameters** | `Pn / Rs / Ld / Lq / ψ_f / J / B` values + machine type (SPM / IPM / salient / surface) | ❌ Numerical values are reference-specific (🔴). ✅ "Type + order-of-magnitude range" goes into the skill as a `Required Input` annotation |
   | ② **Wiring logic** | Top-level signal-flow diagram + subsystem port connections + feedback paths | ✅ The skill's `build_template.m` embodies the topology (not the on-screen layout) |
   | ③ **Algorithm implementation** | S-function / lookup table / inline algorithm | ✅ The skill reimplements with a modern equivalent — typically a MATLAB Function block or Stateflow chart — capturing the algorithm's essence rather than copying the legacy syntax. This is not a violation of anti-cheating discipline; it is precisely what skill distillation is for |
   | ④ **Generic blocks** | Clark, Park, PMSM block, hysteresis, etc. — atomic Simulink / Simscape blocks | ✅ Check `shared/building_blocks/` for reuse; add any new generic block to the library (🟡 SHAREABLE) |

   **Key insight**: A vintage reference may use APIs from much earlier MATLAB releases (e.g., S-functions where MATLAB Function blocks would now be idiomatic). The rebuild should use **R2024b-current equivalents** — this removes implementation-specific carryover that does not belong in a generalized skill.

8. **Formula compatibility audit** — verify each formula from Phase 1.5 against the reference's implementation:
   - **Match** → ✅ marked `verified`.
   - **Mismatch** → ⚠️ this is a **finding**, not a hallucination. Four resolutions:
     - The Phase 1.5 derivation has an error → revise the formulas document (rare; should have been caught at Phase 1.5 user sign-off).
     - The reference has a bug or unconventional choice → record in the deep-dive under a "Non-Standard Implementations" section; cite as evidence of reference-specific quirks when writing about the methodology.
     - The two operate under different assumptions → annotate the caveat in both sources.
     - The reference uses a higher-order formula the Phase 1.5 pass missed → return to Phase 1.5 and add incrementally.
   - **Formula present but unused by the reference** → mark as `over-derivation` (do not delete; leaves headroom for the skill design).
   - **Implementation present but no formula derived in Phase 1.5** → mark as `missed pre-derivation`; return to Phase 1.5 and add.

**Outputs**:
- `methods/<method>/reference_study/<reference>_deep_dive.md` (🔴 CONFIDENTIAL)
- `methods/<method>/reference_study/<reference>_formula_audit.md` (🔴 CONFIDENTIAL — references implementation specifics)

**Confidentiality**: Both files are headed `CONFIDENTIAL — Phase A artifact, NEVER feed to Phase C rebuild subagent`.

**Acceptance**:
- Self-check: can you rebuild the reference from this document alone? If any gap remains ("I didn't understand this part"), fill it before Phase 3.
- Every formula listed in Phase 1.5 must be tagged in the formula audit as `verified`, `finding`, or `missed`. No omission permitted.

---

### Phase A.2 — Building-Blocks Library Extraction ⭐

**Owner**: Claude.
**Location**: Main session.

**Goal**: Extract **atomic blocks** from the reference into a standalone `.slx` (no wiring, no topology), creating a 🟡 SHAREABLE asset for the Phase 5 rebuild subagent. This addresses a frequent failure mode: a subagent armed only with methodology cannot derive R2024b library paths or mask field names from prose, and the rebuild fails at the physical layer despite a sound design.

**Actions**:

1. Create `shared/building_blocks/<domain>_blocks.slx`.
2. Copy atomic blocks from the reference — Simscape Electrical blocks (DC Source, Universal Bridge, PMSM, `powergui`, measurement blocks), coordinate transforms (Clark, Park, inverses — pure mathematical formulae), generic signal utilities (Bus Selector preconfigured with R2024b signal names, ZOH, Rate Limiter, Unit Delay).
3. **Replace every mask parameter with a workspace-variable placeholder.** Specific numerical values must not survive into the library.
4. **Must not copy**:
   - Any already-wired subsystem (topology is design).
   - Controller blocks containing design decisions (PI gains, cost weights, prediction-horizon choices).
   - Reference-specific preset selections (e.g., a Universal Bridge `converterType` selection that encodes the reference's machine choice).
5. Lay the blocks out arbitrarily — do not preserve the reference's spatial layout. Layout hints at topology and contaminates the rebuild.
6. Produce a manifest `<domain>_blocks_manifest.md` describing each block's API use.

**Outputs**:
- `shared/building_blocks/<domain>_blocks.slx`
- `shared/building_blocks/<domain>_blocks_manifest.md`

**Confidentiality**: Headers read `SHAREABLE — Phase A.2 artifact, usable by Phase C rebuild subagent`.

**Acceptance — four hard checks**:

1. **Programmatic mask dump and on-disk verification.** For every Simscape Electrical block, dump `get_param(blk, 'MaskValues')` to a verification file (e.g., `shared/building_blocks/<domain>_blocks_verify_<date>.txt`). Eye-balling the console is not sufficient.
2. **Numerical-literal grep.** Grep the dump for numerical literals known to exist in the reference. Any match → acceptance fails. Re-`set_param` with placeholders and re-dump until zero matches.
3. **Paired review.** A second fresh session (or fresh subagent) independently probes the library's masks and compares against the manifest. Any field-level disagreement → acceptance fails.
4. **Operational-log integrity.** Operation-log entries claiming "mask placeholders substituted" must cite the dump output as evidence. Intent-style claims ("I substituted the placeholders") without evidence are not acceptable — see Appendix D for the operation-log integrity rules.

Passing only the visual check "opening the `.slx` does not reveal a working architecture" is insufficient: parameter leakage through mask values violates the discipline equally.

---

### Phase A.3 — API Know-How Checklist ⭐

**Owner**: Claude.
**Location**: Main session.

**Goal**: Consolidate the 🟡 platform-level facts learned during deep-read — R2024b Simscape paths, mask field names, Stateflow API conventions, bus signal names, solver settings, common compile errors — into a single document, so a future rebuild subagent does not have to guess library paths from memory.

**Sections (suggested)**:

1. **Simscape Electrical library paths** (full R2024b paths for blocks used by the method).
2. **Mask field dumps** — R2024b actual field names (note CamelCase conventions) for each key block.
3. **Stateflow / MATLAB Function API** — chart type (`Stateflow.EMChart` vs `Chart`), script vs Stateflow choice, sample-time configuration order.
4. **Bus signal naming conventions** — domain-specific. For example, the R2024b PMSM block bus signals are `ias, ibs, ics, w, theta, Te`.
5. **Port topology** — for each Simscape Electrical block: `LConn` / `RConn` / Inport / Outport semantics and counts.
6. **Solver configuration** — which solver + `powergui` settings produce a working inverter simulation for this domain.
7. **Common compile / simulation errors and fixes.**
8. **`sim` output unpacking** — e.g., the `StructureWithTime` format and `v.signals.values` access path.

**Output**: `shared/building_blocks/api_notes.md`.

**Confidentiality**: Headers `SHAREABLE — Phase A.3 artifact, usable by Phase C rebuild subagent`.

**Acceptance**: The document contains zero reference-specific numerical values and zero reference-model filenames. Only R2024b platform facts.

---

### Phase 3 — Methodology Summary (Phase B) — Skill Seed

**Owner**: Claude (main session, may reuse Phase 2 context).

**Critical**: This document is the **only** input the Phase 5 rebuild subagent is allowed to read about the methodology. Do not write implementation details — write the methodology.

**Actions** — answer the three groups of questions below:

**1. Minimal skeleton**
- What is the minimal skeleton for this class of control (purely structural, no parameters)?
- Which modules are **structural** (any implementation of this class must have them)?
- Which interfaces are **conventional** (how is `θ_e` / `θ_m` fed; how is `i_abc` sensed; how is the gate signal routed)?

**2. Enhancement checklist**
- What enhancements does the reference add on top of the minimal skeleton? For each:
  - **Purpose** — what problem does it prevent, or what metric does it improve?
  - **Implementation intent** — only the *what*, not the *how*.
  - Is it required or nice-to-have?

**3. Common misconceptions**
- What incorrect assumptions did you (Claude) originally hold about this class of control? Examples:
  - "Prediction horizon `N = 1` is always sufficient."
  - "A separate inner current loop is unnecessary."
  - "Speed error can be used directly as the cost-function variable."
- For each misconception, what implementation choice in the reference contradicted it?
- Write a checklist for future design of similar systems to avoid the same mistakes.

**Output**: `methods/<method>/reference_study/<reference>_understanding.md`.

**Confidentiality**: Header `SHAREABLE — Phase B artifact, usable by Phase C rebuild subagent`.

**Critical acceptance**: The document **must not contain**:
- ❌ Specific numerical parameters — use placeholders (`<Kp_placeholder>`, `<Lq_typical_range>`).
- ❌ Specific reference filenames — write "the reference" or "the baseline".
- ❌ Per-field mask dumps.
- ❌ Verbatim MATLAB Function code — write pseudocode or the mathematical formula it implements.

---

### Phase 4 — Verbal Self-Check

**Owner**: Claude narrates + user judges pass / fail.
**Location**: Main session.

**Actions**:

1. From the `understanding.md` produced in Phase 3, Claude **narrates aloud** the rebuild plan:
   - What gets built first.
   - How the key signal flows are routed.
   - How the three main design decisions are chosen.
   - What pitfalls to anticipate.
2. The user listens and identifies, on the spot, any missing point, misinterpretation, or hallucination.
3. If any gap is found → return to Phase 3 and revise `understanding.md`; otherwise → proceed to Phase 5.

**Acceptance**: User signs off — "ready for Phase 5."

---

### Phase 4.5 — Theory Sanity Check by Hand ⭐

**Owner**: Claude.
**Location**: Main session.

**Trigger**: The skill involves designing a feedback controller (PI / PID / state-feedback / sliding-mode / observer-based) **and** the simulation involves transient testing (load step, disturbance rejection, reference tracking).

**Why this phase exists**: One of the most consequential silent-failure modes in AI-driven controller design is a closed-loop disturbance transfer function with a slow pole that is *independent* of the controller bandwidth. A canonical example:

> For a speed-loop PI controller designed by pole-zero cancellation (PZC) — `K_i / K_p = B / J` — the closed-loop disturbance transfer function `ω(s) / T_L(s) = -s / [J s² + (B + K_t K_p) s + K_t K_i]` factors so that the slowest pole is `s = -B / J`, **with no dependence on the chosen bandwidth `ω_c`**. Bandwidth tuning does not improve disturbance rejection — a fact treated as first-year material in standard control textbooks (Schröder, *Elektrische Antriebe — Grundlagen*; Kazmierkowski, Krishnan and Blaabjerg, *Control in Power Electronics*; Bose, *Modern Power Electronics and AC Drives*), but easily missed when the design is generated programmatically and verified only by simulation.

Without Phase 4.5, this kind of failure surfaces only when a fresh-subagent Phase 8 simulation runs out of evaluation window — a costly late catch. Phase 4.5 intercepts it before the rebuild begins.

**Procedure** (~30 minutes):

1. **Derive three closed-loop transfer functions by hand** — written, not just narrated. On paper, in a `.m` comment block, or in a SKILL.md section:
   - **Reference tracking**: `y(s) / r(s)`.
   - **Disturbance rejection**: `y(s) / d(s)` — the most critical one for load-step scenarios.
   - **Noise** (if a sensor noise input is modeled): `y(s) / n(s)`.

2. **Factor the closed-loop characteristic polynomial.** Identify every closed-loop pole. Mark the **slowest pole** `s_slow = argmin_i |Re(s_i)|`. The equivalent time constant is `τ_max = 1 / |Re(s_slow)|`.

3. **Substitute numerical values** at the typical operating point plus worst-case bounds. Compute `τ_max` using the actual plant `J / B / K_t` and the actual controller `K_p / K_i / ω_c`.

4. **Compare with the test-scenario evaluation window `T_window`.**
   - `T_window` = the time interval over which the acceptance metric is evaluated (e.g., "the last 20 % of the simulation," "the 0.4 s window following a load step").
   - **Hard rule**: `T_window ≥ 5 · τ_max`.
   - If the rule fails, the acceptance metric will report what the controller has *not yet settled to*, not what it converges to. Two valid responses:
     - (a) Extend the simulation time until `T_window` is sufficient (controller unchanged).
     - (b) Redesign the controller so `τ_max` shrinks (simulation time unchanged).
     - **Not valid**: "it looks close enough" — that is exactly the silent-failure mode above.

5. **Record the derivation.** In `methods/<m>/skill_draft/scripts/<controller>_design.m`, add a comment block containing the three transfer-function derivations, the slowest-pole expression, the numerical `τ_max`, and the `T_window` compatibility verdict. Mirror to `shared/formulas/<domain>_formulas.md` if a stable section for closed-loop properties exists.

**Output**:
- `methods/<m>/skill_draft/scripts/<controller>_design.m` containing a Theory Sanity Check comment block.
- Optional: a "Theory Sanity Check Result" subsection in `methods/<m>/skill_draft/SKILL.md`.

**Acceptance**:
- [ ] All three transfer functions are derived in writing (algebraic form, not narrated).
- [ ] `τ_max` is computed with substituted numerical values.
- [ ] `T_window ≥ 5 · τ_max` is verified, OR one of the two valid responses (extend `T_window`, shrink `τ_max`) is applied.
- [ ] The verdict is committed to `design.m` or `SKILL.md`.

**Failure fallback**: If a fresh-subagent simulation in Phase 8 fails and the root cause is a missed slow pole, return to Phase 4.5 to backfill, and additionally treat this as a Phase 10 trigger (does the Phase 4.5 procedure itself miss a case?).

**Validated case**: In one application, a speed-loop PI designed by PZC was applied to a plant with `J = 6.33 × 10⁻⁴ kg·m²` and `B = 3.04 × 10⁻⁴ N·m·s`. The disturbance-rejection slow pole evaluated to `s = -B / J ≈ -0.48 rad/s`, giving `τ_max ≈ 2.08 s` and a settling time `t_disturb_settle ≈ 4 J / B ≈ 8.3 s`. The acceptance metric evaluated "the last 20 % of a 1.0 s simulation" — a `T_window = 0.2 s` against a required `5 · τ_max ≈ 41.7 s`. Two orders of magnitude short. Without Phase 4.5, the failure surfaced only at fresh-subagent verification; with Phase 4.5, the mismatch is caught before the rebuild begins. The remedy (option b above) was to redesign the PI by symmetric-optimum tuning, which restored a slow-pole expression depending on `ω_c` and recovered disturbance rejection. See Appendix C, Case 1.

#### Sub-condition: PI-Heuristic Transferability Check

**Trigger**: The controller design borrows a heuristic from a different control family — typically a PI-derived rule of thumb such as "decrease `a` to speed up the response" or "increase `ω_c` to improve disturbance rejection."

**Action**: Verify that the parameter-sensitivity scaling actually transfers. **Pay attention to the inverse exponent** — it is family-specific:

| Family | Key scaling | Inverse exponent | Implication |
|---|---|---|---|
| **PI** (PZC or symmetric-optimum) | `ω_c ∝ 1 / (a · T_eq)` | linear | `a = 2` doubles speed |
| **SMC** (super-twisting, Kessler-tuned) | `λ ∝ 1 / (a² · T_eq)` | squared | `a = 2` quadruples speed — the time-scale ratio drops from 4× to 2×, violating cascaded time-scale separation |
| **DTC** (symmetric-optimum outer loop) | `τ_max` non-monotonic in `a` | cubic with paradox | `a = 4` can be slower than `a = 3` |

**Warning**: Heuristics do **not** transfer across families without verification. In the SMC case above, `a² · T_eq` scaling causes `a = 2` to violate cascaded time-scale separation, so any `<controller>_design.m` for that family must hard-reject `a_so < 4`.

**Procedure**:
1. List every PI-derived heuristic the method uses ("smaller `a` is faster," "larger `ω_c` rejects disturbance better," "larger `K_p` is faster").
2. For each heuristic, look up the method's closed-loop transfer function (already derived in Phase 4.5 step 1).
3. Check the actual scaling of bandwidth / time constant / convergence rate with respect to `a / ω_c / K_p`.
4. If the scaling is not PI-style linear → add an explicit warning to `SKILL.md` and a hard-reject branch to `<controller>_design.m`.

**Output**: A `heuristic_check` comment block in `<controller>_design.m` plus, if the scaling deviates, a "PI-Heuristic Warnings" section in `SKILL.md`.

**Acceptance**:
- [ ] Every PI-derived heuristic used in the design is listed.
- [ ] For each, the actual scaling law is verified against the family's transfer function.
- [ ] Non-linear scalings have an explicit warning in `SKILL.md` and a corresponding hard-reject in `design.m`.

**Validated cases**: Two distinct deviations from linear PI scaling have been documented in distilled skills: (i) a super-twisting SMC where `λ ∝ 1 / (a² · T_eq)` invalidated the "`a = 2` halves the settling time" intuition; (ii) a DTC outer loop where `τ_max` was non-monotonic in `a` with a cubic paradox. Both led to hard rejects in the corresponding `design.m`. See Appendix C, Cases 2 and 3.

---

### Phase 5 — Independent Rebuild (Phase C) — Subagent Isolation ⭐

**Owner**: Claude, via a spawned subagent (Agent tool).
**Location**: **Isolated subagent context** — this is the critical isolation step.

**Why a subagent**:
- The main session has already read the deep-dive document; main-session context is contaminated and cannot rebuild blind.
- A fresh subagent starts with an empty context window — it cannot "peek" at Phase A artifacts even if it tries.
- A subagent can run in the background under user supervision.

**Subagent prompt template**:

````
You are an independent rebuild subagent for the {domain} control system.

## Task

Build a {domain} control model from first principles. Deliverables:
- A compileable, simulatable `.slx` file.
- A programmatic build script `build_rebuild.m`.
- A run-output `.mat` (full waveforms).
- `rebuild_summary.md` — a report describing what you built and why.

## Allowed reads (🟢 / 🟡 SHAREABLE assets)

- `<project>/methods/<method>/reference_study/<reference>_understanding.md` — the methodology summary; primary methodology reference (🟢).
- `<project>/shared/formulas/<plant>_formulas.md` — signed equations (🟢).
- `<project>/shared/building_blocks/<domain>_blocks.slx` — atomic blocks library, no wiring (🟡).
- `<project>/shared/building_blocks/api_notes.md` — R2024b platform facts (🟡).
- `<project>/shared/building_blocks/<domain>_blocks_manifest.md` — block inventory (🟡).

## Strictly forbidden reads (three-layer must-NOT-read template — see Appendix F for full SOP)

### Layer 1 — Project-confidential (confidentiality boundary)
- `<project>/methods/<method>/reference_study/<reference>_deep_dive.md` (Phase A internal document).
- `<project>/methods/<method>/reference_study/**` (any other Phase A artifact).
- `<project>/shared/reference_models/**` (the reference `.slx` originals).
- Any `*_deep_dive.md`, `*_dump.md`, or `*_diff.md` anywhere — diff tables contain topology details.

### Layer 2 — Historical contamination (cheat sources)
- `<project>/sessions/**` — project session logs (heavy contamination source).
- `<project>/methods/<method>/rebuild_round*/**` (previous rebuild iterations).
- `<project>/methods/<method>/phase8_run/session_*_run/**` (previous verification working directories, including FAIL audit trails).
- Any `PHASE_REPORT*.md` (verification stage reports).
- `<project>/methods/<method>/phase8_run/HANDOVER.md`, `build_*.m`, `*_log.txt` (historical build / handover).

### Layer 3 — Meta-contamination (workflow layer)
- `<project>/methods/<method>/skill_draft/leak_audit.md` (if it exists — meta-document; self-prohibited by anti-contamination discipline).
- Do not run `/motor-control` or any equivalent project-init skill (it auto-loads `sessions/<latest>/session-memory.md`, a heavy contamination source).
- Do not run wildcard sweeps like `find . -name "*.md" | xargs cat` that bypass the layered exclusion.

## Acknowledgement template

Before executing, post four confirmation lines:
1. "I have read the listed must-read files (count + list)."
2. "I understand the must-NOT-read list (3 layers, X categories total)."
3. "I will not open any file in the categorized exclusions, even if cited from must-read files."
4. "If unsure whether a path falls under exclusion, I will skip + escalate to main session."

## Build guidance ⭐ — prefer the building-blocks library over guessed library paths

When constructing the model, use `add_block('<project>/shared/building_blocks/<domain>_blocks.slx/<BlockName>', target_path)` to copy from the building-blocks library. Only use raw `simulink/...` library paths for generic blocks not present in the library (Constant / Sum / Gain / etc.).

## Verification scenario

{scenario_spec} — fully replicate the reference's scenario (reference profile, disturbance step, simulation time).

## Failure escalation

If the methodology summary, API notes, and building-blocks library together are insufficient to complete the task, **stop and report**. In `rebuild_summary.md` under a section "Gaps in understanding / API / building blocks," list:
- What specific information is missing.
- What default assumption you made (acknowledge it is a guess).
- Which document should be revised to fill the gap (`understanding.md` / `api_notes.md` / `blocks.slx`).

Do not guess silently. Silently guessing produces "runs but wrong" output, which is harder to diagnose than an explicit gap report.

## Confidentiality

Do not transmit any file content to external services.
````

**Subagent tool authorization**: Read / Write / Edit / Bash / matlab MCP tools. **Not** authorized: WebFetch.

**Output**:
- `methods/<method>/rebuild/rebuild_model.slx`
- `methods/<method>/rebuild/rebuild_build.m`
- `methods/<method>/rebuild/rebuild_waveforms.mat`
- `methods/<method>/rebuild/rebuild_summary.md`

**Acceptance — five hard checks** (all must pass before Phase 6):

1. **Four-piece delivery is complete.**
2. **The model is self-contained and one-click runnable.** The user double-clicks the `.slx` in the Simulink GUI and presses Run; the simulation must run without first executing a build script to populate the base workspace. Concretely:
   - **`InitFcn` injection of parameters**: the model property `InitFcn` field must contain every workspace parameter (`Rs / Ld / Lq / ψ_f / J / B / Pn / Vdc / Tsc` etc.), in the same style as the reference.
   - **Algorithm code is inline**: MATLAB Function block `Script` fields must contain the complete algorithm code; a one-line wrapper calling an external `.m` is not allowed.
   - **Controller-internal parameters are co-sourced with the plant**: parameters like `Rs / Ld / Lq / ψ_f` hard-coded inside a chart `Script` are acceptable provided the hard-coded values are *identical* to those injected by the plant's `InitFcn`. The build script can use `sprintf` to embed workspace variables into the chart `Script` at construction time. Do **not** place controller parameters in an external `.m` — they will decouple from the plant parameters and produce model mismatch.
3. **Build script is idempotent.** `build_rebuild.m` must run repeatedly without error (handle "model already exists," "chart already configured," etc.).
4. **`summary.md` contains a design-decision table.** Every key decision must be recorded as `(option, choice, rationale)`. "I chose X" without rationale is not acceptable.
5. **Round-1 expected FAIL is formally declared.** If the Round 1 baseline is intentionally designed to FAIL a particular Mode-B metric (e.g., a sliding-mode controller with discontinuous `sgn` will exhibit chattering; a single-vector finite-control-set MPC will hit bandwidth limits; a DTC implementation will show start-up ripple), the `SKILL.md` / `understanding.md` must explicitly state "Round 1 expected to fail Mode B `Sx` (specific metric, specific threshold)." This prevents an expected pedagogical FAIL from being misdiagnosed as an implementation bug — see Appendix C, Case 3 for a documented case where 18 experiments under a broken-FOC implementation were initially misdiagnosed as control-law limitations, prompting unnecessary architectural redesigns until the silent-failure root cause (Simulink `Goto` blocks with local visibility) was found.

Any failed check → the subagent must redo the task. The main session **must not** "patch + claim pass" (this violates the operation-log integrity rule of Appendix D).

#### Sub-condition: Round-Chain Bypass on Historical Pollution ⭐

**Trigger**: Round 1 baseline FAILs, but diagnosis shows the root cause is **not** the control law itself (it is an implementation bug / topology error / R2024b API quirk / Simulink silent-failure mode).

**Decision tree**:

```
Round 1 FAIL
  │
  ├─ Root cause is the control law itself (e.g., chattering, bandwidth limit, inherent ripple)?
  │     └─ YES → proceed with round 2 reverse-correction pedagogy (standard flow)
  │
  └─ Root cause is an implementation bug?
        ├─ Is the fix a single-line setting / API quirk?
        │     └─ Apply the root-cause fix; promote the patched build directly as the v1 baseline.
        │        Keep the sgn → sat → STA (or equivalent) round chain in DEVELOPMENT.md as
        │        historical context, NOT as SKILL.md teaching content.
        │
        └─ Is the fix an architectural change (topology / wiring)?
              └─ Abandon the current round chain. Restart Phase 5 with a corrected build_template.m.
                 Earlier round-chain experiments become history-of-error in DEVELOPMENT.md.
```

**Anti-pattern**: Running round 2 / round 3 on top of a broken implementation generates spurious data that contaminates the reverse-correction pedagogy. The skill ends up "teaching" the wrong lesson because round 2's apparent improvement is masking, not solving, the underlying bug.

**Validated case**: In one rebuild, a Round 1 SMC baseline with the `sgn` reaching law was expected to fail by design with chattering. Rounds 2 (`sat`) and 3 (super-twisting) were intended to reduce chattering. Across multiple experiments the chattering persisted, prompting hypotheses that the closed-loop PI inner loop was the chattering source and pointing toward architectural redesign. The actual root cause was much simpler: a Simulink `Goto` block providing the electrical angle `θ_e` for the inverse-Park transform was configured with default local-tag visibility, so the matching `From` block inside an Anti-Park subsystem silently saw nothing and the FOC degraded to an open-loop lab-frame inverse-Park. A single-line fix — `set_param(<Goto path>, 'TagVisibility', 'global')` — reduced `iq_std` by two orders of magnitude (from `32.8` to `0.250`). The methodology decision following this discovery: the patched super-twisting + PD-type sliding controller (with the `Goto` visibility fix and a `Vdc` raised to give healthy BEMF margin) was promoted directly as the v1 baseline, bypassing the sgn → sat → STA pedagogical round chain. The 18 invalidated experiments are retained in `DEVELOPMENT.md` as a documented history-of-error, but `SKILL.md` does not teach from them. See Appendix C, Case 3.

**Why retain in DEVELOPMENT.md but not delete**: The history-of-error is itself a methodology contribution — it documents "how to avoid running round chains on broken implementations." But the production `SKILL.md` cannot present broken-FOC artifacts as teaching examples; that would mis-teach the user.

---

### Phase 6 — Structural and Behavioral Diff + Visual Check ⭐

**Owner**: Claude (main session) + **user mandatory review** (the user must not be bypassed; Claude is not authorized to self-pass).

**Acceptance has two layers**: structural (Layer 1) and behavioral (Layer 2). Layer 2 evaluation begins **only after** Layer 1 passes — comparing waveforms when the structures already disagree wastes signal.

#### Layer 1 — Structural Diff

The main session programmatically dumps:

1. For the reference: every top-level block + one-level subsystem expansion + every block's full mask + every signal-line connection (`src → dst port`) + model properties (Solver, StopTime, sample times, `InitFcn`).
2. For the rebuild: the same.
3. A diff table written to `sessions/session-NNN/<reference>_vs_<rebuild>_diff.md` (**session-local, not committed to git**, because it contains reference-specific topology).
4. **Item-by-item classification with the user** (this is the strict rule — Claude must not auto-classify). For every diff entry, the user assigns one of four categories:
   - 🟢 **Subagent implementation bug** → reverse-revise `understanding.md` to constrain the next round.
   - 🟡 **Methodology gap in `understanding.md`** → reverse-revise `understanding.md` (must be fixed before the next round).
   - 🟡 **R2024b platform-API fact** → reverse-revise `api_notes.md`.
   - 🔴 **Reasonable design choice by the subagent** → do **not** revise documentation; preserve the subagent's design autonomy. Future rounds may diverge again on this point without it counting as a bug.
5. **Claude must not auto-classify**: if Claude pre-fills the classification column, it tends to over-fit by labeling every difference as "🟡 methodology gap," which collapses `understanding.md` into a faithful copy of the reference and destroys generalization. The classification column must arrive at the user empty.

#### Layer 2 — Behavioral Diff (only after Layer 1 passes)

##### Pre-condition: Visual 4-Check ⭐

**Why this gate exists**: Numerical-only metrics (hit rate, RMS, std, overlay agreement) can produce plausible values on a broken implementation. The metrics will not reveal a controller that is in fact running open-loop in the lab frame, nor a motor that is in fact stalled while a current loop saturates against its rails. Failing to look at the physical waveforms before scoring them has a documented cost: in one validated case (Appendix C, Case 3), 18 experiments under a silently broken implementation produced plausible `iq_std` values that misled the diagnosis for the better part of a week. The Visual 4-Check, if it had been enforced earlier, would have intercepted the first experiment via the lab-frame DC-locked `abc` signature.

**4 mandatory visual checks** — all four must pass before any 5-metric overlay is computed:

| Check | Signal | PASS criterion | Example failure signature |
|---|---|---|---|
| **1. Motor rotates** | `ω_m` | Tracks `ω_ref`; not stuck near 0 and not oscillating around a stalled equilibrium | `ω_m` long-term below 5 % of `ω_ref` → motor out of control |
| **2. `i_q` tracks** | `i_q,meas` vs `i_q,ref` | `i_q,meas` tracks smoothly, **not** bang-bang at `±i_q,max` | `i_q,meas` persistently at `±i_q,max` → broken-FOC under-current saturation |
| **3. `abc` AC sinusoidal** | logger channels `i_a / i_b / i_c` | AC sinusoidal at the electrical frequency, **not** DC-locked at a single angle | DC-locked `abc` = lab-frame open-loop signature: the rebuilt model is performing the inverse-Park with a constant electrical angle, indicating that the angle-source signal is not reaching the transform |
| **4. Torque energy balance** | `T_e` vs `T_L + B·ω` | Steady-state `T_e ≈ T_L + B·ω` | `T_e` far from `T_L + B·ω` → energy imbalance, indicating a model-level bug |

**Any visual check failing → metrics are not trustworthy → reverse-revise the implementation and re-run. Do not proceed to the 5-metric overlay.**

##### 5-Metric Overlay (only after the Visual 4-Check passes)

Load the baseline waveforms and the rebuild waveforms; overlay the five primary signals (`ω_m / i_q / i_d / T_e / i_abc`); compute pointwise hit rate.

**Layer 2 acceptance**: Visual 4-Check passes **and** every primary signal hits ≥ 95 %.

##### Layout Gate (readability — mandatory, not advisory) ⭐

**Why this gate exists**: An automated build (a subagent issuing `add_block` / `add_line` programmatically) produces, by default, a tangled diagram — overlapping blocks, lines cutting through blocks, a mesh of line-line crossings — that a human cannot review. An advisory "please lay it out neatly" clause gets silently skipped. The layout gate makes readability a hard constraint — and because it **only moves blocks/lines and never changes a connection**, the simulated behavior is 100 % unchanged.

| Tier | Metric | Nature | How it is met |
|---|---|---|---|
| **Hard** | `block_overlaps == 0` | enforced by self-test T7 (build FAILs otherwise) + a mandatory human screenshot sign-off | always reachable on a 2-D plane; the main session exports a `.png` for the user (the same human-in-loop model as the Visual 4-Check) |
| **Soft** | line-line crossings / line-through-block hits | **reported only, NEVER a numeric gate** | a layout-tidy pass (arrange + de-overlap + planarity diagnosis) minimizes them |

**Why crossings cannot be hard-gated**: A non-planar signal graph (a K3,3 or K5 subgraph) cannot reach zero crossings on a 2-D plane (Kuratowski) — N estimators each fanning out to a selector *and* a logger while sharing inputs is a textbook K3,3. Reaching zero requires *restructuring* (encapsulating fan-in groups into a Subsystem, routing long or shared signals via Goto/From), not moving blocks; hard-gating a crossing count would reject valid non-planar models.

**`arrangeSystem` pitfall**: `arrangeSystem(mdl, 'FullLayout')` fixes overlaps and compactness but can *worsen* crossings — the tidy pass must measure crossings before and after and roll back when `after > before` (never accept a net crossing regression).

**Validated observation**: A roughly 90-block auto-built model could not be made readable by block-moves alone — its signal graph is provably non-planar (a K3,3 witness subgraph) — establishing the boundary: small models tidy by moving blocks, complex non-planar models require restructuring (Subsystem encapsulation + Goto/From).

#### Failure handling

- Any Layer failing → Phase 6 reverse-correction round `N` (rules below).
- Both Layers passing → Phase 6 PASS, proceed to Phase 7.

#### Reverse-correction routing matrix

| Symptom | Root-cause layer | Document to revise |
|---|---|---|
| Structural diff shows missing / extra block | Design / methodology gap | `understanding.md` (user assigns 🟢 or 🟡) |
| Structural diff shows mask-field disagreement | Platform-API knowledge | `api_notes.md` (🟡 R2024b fact) |
| Structural diff shows different wiring | Design choice vs error | Discuss with user, classify |
| Hit rate poor but waveform shapes match | Design decision | `understanding.md` |
| Physical layer not working (`i_abc ≡ 0`, `T_e = NaN`, etc.) | Platform-API or mask config | `api_notes.md` + `blocks.slx` |
| Signal name not found / port cannot connect | R2024b platform fact | `api_notes.md` |
| Compiles but diverges | Design decision or discretization | `understanding.md` (possibly a solver entry in `api_notes.md`) |
| Single block's mask formula is wrong | Platform-API fact | `api_notes.md` |

**Output**:
- `sessions/session-NNN/<reference>_vs_<rebuild>_diff.md` (structural-diff table + user classifications; session-local, **not committed to git**, contains reference-specific topology).
- `methods/<method>/rebuild/comparison.png` (behavioral overlay — five or six panels).
- `methods/<method>/rebuild/comparison_report.md` (hit-rate table + analysis; numeric only, no topology, may be committed to git).

**Composite acceptance**:
- Layer 1: every structural diff entry has been classified, with user confirmation that no entry remains unassigned.
- Layer 2: Visual 4-Check passes **and** every primary signal hits ≥ 95 %.
- **Layout gate**: `block_overlaps == 0` (self-test T7 hard pass) + the user has seen a diagram screenshot; crossings / line-through-block are reported soft, not gated.
- **Claude must not self-pass**: every acceptance gate must have a user-visible review trace (in the session-memory bookmark or commit message).

---

### Phase 7 — Skill Draft

**Owner**: Claude (main session).
**Precondition**: Phase 6 has passed.

**Actions**:

1. Extract the reusable portions of `understanding.md` → `SKILL.md` (frontmatter + body).
2. Extract a generic template from `rebuild_build.m` → `scripts/build_template.m` (parameters placeholder-ized).
3. Copy relevant know-how entries → `references/` inside the skill folder.
4. Add a `description` line in the skill's frontmatter that activates the skill on appropriate triggers (control-family name, plant type, simulation context).

**Output**: `methods/<method>/skill_draft/` — a draft skill not yet promoted to the global skills directory.
- `SKILL.md`
- `scripts/build_template.m`
- `references/`

**Acceptance**: The skill directory is complete; the `SKILL.md` frontmatter has a precise `description` that lets Claude Code auto-route relevant tasks to this skill.

---

### Phase 8 — Generalization Audit (fresh session + new problem) ⭐

**Owner**: A new Claude session with no prior context from this project.
**Location**: **Entirely new Claude Code conversation** (user `/clear` or new window).

**Key insight**: Whether the generalization test has a same-method baseline depends on the reference suite. Many projects only carry a single implementation of any given method (which is the same `{reference_model}` used in Phase 4–6, and reusing it would degenerate Phase 8 into a Phase 6 re-run). Implementations of *different* methods are not behavior-compatible (a DTC implementation is not a valid baseline for a finite-control-set-MPC rebuild). Hence two acceptance modes:

**Main-session preparation** (before Phase 8 starts):

1. Pick a different machine-parameter source (any reference can contribute its motor body parameters, but only the parameters — not its control-layer topology).
2. Design a different scenario (different `ω_ref`, different `T_L` profile, different time constants).
3. Decide Mode A or Mode B (criteria below).
4. Write the fresh-session prompt (task + parameter table + scenario + baseline path if any + acceptance-mode declaration).

#### Phase 8 Brief Design — `SKILL.md` Envelope Alignment ⭐

**Action**: The Case parameters (`J / i_q,max / sim_time / T_L,max / V_dc / ...`) in the fresh-subagent prompt must lie inside the default envelope declared in `SKILL.md`'s Required Inputs section. **A brief that drifts outside the envelope is an acceptance-test design issue, not a SKILL.md fix trigger.**

**Pre-flight checklist** (before writing the prompt):

| Parameter | Check |
|---|---|
| `J / B / Pn` | Within 2× of the SKILL.md default envelope |
| `i_q,max` | ≥ 1.4 · `i_q,steady at T_L,max` (avoid starvation-saturation artefacts) |
| `sim_time` | ≥ 5 · `τ_max` (the slowest pole from Phase 4.5) |
| `V_dc` | ≥ 1.5 · `ω_max · ψ_f / √3` (peak phase BEMF) |
| `T_L,max` / step time | `T_L,step_time ≥ ramp_time + 5 · τ_inner` so the load step does not coincide with a transient |

**Anti-pattern**: A subagent reports "the scenario drifted outside the SKILL.md envelope" → main session mistakes the report for a SKILL.md fix request. The correct response is to adjust the brief (raise `J`, raise `i_q,max`, lengthen `sim_time`) as an acceptance-test design improvement.

**Validated case**: A Phase 8 run was iterated through three brief versions before passing: Run 1 (`sim_time = 0.6 s` and `i_q,max = 10 A` against a `T_L,max = 2 N·m`) gave only `1.2×` `i_q` headroom (below the 1.4× threshold) and `sim_time` short of the slowest pole — Layer 2 returned 1 / 5. Run 2 lengthened `sim_time` to 1.0 s → Layer 2 returned 2 / 5. Run 3 raised `J` to `4 × 10⁻⁴ kg·m²` and `i_q,max` to 12 A so the Case fell back inside the SKILL.md envelope → Layer 2 returned 4 / 5 + Visual 4 / 4 passed. The FAIL → PASS progression surfaced legitimate `SKILL.md` v1.x improvement candidates (e.g., explicit guidance for high-`M` plants), which were captured for later but did not block the v1.0 promotion. See Appendix C, Case 3.

**Fresh-session execution** (the prompt fires):

1. The new session reads `methods/<m>/skill_draft/SKILL.md` plus helpers — and **does not** read `deep_dive.md`, `understanding.md`, dumps, or any `rebuild_round*/`.
2. Following SKILL.md, it builds the full rebuild (plant + measurements + outer loop + chart + logging + solver + `InitFcn` + self-tests).
3. It runs Layer 2 verification under the chosen mode.

#### Mode A — Same-method baseline available
- Applies when the reference suite includes a second independent implementation of the same method.
- Baseline: that second implementation's waveforms under the new scenario.
- Acceptance: 5-metric overlay hit rate ≥ 90 % (looser than Phase 6's 95 %, by 5 %).

#### Mode B — Sanity-check, no same-method baseline (the common case)
- Applies when only a single same-method reference exists (i.e., the `{reference_model}` from Phase 4–6 itself).
- Baseline: **none**. Do not reuse `{reference_model}` (Phase 8 would degenerate to a Phase 6 re-run). Do not use a different-method implementation as baseline (incompatible behavior).
- Borrowed resource: the motor body parameters (`Rs / Ld / Lq / ψ_f / Pn / J / B`) from any reference; never its control-layer topology.
- Custom scenario: square-wave `T_L`, stepped `ω_ref`, different time constants — substantially different from the `{reference_model}` scenario.
- Acceptance: ≥ 4 / 5 = 80 % of the following physical-sanity criteria (looks looser than Mode A but is in fact stricter — physical sanity must satisfy engineering reality, not match some reference's behavior):
  - **S1**: `ω_m` settles to `ω_ref ± 5 %` (after settling, post-load).
  - **S2**: `ω_m` post-load recovers to ≥ 95 % of target within ~`5 / inner_loop_BW`; never collapses below 50 % of target.
  - **S3**: `i_q` stays within saturation limits and is well-loaded at steady state (healthy PI, no wind-up).
  - **S4**: `i_d` settles to `i_d,ref ± 0.5 A` (no model mismatch).
  - **S5**: numerical sanity (no NaN / Inf; `i_abc` is sinusoidal with no DC offset; `ω_m` has no monotonic drift).

**Common acceptance (both modes)**:
- The skill is **structurally self-contained**: the fresh session asks no clarifying questions before starting work.
- Layer 2 hit rate meets the chosen-mode threshold.

**Failure handling**:
- Fix the SKILL.md `description` / scripts → re-run Phase 8.
- After two Phase 8 failures the skill is not mature → tag `draft`, do not promote.

#### Phase 8 PASS — 4-Gate Sequence (hard sequence) ⭐

**Why a strict sequence**: "Acceptance" used loosely as a single test invites silent skipping. Fresh-subagent numerical PASS is a *necessary* condition for promotion but **not sufficient** — the main session can pass numerical and audit checks while the user has not yet looked at the simulated waveforms in MATLAB. A documented case (Appendix C, Case 2) shows what happens when this gate is bypassed: a skill was promoted to the global skills directory after numerical PASS + reverse-leak audit; the user later flagged that no manual visual review had occurred; the promotion was reverted, the user reviewed in MATLAB, and only then re-promoted. The lesson: **AI numerical PASS and user terminal visual review are different acceptance layers; they cannot substitute for each other**.

**4 gates, in order — all must pass before Phase 9**:

| Gate | Owner | PASS criterion | Failure path |
|---|---|---|---|
| **G1 Numerical** | fresh subagent | Layer 1 self-tests all PASS (default 5 / 5, including the layout gate T7 `block_overlaps == 0`) **and** Layer 2 hit rate meets the mode threshold (A: ≥ 90 %, B: ≥ 4 / 5 = 80 %) | FAIL → enter Refinement Loop on FAIL (below) and bump `v0.x` |
| **G2 Reverse-leak audit** | main session | A 4-grep audit returns clean: matches only against the SKILL.md changelog metadata or compliance statement, **not** against any must-NOT-read file actually being read. The four greps: ① `rebuild_round*` ② `deep_dive\|_audit\.md\|_dump` ③ `sessions/` ④ cross-method (`methods/<other_method>/`) | FAIL → if the fresh subagent really read a must-NOT-read file, this verification is invalidated and a fresh subagent must be spawned; if it is only a textual citation, no action |
| **G3 Documentation precision** | main session | Ambiguities and vague phrasings flagged by the fresh subagent must be revised in SKILL.md before promotion (e.g., a metric specified as "monotonic increasing" without distinguishing strict-per-sample vs net-monotonic will be read strictly and falsely FAIL on any controller that has expected per-sample reverse motion from hysteresis dead-bands) | FAIL → revise phrasing; fresh-subagent verification need not be re-run (both readings have demonstrably passed); this is a documentation-level improvement |
| **G4 User visual review** | user, in person at the MATLAB desktop | The main session uses `mcp__matlab__evaluate_matlab_code` (or `matlab` CLI) to open the `.slx`, run the simulation, and bring up every relevant Scope (including XY graphs and `abc` traces). The user **must pass the Visual 4-Check** (motor rotates / `i_q` tracks / `abc` AC sinusoidal not DC-locked / `T_e` energy balance) plus any method-specific failure signature, then verbally or in-writing confirm "passed." | FAIL → user describes the anomalous waveform → enter v0.x revision cycle |

**Gate 4 Visual 4-Check** — identical to the Phase 6 Layer 2 pre-condition (cannot be skipped):
- **Check 1** — motor rotates (`ω_m` tracks `ω_ref`, not stuck).
- **Check 2** — `i_q` tracks (`i_q,meas` vs `i_q,ref` smooth, **not** bang-bang at `±i_q,max`).
- **Check 3** — `abc` AC sinusoidal, **not** DC-locked (electrical-frequency sinusoid; DC-locked = lab-frame open-loop signature).
- **Check 4** — `T_e` energy balance (steady-state `T_e ≈ T_L + B·ω`).

Plus method-specific failure signatures, for example:
- **For DTC**: the `α-β` stator-flux trajectory should be circular (a hexagonal trajectory indicates an 8-state switching-table contamination, a known failure mode of naive Takahashi tables on PMSMs).
- **For finite-control-set MPC**: the `α-β` XY trajectory should rotate in the correct direction (reversed rotation is a critical bug signature of incorrect inverse-Park sign convention).

**Plus diagram readability**: the opened `.slx` should present a readable layout (zero block overlaps is machine-guaranteed by self-test T7; the main session attaches a diagram screenshot). A tangled diagram (overlapping blocks / a mesh of crossing lines) sends the model back to the layout gate, independently of whether the waveforms pass.

**Main-session protocol**: The main session **must explicitly ask** the user: "Please open MATLAB and verify the 4 Scopes (`ω_m`, `i_q`, `abc`, `T_e`). Have all four visual checks passed?" Wait for the user's **explicit, item-wise** response (or an explicit "all four pass"). **Do not accept** an ambiguous "OK" / "looks fine" — the main session must re-confirm if the response is not explicitly four-pass.

**Triggering lesson**: 18 experiments under a broken-FOC implementation produced plausible numerical metrics that led to a wrong root-cause hypothesis ("the PI inner loop is the chattering source"). If the Gate 4 Visual 4-Check had been enforced from the first experiment, the lab-frame DC-locked `abc` signature would have stopped the misdiagnosis on day one. See Appendix C, Case 3.

**Main-session discipline**:
- Gates 1 – 3 the main session runs itself (subagent verification, grep, phrasing revision).
- **Gate 4 the main session must not pass on the user's behalf** — it must explicitly ask the user and wait for an explicit response.
- Gates 1 – 3 produce only an "**eligible** for Phase 9 promote" label; only Gate 4 PASS authorizes the `cp → ~/.claude/skills/` step.

#### Refinement Loop on FAIL ⭐

In practice the path from `v0.x` to `v1.0` is rarely a straight line. A documented case (Appendix C, Case 1) needed four FAIL-revise rounds before promotion. The original linear "write → verify → promote" model with "two FAILs = archive" hard-stop did not survive contact with reality; instead, FAIL-revise loops follow a budget.

**Versioning discipline**: every fresh-subagent verification must bump the skill's draft version first:
- `v0.1-draft → v0.2-draft → v0.3-draft → v0.4-draft → v1.0` (promote).
- A draft is always newer than production (except for the instant of the production tag).
- The `skill_draft/SKILL.md` frontmatter `changelog:` field records every `v0.x` change and its trigger reason.

**Failure-budget ladder**:

| Cumulative consecutive FAILs | Main-session action | Expected output |
|---|---|---|
| **1 FAIL** | Diagnose → patch SKILL.md → bump `v0.x → v0.(x+1)` → spawn the next subagent | FAIL-diagnosis report (`PHASE_REPORT_*.md`) + a new changelog entry |
| **2 FAILs in a row** | Same as 1 FAIL, but raise the alert (is the patch addressing the surface or the root cause?) | Same + warning flag |
| **3 FAILs in a row** | **Do not spawn.** Trigger a main-session role-transition review (below). | Role-transition audit log + anti-contamination audit table |
| **5 FAILs in a row** | **Suspend all spawns.** The methodology itself needs review (Phase 10 trigger). | Methodology `v_x → v(x+1)` revision |

##### Role-Transition Protocol

After the build phase is finished and a post-build review is needed (inspecting artifacts, comparing to the reference, finding discrepancies), the main session **may temporarily lift the anti-cheating discipline (#2) and the anti-contamination discipline (#6)** to enter a read-only review state — but under strict accounting:

1. **Trigger conditions** — all must hold:
   - Build phase is complete (Phase 5 delivered four-piece artifact + Phase 6 behavioral diff has run).
   - The skill failed a verification round; post-build review is required.
   - **The user explicitly authorizes** the transition (must be visible in the conversation — not implied).

2. **Session-memory record** (mandatory): annotate the transition timestamp + the user's verbatim authorization + the reason for the transition.

3. **Read-log audit table** (mandatory, written to session memory or the verification-report §10):

   | File path | Time | Purpose | Range |
   |---|---|---|---|
   | `shared/reference_models/foo.slx` | HH:MM | Compare InitFcn | `get_param InitFcn` (full) |
   | `sessions/session-XXX/foo.md` | HH:MM | Trace historical PI value | `grep "Kp_w"` ± 5 lines |
   | ... | ... | ... | ... |

4. **Constraints (must be preserved after transition)**:
   - The current session **cannot** perform another fresh-subagent verification (the context is now contaminated).
   - Any subsequent verification must open a new main session (user `/clear` or a new editor window) and then spawn a fresh subagent.
   - Future fresh subagents remain bound by the original must-NOT-read list (the role transition is a single-session authorization; it does **not** propagate).

5. **Audit trail**: every operation performed during the transitioned state, every finding, and every decision must be written to the verification report's §10 (Role Transition Audit).

**Why this matters**: the role transition is an **exception clause** to the anti-contamination discipline. It must be gated explicitly so it does not turn into an ad-hoc loophole. See Appendix F §5 for the full role-transition SOP.

---

### Phase 9 — Promote and Sync

**Owner**: Claude (main session).

**Precondition**: Phase 8 4-Gate sequence has fully passed, **and** the user has explicitly authorized promotion. The main session must not self-authorize promotion solely on the basis of fresh-subagent numerical PASS + reverse-leak audit clean. Gate 4 (user visual review at the MATLAB desktop) requires the user's verbal or written sign-off — see Appendix C, Case 2 for a documented case where Gate 4 was bypassed, the skill was promoted prematurely, the user flagged the missing visual review, and the promotion had to be reverted and redone.

**Actions**:

1. `cp -r` the skill from `methods/<method>/skill_draft/` to `~/.claude/skills/<name>/` (or the equivalent global skills location for your Claude Code installation).
2. Update the SKILL.md frontmatter: version `0.x-draft` or `1.0-draft` → `1.0` (or whatever stable version is being released). Optionally record the 4-Gate sequence pass + user terminal-review date in the frontmatter.
3. `git commit` the promotion. The commit message should list the 4-Gate audit trail (subagent verification numerical results + audit-grep clean + user sign-off verbatim).

**Output**: `~/.claude/skills/<target_skill_name>/` — now globally available across Claude Code sessions.

---

### Phase 10 — Methodology Self-Update ⭐

**Owner**: Claude + user.

**Action**: After every application of this workflow, reflect on the workflow itself:

- Is any Phase redundant?
- Did this application expose a new gap?
- Are the equivalence-grade thresholds (Phase 6 / Phase 8) too loose or too strict?
- Are the workflow parameters (`{reference_model}` / `{generalization_test}` / `{acceptance_criteria}` / ...) sufficient?
- Does the anti-contamination discipline (rule #6 + Appendix F) need a new template?
- Does the failure-budget ladder need adjustment?

**Output**: Update this workflow document (version + the "Validated case studies" appendix).

#### Generalization Roadmap ⭐

The methodology's defensibility scales with `N` — the number of independent applied cases.

| Status | Validates |
|---|---|
| `N = 1` | Initial methodology + anti-contamination discipline + three-layer documentation + Theory Sanity Check |
| `N = 2` | Cross-method generalization within a domain (e.g., a second control method on the same machine class) |
| `N = 3` | Cross-family generalization — three independent control families across machine sub-types |

**Publication gates**:

- **`N = 1`** — *not* publishable as a general methodology paper; single-case overfit risk is too high. Publishable as a case study + initial methodology (industrial-electronics conference / workshop venues).
- **`N ≥ 2`** — methodology contribution becomes defensible. Publishable in a journal (e.g., *IEEE Transactions on Industrial Electronics*, *IEEE Access*, *IET Electric Power Applications*).
- **`N ≥ 3`** — methodology + cross-family generalization claim becomes defensible. Extension to non-motor-control domains (power electronics, robotic motion control, industrial process control) becomes a plausible follow-up.

**Anti-patterns**:

- ❌ Investing in a top-venue methodology paper at `N = 1` — reviewers will, justifiably, raise overfit concerns.
- ❌ At `N ≥ 2`, forcing a Phase 8 PASS by interpretively loosening the acceptance threshold to clear the gate. This destroys the methodology's falsifiability and the resulting `N` claim is hollow.
- ✅ If the `N`-th case FAILs, that is itself a methodology contribution: the FAIL surfaced a gap that the next workflow revision can fix. A documented FAIL is more credible than a forced PASS.

**Current public-release validation**: `N = 3` across three independent PMSM control families (FCS-MPC, DTC, SMC) targeting surface-mounted and interior-permanent-magnet machines. See Appendix C, Cases 1 – 3.

---

## Three-Layer Documentation Discipline ⭐

A skill that survived four or more FAIL-revise cycles has three audiences with different needs. After `v0.x → v1.0` promotion, the **same raw material is rewritten three times** for those three audiences.

| Layer | File | Audience | Content | Voice |
|---|---|---|---|---|
| **Layer 1 — Production** | `~/.claude/skills/<name>/SKILL.md` | The end user who invokes the skill | "How to use" — stable methodology + current best practice | Gold standard. **No** failure history, no open trade-offs, no "we once tried X and it failed." Concise, confident, actionable. |
| **Layer 2 — Development** ⭐ | `methods/<m>/skill_draft/DEVELOPMENT.md` | Skill maintainers, future contributors, anyone forking and modifying | "How it was built" — `v0.x → v1.0` iteration history + decision rationale + known trade-offs + deferred edge cases + "why this rather than that" | Engineer retrospective. Failure history visible, decisions traceable, so a maintainer can see which choices are load-bearing and which are incidental. |
| **Layer 3 — Session memory** | `sessions/session-NNN/session-memory.md` | Researchers writing papers or reproducing the workflow | "Raw timeline" — operation log + who decided what + full chronology including detours and debugging | Append-only. Never edit historical entries. Researchers mine paper material from here. |

**Promote-time discipline** (must be followed at every `v0.x → v1.0` promotion):

1. **Production SKILL.md update**: bump version + status + add a v1.0 changelog entry. **Delete** any draft-stage phrasing like "Awaiting verification," "TODO," or "Known issue."
2. **DEVELOPMENT.md sync**: append a post-promote retrospective section to `methods/<m>/skill_draft/DEVELOPMENT.md`:
   - One-line `v0.x → v1.0` summary (what problem, solved by what).
   - Verification-rounds table (each round's FAIL / PASS count + root cause + fix).
   - Known limitations and deferred edge cases (issues acknowledged but not blocking v1.0 PASS).
   - Decision-log highlights (which decisions were contested, decided by user fiat / data / explicit push-back).
3. **Session memory** is untouched. Append-only — new entries go in the current session's `session-memory.md`; do not back-fill prior sessions.

**Why three layers, not two or four**:

- **Two layers** (SKILL.md + session memory) — readers of SKILL.md cannot see trade-offs; readers of session memory drown in timeline detail and cannot find architectural decisions.
- **Four layers** (adding an RFC / ADR layer) — overkill for skill scope; reconsider when an individual skill grows substantially in complexity.
- **Three layers** — audiences are cleanly separated, and the single-point maintenance burden stays manageable.

**Anti-pattern quick reference**:

| ❌ Anti-pattern | ✅ Correct pattern |
|---|---|
| Production SKILL.md contains "v0.3 tried pole-zero cancellation but it FAILed under low `B/J`, so we now use symmetric optimum" | Production SKILL.md decision table row: "if `B / J < ω_c / 5` → use symmetric optimum"; no failure history. DEVELOPMENT.md `v0.3 → v0.4` section details the PZC FAIL root cause |
| DEVELOPMENT.md copies session-memory entries verbatim | DEVELOPMENT.md distills architectural decisions + retrospective; timeline detail stays in session memory, referenced by `path:line` |
| Session memory is "cleaned up" by deleting historical entries | Session memory is append-only; deleting entries destroys paper material and breaks the audit trail |

### DEVELOPMENT.md Template

````markdown
# <skill-name> — Development Log

## v1.0 Release Retrospective (YYYY-MM-DD)
**One-line summary**: [problem solved + how]

## Verification Rounds

| Round | Version | Result | Root cause | Fix |
|---|---|---|---|---|
| Session-NNN | v0.1 | FAIL X/Y | ... | ... |
| Session-NNN | v0.2 | FAIL X/Y | ... | ... |
| Session-NNN | v1.0 | PASS X/Y | — | promoted |

## Known Limitations / Deferred Edge Cases

- ...

## Decision Log

### Decision N: [topic]
- Options considered: ...
- Choice: ...
- Rationale: ...
- Trade-off accepted: ...
- User pushback / data: ...

## References

- Production SKILL.md: `~/.claude/skills/<name>/SKILL.md`
- Source draft: `methods/<m>/skill_draft/`
- Session memory anchors: `sessions/session-XXX..session-YYY`
- Verification reports: `methods/<m>/phase8_run/session_*_run/PHASE_REPORT_*.md`
````

---

## Appendix A — Failure Exit Detail

### Phase 6 — First reverse-correction

- Claude reads the "Gaps in understanding" section of `rebuild_summary.md`.
- Compares the differing signals (e.g., `i_d` drift → `understanding.md` omitted the decoupling-feedforward discussion).
- Reverse-revises `understanding.md`; tag the revision (`v1.1`).
- Re-run Phase 4 → 5 → 6.

### Phase 6 — Second reverse-correction

- Claude no longer revises `understanding.md` on its own.
- The user reads `comparison_report.md` and **points to the specific entry in `understanding.md` that is wrong**.
- Claude revises per user direction; re-run Phase 5 → 6.

### Phase 6 — Third failure (archive)

- Archive to `sessions/session-NNN/failed_methodology/`.
- Write `post_mortem.md`: why did the methodology fail on this model (reference too complex? wrong abstraction level in `understanding.md`? insufficient parameterization dimensions?).
- **Do not** promote the skill; the learning stays in-project.
- Update `learnings.md`: what kind of enhancement does this class of model need from the workflow.

---

## Appendix B — Reference-Model Confidentiality Checklist

### ✅ A skill may contain

- Control-theory formulas (Park / Clarke, general MPC cost form, etc.).
- Generic code templates with parameter placeholders (`<L_q_placeholder>`, `<K_p,ω_range>`).
- Structural block lists (PWM generator, PMSM, DC source — Simulink built-ins).
- Methodology decision trees (e.g., "how to choose between `N = 1` and `N > 1`").
- Numbered references to know-how entries (do not paste the body).

### ❌ A skill must not contain

- Specific numerical parameters (`L_q = 5.513 mH` → use `<L_q>`).
- Specific reference-model filenames (`<reference_name>.slx` → "the reference" / "the baseline").
- Mask-field dumps.
- Author or affiliation information from the reference.
- Anything that could let a reader trace the skill back to a specific advisor, institution, or research group.

### Audit checklist (run before Phase 7 closes)

- [ ] `grep` the full skill text for the reference's filename → zero hits.
- [ ] `grep` for specific numerical literals known to exist in the reference (machine constants, controller-gain values, time-constant defaults) → zero hits.
- [ ] Could the skill be applied to another problem in the same class by changing parameters alone? Test by substituting parameters and verifying it still runs.

---

## Appendix C — Validated Case Studies

The methodology was applied to three independent control families (FCS-MPC, DTC, SMC) on surface-mounted and interior-permanent-magnet synchronous machines. The cases below are **anonymized narrative** summaries — domain, problem encountered, diagnosis, resolution, and the methodology insight surfaced. They are **not** accompanied by raw artifacts (`.slx`, waveforms, logs) in this public release; read them as documented engineering experience, not as independently reproducible benchmarks. Where the cases are referenced from the main workflow text (Phase 4.5, Phase 5 round-chain bypass, Phase 6 / 8 Visual 4-Check, Phase 8 4-gate, Phase 9 promote gate), the corresponding rule was either introduced or sharpened by the case.

### Case 1 — Pole-Zero-Cancellation PI Silent Failure (`N = 1` validation)

**Domain**: Single-vector finite-control-set MPC inner-current loop + cascaded PI speed loop, surface-mounted PMSM.

**Plant**: `J = 6.33 × 10⁻⁴ kg·m²`, `B = 3.04 × 10⁻⁴ N·m·s`, `K_t ≈ 0.36 N·m / A`. `B / J ≈ 0.48 rad/s`.

**Problem**: The speed-loop PI was designed by pole-zero cancellation (`K_i / K_p = B / J`) and verified in simulation against a load-step disturbance scenario. Fresh-subagent Phase 8 verification returned only 2 / 5 on the acceptance metric. The metric in question was the post-load `ω_m` recovery within "the last 20 % of a 1.0 s simulation."

**Diagnosis**: The closed-loop disturbance-rejection transfer function `ω(s) / T_L(s) = -s / [J s² + (B + K_t K_p) s + K_t K_i]`, after the PZC factorization, has a slow pole at `s = -B / J ≈ -0.48 rad/s` that is **independent of the chosen bandwidth `ω_c`**. The equivalent time constant `τ_max ≈ 1 / |s| ≈ 2.08 s`; the load-step settling time `t_disturb_settle ≈ 4 J / B ≈ 8.3 s`. The acceptance metric's evaluation window `T_window = 0.2 s` was over two orders of magnitude shorter than the required `5 · τ_max ≈ 41.7 s`. The controller had not failed; the controller had not yet settled. The metric was reading what the controller had *not yet converged to*.

**Resolution**: Redesigned the PI by symmetric optimum, which restored a slow-pole expression dependent on `ω_c` and recovered disturbance rejection. The version timeline: `v0.1` FAIL (1 / 5 — different root cause, metric specification error) → `v0.2` FAIL (different scenario shortfall) → `v0.3` FAIL (the PZC silent-pole case above) → `v0.4` PASS (5 / 5 + Layer 1 5 / 5). Four FAIL-revise rounds, three distinct root causes, one PASS.

**Methodology insight**: This case introduced **Phase 4.5 — Theory Sanity Check by Hand**. A 30-minute hand derivation of the three closed-loop transfer functions (reference / disturbance / noise), pole factorization, and a `T_window ≥ 5 · τ_max` check would have intercepted the PZC failure before the rebuild even began, instead of waiting for a fresh-subagent simulation run to surface it.

### Case 2 — Phase 8 Gate 4 Bypass and Recovery (`N = 2` validation)

**Domain**: Direct Torque Control with αβ-frame stator-flux estimation and a Sutikno 6-state switching table, interior-permanent-magnet PMSM with `L_q / L_d ≈ 3.0`.

**Problem**: After fresh-subagent Phase 8 verification returned Layer 1 5 / 5 + Layer 2 5 / 5 and the main session's reverse-leak audit returned clean, the main session interpreted these results as sufficient for promotion. It executed `cp → ~/.claude/skills/`, updated SKILL.md frontmatter to `version: 1.0`, and recorded the promotion in the project skills table — **without** explicitly asking the user to open the `.slx` at the MATLAB desktop and run a manual visual inspection of the Scopes.

**User flag and recovery**: The user noticed and pointed out that they had not actually reviewed the model. The main session reverted the promotion (moved the skill to `~/.claude/_trash/`), opened the model in MATLAB, ran the simulation, and brought up the four mandatory Scopes (`ω_m`, `i_q`, `T_e`, and the `α-β` flux trajectory — which should be circular; a hexagonal trajectory would signal 8-state switching-table contamination, a known failure mode of naive Takahashi tables on PMSMs). The user reviewed all four, signed off with "the basic control is working," and only then was the promotion redone.

**Methodology insight**: This case sharpened **Phase 8 PASS into a 4-Gate hard sequence** — Gate 1 numerical, Gate 2 reverse-leak audit, Gate 3 documentation precision, Gate 4 user visual review at the MATLAB desktop. The Gate 4 → Phase 9 wording was strengthened: "Phase 8 4-gate sequence has fully passed **and** the user has explicitly authorized promotion." Earlier wording used "Phase 8 passes" as a single test, which was under-specified and silently included only Gates 1 – 3. Fresh-subagent numerical PASS and user terminal review are different acceptance layers; one cannot substitute for the other.

### Case 3 — Goto-Block Silent Failure and Round-Chain Invalidation (`N = 3` validation)

**Domain**: Super-twisting sliding-mode-control speed loop + PD-type sliding surface, with a cross-decoupling-feedforward dq PI inner-current loop, on a PMSM driven by SVPWM through an averaged-inverter model.

**Initial assumption**: The Round 1 baseline used a discontinuous `sgn` reaching law and was expected to fail by design with chattering. Rounds 2 (`sat` boundary-layer) and 3 (super-twisting) were planned to reduce chattering progressively, building the skill's reverse-correction pedagogy.

**Observed failure**: Across 18 experiments spanning sgn / sat / super-twisting / hand-built PI vs Discrete PID variants / `ω_c` sweep / `V_dc` sweep, the chattering persisted with `i_q,std ≈ 32.8`. The hypothesis space drifted toward the closed-loop PI inner loop being the chattering source, prompting candidate architectural redesigns (ideal-dq voltage source bypassing inverter + SVPWM, continuous `powergui`, non-linear inner controllers).

**Root cause (found late)**: The Simulink `Goto` block providing the electrical angle `θ_e` to the inverse-Park transform inside an `Anti_Park` subsystem was configured with the default `TagVisibility = 'local'`. The matching `From` block inside `Anti_Park` (a one-level-deeper subsystem) therefore silently saw nothing and was reading zero. The inverse-Park transform was being computed with a constant electrical angle, degrading the entire FOC pipeline to an open-loop lab-frame inverse-Park. The hypothesized chattering source — the inner PI — was a red herring; the real issue was that the inner PI was driving a current loop whose `dq` reference frame was frozen.

**Single-line fix**: `set_param('<path>/Goto_The', 'TagVisibility', 'global')`. After the fix, `i_q,std` dropped from `32.8` to `0.250` — two orders of magnitude. Additional supporting fixes: a dedicated `Gain_Pn_omega` block to feed `ω_e` (not `θ_e`) into the cross-decoupling feedforward; raising `V_dc` from `300 V` to `500 V` to give `V_max / BEMF ≈ 1.89×` margin (well above the `1.5×` minimum-headroom rule).

**Decision**: The fixed super-twisting + PD-type sliding controller (with the `Goto` visibility fix and the elevated `V_dc`) was promoted directly as the `v1.0` baseline, **bypassing the planned sgn → sat → super-twisting reverse-correction round chain**. The 18 invalidated experiments are retained in `DEVELOPMENT.md` as a documented history-of-error.

**Methodology insights**: This case introduced or sharpened several rules:

1. **Phase 6 Layer 2 and Phase 8 G4 Visual 4-Check**: previous workflow versions had numerical-only acceptance metrics, which produced plausible values on a broken implementation (`i_q,std = 32.8` numerically scores like a "real chattering pattern" even though the underlying FOC is open-loop). The `abc` DC-locked signature would have intercepted the first experiment.
2. **Phase 5 Sub-condition: Round-Chain Bypass on Historical Pollution**: when Round-1 FAIL is caused by an implementation bug (rather than the control law itself), running rounds 2 / 3 on top of the broken implementation generates spurious data that contaminates the reverse-correction pedagogy. The patched build is promoted directly; the round chain becomes history-of-error in `DEVELOPMENT.md`.
3. **Phase 5 acceptance #5 Round-1 expected FAIL formalization**: the SKILL.md must explicitly state which Mode-B metric a deliberate Round-1 baseline is expected to fail (with the specific metric and threshold), so that an *unexpected* FAIL is unambiguously an implementation bug, not the expected pedagogical FAIL.
4. **Appendix F §9 Reference-Locked Physical Isolation**: during the long misdiagnosis, the main session at one point read a reference `.slx` for comparison, breaching the anti-contamination discipline. Subsequent rebuilds required physical relocation of the reference (`mv ~/Desktop/<reference>.slx`), README annotation, and a fresh main session before continuing.

---

## Appendix D — Operation-Log Integrity

### The Core Problem

LLM agents (Claude included) writing operation logs carry a systematic bias:

- When writing "I have done X," the agent tends to describe **intent** ("I planned to do this") rather than **fact** ("I did X, and dump output proves Y").
- This is not deliberate misreporting; it is **narration-verification decoupling** — the sense of completion is not the same as the state change having occurred, and neither is the same as that state change having been independently verified.
- Risk: a future session reading "X has been done" trusts the claim and skips verification, until a few sessions later an unrelated task surfaces the true state.

### Four Hard Integrity Rules

Apply to **all** Phases, not just Phase A.2.

#### Rule 1 — State-changing operations must come with verify output

Writing "set `Rs = Rs` placeholder" must be followed immediately by:

```
verify dump:
  Resistance = Rs  ✅
```

The verify output may be a console screenshot, a dump-file path, or a `grep` command's output. **Not acceptable**: intent-style claims like "I've already set it" or "should be in effect."

#### Rule 2 — Bulk operations must enumerate every object

Writing "all Simscape Electrical blocks have been placeholder-ized" must list **every** block's processing result:

```
- DC_Voltage_Source.Amplitude = Vdc_val  ✅
- PMSM.Resistance = Rs  ✅
- PMSM.dqInductances = [Ld Lq]  ✅
- ... (every block enumerated)
- UB.converterType = Inverter  ✅
```

**Not acceptable**: summary phrasing like "PMSM / UB / RateLimiter mask placeholders substituted" — that summary-style claim is exactly the failure mode the rule exists to prevent.

#### Rule 3 — Intent claims must be explicitly tagged

If an operation is only a **plan** or **intent** (not yet executed), tag it:

```
[TODO] Plan to placeholder-ize PMSM mask (pending execution)
```

**Not acceptable**: writing a TODO as "done" before doing it.

#### Rule 4 — Cross-session references must be reproducible

Writing "session-NNN has completed Phase A.2" implies a future session can **reproduce the verify programmatically** (re-run dump / re-run grep) based on that claim. If reproduction fails → the original record is **historical-falsification evidence** and must be flagged in the current session as `historical_falsification_detected_at: <date>`, triggering a reverse-correction.

### Per-Phase Enforcement

| Phase | Integrity hard requirement |
|---|---|
| Phase A deep-read | Each of the six actions records dump output in `deep_dive.md` (mask, `find_system` list, `sfroot.find`, etc.) |
| Phase A.2 building-blocks library | The four-acceptance hard checks above |
| Phase A.3 API know-how | Each API fact carries "R2024b verified pass / fail" evidence — not memory |
| Phase B summary | Confidentiality grep is an automated scan script + zero-hit dump screenshot |
| Phase 5 subagent rebuild | The gaps section of `rebuild_summary.md` must say "I tried X; the error was Y," not "X probably doesn't work" |
| Phase 6 behavioral diff | Hit rate must come with an overlay PNG and the computation script — not "looks close enough" |

### Anti-Pattern Quick Reference

| ❌ Anti-pattern | ✅ Correct pattern |
|---|---|
| "Placeholder-ized" | "Set `set_param` on 12 PMSM mask fields; dump verified zero reference-specific values (see `verify_<date>.txt`)" |
| "Should be fine" | "Verified in simulation; output = ..." |
| "I'm planning to do this" | "[TODO] pending execution" |
| "Same as the reference" | "Hit rate X % (see `comparison.png`)" |
| "Other blocks handled the same way" | Enumerate every block |

### Meta: Why Operation-Log Integrity Belongs in the Methodology

The reliability of the methodology depends on the trustworthiness of each Phase's output. If any Phase output is intent-style claim disguised as fact, the entire workflow collapses — downstream Phases build on a wrong premise, and by the time a failure surfaces several sessions of work depend on the incorrect record. Phase 10 (methodology self-update) exists to detect this kind of collapse — but Phase 10 only works if integrity holds in the present, otherwise the self-update sees fabricated data too.

**Operation-log integrity is the methodology's immune system.**

A documented case prompted this appendix: an operation log claimed "mask placeholders applied," but a follow-up programmatic probe by a different session revealed only 1 / 12 fields had actually been substituted. Reference-specific numerical values had leaked into a subsequent fresh-subagent verification. The discipline above formalizes the corrective so the same failure cannot recur silently.

---

## Appendix E — Main-Session Structural-Diff SOP

> **Trigger**: Phase 6 Layer 1 acceptance. After Phase 5 subagent delivers the four-piece artifact and the self-contained-model verification passes, the main session must run the structural diff before the behavioral overlay.
> **Core rule**: **The user must review**. Claude cannot self-pass and cannot "the differences look small so I'll skip." Every diff entry must be classified by the user.

### Step 1 — Programmatic dump of the reference's structure

**Location**: `sessions/session-NNN/<reference>_dump.md` (session-local).

**Dump contents** (granularity: fully expanded + every signal line + every mask field):
1. All top-level blocks (`find_system(mdl, 'SearchDepth', 1, 'Type', 'Block')`).
2. One-level expansion of every subsystem (`find_system(blkpath, 'SearchDepth', 1, 'Type', 'Block')`).
3. Every block's mask fields (`get_param(blk, 'DialogParameters')` then `get_param(blk, field)` per field).
4. Every signal line's connections (`get_param(blk, 'PortHandles')` → `get_param(port, 'Line')` → `get_param(line, 'SrcBlockHandle' / 'DstBlockHandle')`).
5. Model properties (`get_param(mdl, 'StopTime' / 'Solver' / 'SolverType' / 'InitFcn')`).
6. Stateflow chart `Script` if any (`sfroot().find('-isa', 'Stateflow.EMChart')`).

### Step 2 — Programmatic dump of the rebuild structure

Same as Step 1, output to `sessions/session-NNN/<rebuild>_dump.md`.

### Step 3 — Diff table

**Location**: `sessions/session-NNN/<reference>_vs_<rebuild>_diff.md` (**session-local, not committed to git** — contains reference-specific topology).

**Structure**: one row per difference, five columns:

| Dimension | Reference | Rebuild | Severity | User classification |
|---|---|---|---|---|
| block name / mask field / line | value / config | value / config | CRIT / HIGH / MED / LOW | 🟢 / 🟡 / 🔴 / pending |

### Step 4 — Item-by-item classification with the user

**Claude must not auto-classify.** The main session presents each diff entry to the user; the user picks one of four categories:

- 🟢 **Subagent implementation bug** — the subagent got it wrong (used the wrong block, reversed a connection direction). Reverse-revise `understanding.md` to add the constraint.
- 🟡 **Methodology gap in `understanding.md`** — `understanding.md` did not specify the rule (e.g., did not require `InitFcn` injection of PI gains). Reverse-revise.
- 🟡 **R2024b platform-API fact** — the difference stems from an API knowledge gap. Reverse-revise `api_notes.md`.
- 🔴 **Reasonable design choice by the subagent** — the choice differs from the reference but is reasonable (layout direction, naming style, equivalent implementation). Do not revise documentation; preserve the subagent's design autonomy.

**Anti-patterns to avoid**:
- ❌ Claude pre-fills every difference as 🟡 → `understanding.md` collapses into a copy of the reference, destroying generalization.
- ❌ Claude pre-fills every difference as 🔴 → `understanding.md` never improves, and the next subagent makes the same mistakes.
- ✅ The user judges per entry: "is this a teaching point + will skill reuse need it?"

### Step 5 — Reverse-revise `understanding.md` / `api_notes.md` per user classification

Only 🟢 / 🟡 entries trigger revisions. 🔴 entries do not.
Revisions must respect the Three-Tier Shareability Classification: topology detail never enters `understanding.md`; only "principled constraint + design decision" enters.

### Step 6 — Re-run Phase 5 round `N+1` verification

The subagent rebuilds with the updated `understanding.md`. Return to Step 1 of this appendix and re-diff to check convergence.
**Convergence criterion**: previously 🟢 / 🟡-classified entries have all been eliminated this round (or reduced to ≤ 2 LOW-severity entries).

### Known pitfalls

- **Confidentiality leak risk**: the diff table contains the reference's full topology. **It must never be shown to a subagent** (violates the isolation discipline). Main-session and user only.
- **Claude tends to auto-classify**: when prompting Claude, explicitly leave the classification column **empty** — do not pre-fill suggestions (pre-filling is itself a form of priming).
- **Long diff tables cause review fatigue**: the user attends to the first 5 entries and drifts on the next 50. Suggest sorting by severity (CRIT first) and discussing ≤ 10 entries per session.

---

## Appendix F — Anti-Contamination Discipline: Complete SOP ⭐

> **Position in the methodology**: Appendix F is the operational specification for the seventh-listed Core Principle (rule #6, Anti-Contamination). It is also the methodology's primary publication contribution.
> **When to read**: every spawn of a fresh-subagent verification (Phase 5 rebuild, Phase 8 generalization audit) must read this appendix and follow it.
> **Prior-art positioning**: Existing literature on agent-skill governance focuses on permissions and provenance ([arXiv 2602.12430](https://arxiv.org/abs/2602.12430)), not on build-time anti-cheating verification. Existing official skill documentation ([matlab/skills](https://github.com/matlab/skills), [Anthropic skill docs](https://code.claude.com/docs/en/skills)) defers verification to a one-line "test thoroughly." The SOP below addresses the build-time isolation gap with a formal protocol.

### §1. Statement of the Core Problem

When a fresh subagent verifies a skill, anything the subagent can read from the reference model, prior session memory, or earlier verification reports lets it "cheat" the rebuild — it can reproduce a working-looking implementation that secretly draws on what it shouldn't have read. The resulting skill verification is hollow.

Concrete cheating modes:

- **Direct copy-paste**: copying wiring, parameter values, or algorithm code from the reference's build script.
- **Structural mimicry**: not copying values, but copying topology / naming / bus structure.
- **Magic-number reproduction**: copying a PI gain or `λ` value from prior session memory that "looks right," bypassing derivation.
- **Failure-path memorization**: knowing from the previous round's FAIL report that "the `v0.3` approach FAILed here," then avoiding it without fixing the root cause.
- **Documentation osmosis**: reading the reference's deep-dive document, internalizing implementation choices, then quietly reproducing them in the skill.

**Anti-contamination discipline = a main-session-enforced must-NOT-read list, an audit protocol when issues are suspected, and a role-transition exception clause.**

### §2. Anti-Contamination vs Anti-Cheating (rule #6 vs rule #2)

| Dimension | Rule #2 — Anti-cheating (foundational) | Rule #6 — Anti-contamination |
|---|---|---|
| Scope | "During the rebuild, do not peek at the reference" — single rule | Systematic discipline: three-layer must-NOT-read template + main-session audit + role-transition + failure budget |
| Depth | Conceptual statement | Operational SOP (the same template every spawn) |
| Implementation | Implicit (a few lines in the subagent prompt) | Explicit (every spawn has an audit trail; the main session runs `grep`) |
| Verification | Trust the subagent's self-report | Main session **verifies objectively** (`grep` / mathematical consistency / structural consistency) |

Rule #6 is the engineering build-out of rule #2, not a replacement. Both coexist.

### §3. Three-Layer Must-NOT-Read Template

Every fresh-subagent spawn must include the following three layers in its prompt's "must-NOT-read" section (substituting `<project>` / `<method>` etc.):

#### Layer 1 — Project-confidential (the foundational confidentiality boundary)

- `<project>/methods/<method>/reference_study/**` — Phase A deep-dive and understanding documents.
- `<project>/shared/reference_models/**` — reference model `.slx` originals and accompanying `.m`.
- Any `*_deep_dive.md`, `*_dump.md`, `*_vs_*_diff.md` — diff tables contain full topology and are confidential at the level of the reference model itself.

#### Layer 2 — Historical contamination (the cheat-source layer)

- `<project>/sessions/**` — project session logs (heavy contamination source).
- `<project>/methods/<method>/rebuild_round*/**` — previous rebuild iterations.
- `<project>/methods/<method>/phase8_run/session_*_run/**` — previous verification working directories (FAIL audit trail + main-session reasoning chain).
- Any `PHASE_REPORT*.md` — verification stage reports.
- `<project>/methods/<method>/phase8_run/HANDOVER.md`, `build_*.m`, `*_log.txt` — historical build / handover artifacts.
- Any `<method>_deep_dive.md` historical snapshot.

#### Layer 3 — Meta-contamination (workflow layer)

- `<project>/methods/<method>/skill_draft/leak_audit.md` (if it exists — the meta-document is self-prohibited).
- Do not run `/motor-control` or any equivalent project-init slash command (auto-loads `sessions/<latest>/session-memory.md`).
- Do not `cd` to `<project>` and run `git log -p` — diff patches contain historical source which may include confidential material.
- Do not run wildcard sweeps like `find <project> -name "*.md" | xargs cat` that bypass the layered exclusion.

#### Standard acknowledgement template

Every spawn must post four confirmation lines before executing:

1. "I have read the listed must-read files (count + list)."
2. "I understand the must-NOT-read list (3 layers, X categories total)."
3. "I will not open any file in the categorized exclusions, even if cited from must-read files."
4. "If unsure whether a path falls under exclusion, I will skip + escalate to main session."

### §4. Main-Session Audit Protocol

After the spawn completes and before user acceptance, the main session must run **three objective audits** (not trust the subagent's self-report):

#### §4.1 Grep audit

Against the subagent's output (`.m` scripts, `.slx` text dumps, `.md` reports), `grep` for contamination signatures:

```bash
# Layer 1 — reference filenames / author / specific parameter values
grep -rE "<reference_model_filename>|<author_marker>|<reference_specific_constants>" subagent_output/

# Layer 2 — session IDs / prior verification artifact names
grep -rE "session_._run|build_C|build_S|build_AB|HANDOVER|PHASE_REPORT" subagent_output/

# Layer 3 — workflow leak (forbidden slash-skill, forbidden file references)
grep -rE "/motor-control|leak_audit|deep_dive\.md|reference_models/" subagent_output/
```

**Acceptance**: each grep line should return zero hits. Any hit → fresh-context invalidated → respawn (see §6 failure budget).

#### §4.2 Mathematical-consistency audit

Every numerical value the subagent uses (PI gains, controller parameters, thresholds, time constants, cost weights) should be **derivable from the skill's formulas plus the plant parameters the user provided**. Identical inputs producing identical outputs is not contamination — it is mathematics. Contamination shows up as **magic numbers that the skill formulas alone cannot produce** — those values came from prior session memory or the reference, not from the skill.

**Acceptance**: each numerical value can be expressed as `value = formula(skill_inputs, plant_params)`, with `formula` traceable to SKILL.md or `scripts/<helper>.m`.

#### §4.3 Structural-consistency audit

`diff subagent_build_script.m methods/<m>/skill_draft/scripts/build_template.m` — the difference should be **concentrated** in:

- The parameter block (placeholders `NaN` → real values).
- Model name, output filenames.
- The header comment block.

The difference should **not** show up in:

- Skeleton (block list / wiring sequence).
- Chart `Script` (algorithm code).
- Self-test section.
- `InitFcn` injection logic.

**Acceptance**: `grep` the diff hunks; the skeleton portion shows zero differences. Differences in the skeleton are evidence of contamination.

### §5. Role-Transition Protocol

> See Phase 8 Refinement Loop on FAIL for the in-flow description. This section lists the SOP checklist.

**Trigger conditions** (all must hold):
- [ ] Build phase is complete (Phase 5 + Phase 6 finished).
- [ ] The skill failed a verification round and requires post-build review.
- [ ] The user **explicitly** authorizes the transition (visible verbatim in the conversation — not implied).

**Mandatory read-log audit table** (must be recorded in the verification report's §10):

| File path | Time | Purpose | Range read |
|---|---|---|---|
| ... | HH:MM | ... | `get_param` field / `grep` result / line range |

**Constraints preserved after the transition**:
- [ ] The current session can **no longer** perform a fresh-subagent verification (it is contaminated).
- [ ] Subsequent verifications must open a new main session (`/clear` or new window).
- [ ] Future fresh subagents remain bound by the original must-NOT-read list.
- [ ] Any skill modification after the transition increments the verification-round counter and updates the changelog.

### §6. Failure Budget (coupled with the Phase 8 Refinement Loop)

| Cumulative consecutive FAILs | Main-session action |
|---|---|
| 1 | Fix + bump `v0.x → v0.(x+1)` + spawn the next |
| 2 in a row | Same + alert (is the patch addressing the surface, not the root?) |
| **3 in a row** | **Do not spawn.** Trigger a main-session role-transition review. |
| **5 in a row** | **Suspend all spawns.** Audit the methodology itself (Phase 10 trigger). |

### §7. Validated Application

The full SOP above was first validated against the case described in Appendix C, Case 1, where four consecutive FAIL-revise rounds produced three distinct root causes before reaching a stable v1.0. Audit-trail integrity through that case:

- Each verification round produced a numbered `PHASE_REPORT_*.md` documenting the FAIL diagnosis (or the PASS confirmation).
- A `grep` audit of the final v1.0 build script returned zero contamination signatures.
- The single role-transition during the case (between FAIL round 3 and the v0.3 → v0.4 fix) followed the §5 checklist; the read-log audit table was committed to the verification report's §10.

Two further cases (Appendix C, Cases 2 and 3) exercised additional dimensions of the SOP — Gate-4 user-review enforcement (Case 2), and physical reference isolation following a contamination breach (Case 3, see §9 below).

### §8. Publication Position (Defensible Claims)

Based on the §3 – §7 SOP, the following claims are defensible against existing literature:

1. **Three-layer must-NOT-read template** as a build-time anti-cheating mechanism for LLM-skill verification — no prior published formalization.
2. **Main-session triple audit** (`grep` / mathematical / structural) as objective verification of subagent isolation — no prior equivalent.
3. **Role-transition protocol** as an explicit exception clause to the anti-contamination discipline — practice exists informally; formal SOP is novel.
4. **Failure-budget ladder** + **methodology self-update** (Phase 10) as a lifecycle model for LLM-skill development.

**Positioning relative to existing work**: [arXiv 2602.12430](https://arxiv.org/abs/2602.12430) (Agent Skills) proposes a 4-tier governance framework focused on permissions and provenance; the SOP in this Appendix focuses instead on **build-time verification via isolated rebuild**. The two are complementary, not competing.

**Defensible paper-title candidate**: *Reference-Model Skill Distillation with Adversarial Subagent Verification: A Methodology for AI-Assisted Control-Engineering Simulation*.

### §9. Reference-Locked Physical Isolation ⭐

**Trigger**: A case where the reference must not be read but the main session has already learned something about it (e.g., Phase 1.5 read literature anchors but not the reference's `.slx`; later cross-family generalization requires that this case be conducted without any reference contamination); or a case where the reference exists but the user has not yet authorized unlocking.

**Why documentation alone is insufficient**: README annotations and must-NOT-read lists are documentation-level constraints. When the main session faces a temptation to "just glance at the formula, not the implementation," documentation-level prohibitions can be silently rationalized away. Physical isolation puts the file outside the reach of the `Read` tool, converting the prohibition into a hard constraint.

**Procedure**:

1. **Physically move the file out of the project tree**:
   ```bash
   mv shared/reference_models/<file>.slx ~/Desktop/<file>.slx
   ```
   Or any equivalent out-of-tree path (`~/Documents/sandbox/`, an external drive, etc.).

2. **Annotate the README with the MOVED marker** plus the new path:
   ```markdown
   ## reference_models/
   - `<file>.slx` — **MOVED to ~/Desktop/<file>.slx (YYYY-MM-DD, post-contamination-breach isolation)**
   ```

3. **Redact existing references**:
   - In invalidated skill prompts, build scripts, comments, and session memory, replace every path reference with a placeholder.
   - `grep -rE "<file>\.slx" .` across the project tree must return zero hits.

4. **A fresh main session (or fresh subagent) takes over**:
   - The current main session may still retain knowledge about the reference (contamination does not wash out). Issue `/clear` to the main session, or hand off to a fresh subagent.
   - Annotate the session-memory with the isolation timestamp + the trigger.

**Constraints**:

- Physical isolation is a **one-way operation**. If the reference must be unlocked again, the case must be re-evaluated against the publication gate; the unlock cannot be done ad hoc.
- Should a `v1.x` revision require comparing against the isolated reference, the comparison must occur in a `/clear`-ed main session under single-session authorization (Appendix F §5); the authorization does not propagate to the next spawn.

**Relation to §3 (Must-NOT-Read Template)**:

- §3 is **subagent-side** anti-contamination (explicit forbidden-read list at spawn time).
- §9 is **main-session-side** anti-contamination (the file is physically out of `Read`'s reach).
- Together: subagent isolation + main-session physical isolation = a complete anti-contamination posture at the cross-family validation gate.

**Validated application**: See Appendix C, Case 3, where the main session inadvertently read a reference `.slx` during a long misdiagnosis cycle; subsequent rebuilds required the isolation procedure above (the reference was moved to `~/Desktop/`, the README annotated, and the main session restarted) before the methodology could continue.

---

## Appendix G — Methodology vs Ad-hoc Iteration

| Dimension | Ad-hoc iteration | This methodology |
|---|---|---|
| Learning depth | On-demand (only the parameters and the scenario) | Single thorough pass (Phase A deep-read) |
| Anti-cheating | None (the agent builds while looking at the reference) | Mandatory subagent isolation (Phase 5 + Appendix F) |
| Failure exit | None (can loop into unbounded debugging) | Explicit: 2 reverse-corrections → archive after 3 |
| Skill output | Post-hoc, coarse summary | Phase 7 draft + Phase 8 generalization audit (two-round validation) |
| Generalization | Not verified | Phase 8 fresh session + novel problem |
| Confidentiality | Loose | Two-layer filter (Phase B + Phase 7) |
| Up-front time cost | Low (build immediately) | High (Phase 2 – 3 takes hours) |
| Total time cost | High (multiple debug cycles waste time) | Low (root causes intercepted early) |

---

## Appendix H — When Not to Apply This Workflow

- ❌ Rapid-exploration phase (the user has not yet decided which model to learn from).
- ❌ Original design with no reference model.
- ❌ Pure bug fixing (no new algorithm learning involved).
- ❌ Reference model whose confidentiality level forbids even a Phase A deep-read by Claude. A reference at that confidentiality level probably should not be stored on the developer's machine in the first place.
