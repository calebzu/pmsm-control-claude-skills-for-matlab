# PMSM Claude Skills — Workspace Guide

This repository is a Claude Code workspace for building MATLAB/Simulink models of
PMSM control (FOC, FCS-MPC, DTC, SMC) and for distilling reference models into
reusable skills under anti-contamination discipline.

## One-time setup

Before doing modeling work in this workspace:

1. **Install the MATLAB MCP server** so MATLAB runs directly instead of through Bash:
   - Source: <https://github.com/matlab/matlab-mcp-core-server> (MathWorks, MIT)
   - Build per its README, then register it: `claude mcp add matlab --scope user -- <path-to-binary>`
   - It provides `detect_matlab_toolboxes`, `check_matlab_code`, `evaluate_matlab_code`, `run_matlab_file`.
2. **Install the MathWorks official MATLAB skills for Claude Code** per MathWorks' instructions.
3. Confirm MATLAB R2024b or later with Simulink, Simscape, Simscape Electrical, and Control System Toolbox.

If the MCP server is not available, run MATLAB through its CLI:
`matlab -batch "run('script.m')"` — and use a background process for long simulations.

## Skills

`.claude/skills/` is registered automatically when Claude Code opens this folder:

- `motor-pmsm-base` — plant + dq conventions + building-blocks SOP + broken-FOC defense. Base for the method skills.
- `motor-fcs-mpc` — single-vector Finite-Control-Set MPC current loop.
- `motor-fcs-mpc-dualvector` — dual-vector (two-vectors-per-period) FCS-MPC current loop with q-axis deadbeat time allocation. Layers on `motor-fcs-mpc`.
- `motor-fcs-mpc-trivector` — three-vector (two adjacent active + one zero per period) FCS-MPC current loop; a 2×2 dual-axis deadbeat solve nulls both `i_d` and `i_q` in one period.
- `motor-dtc-pmsm` — Direct Torque Control, αβ frame, Sutikno 6-state switching table.
- `motor-smc-pmsm` — Sliding Mode Control speed loop (PD-type sliding + super-twisting) over a dq PI current loop.
- `simulink-layout-tidy` — general-purpose Simulink layout tidier (compact, overlap-free, honest crossing report). Not motor-specific; supports the layout gate.

Each method skill layers on `motor-pmsm-base` (`motor-fcs-mpc-dualvector` additionally builds on `motor-fcs-mpc`). Read the relevant `SKILL.md` and its `references/` before building.

## Methodology

To learn an external reference model and distil it into a new skill, follow
`workflow/reference_model_learning_workflow.md` — the 11-phase Reference Model Learning Workflow.
Its non-negotiable disciplines:

- **Learn / build separation** — understanding a model and rebuilding it are verified as two separate capabilities.
- **No peeking on rebuild** — the rebuild subagent must not read the reference model; otherwise the result collapses into a copy and the skill is hollow.
- **Anti-contamination isolation** — give every verification subagent an explicit must-NOT-read list (confidential sources, prior-pass artifacts, methodology internals), and audit its output (grep / mathematical consistency / structural consistency) before trusting it.
- **Theory first** — derive the plant equations and control-law formulas before reading any `.slx`; a model that contradicts the derived formulas is a finding, not a hallucination.

## Working rules

- **Plan before building.** Write a numbered plan (parameter table, design decisions, build-script structure) and get user approval before the first `add_block`.
- **`shared/` is read-only.** Reuse `shared/formulas/` and `shared/building_blocks/` by reference; do not duplicate or re-derive. Build your models in `workspace/`.
- **One-click reproducibility.** A built `.slx` must run from a double-click in a fresh MATLAB session — inject all parameters via `set_param(mdl, 'InitFcn', ...)`.
- **Visual check before metrics.** Before trusting numerical scores, confirm the motor rotates, `iq` tracks its reference, the `abc` currents are AC sinusoids (not DC-locked), and the torque energy balance holds. A failed visual check means a broken implementation regardless of the numbers.

## How to work here

- On failure, change approach — don't repeat a method that just failed; after ~3 distinct attempts, stop and report rather than thrashing.
- Never swallow an error silently.
- On long build scripts, re-read your plan every few steps.
- Be direct: if the user's approach has a flaw, give the counter-argument with reasoning rather than going along with it.
