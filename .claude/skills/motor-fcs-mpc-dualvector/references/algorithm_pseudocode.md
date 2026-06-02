# Dual-Vector FCS-MPC — Algorithm Pseudocode

Two inline MATLAB Function blocks: **`DualVecMPC`** (the predictive controller, runs once per `Tsc`) and **`TimeSlicer`** (the two-vector sequencer, runs at the fast rate `Ts`). Control law = signed formulas **N2–N5** in [pmsm_formulas.md §E](../../../../shared/formulas/pmsm_formulas.md); cost = L1 unweighted (paper eq 7), implemented in the Stage 1/2 pseudocode below (not in the signed formula doc). Prediction = forward-Euler, general `(Ld,Lq)` form (SPMSM = special case `Ld=Lq=Ls`).

## Vector set (shared by both stages)

8 inverter voltage vectors `V_k = (2/3)·Vdc·e^{j(k−1)π/3}`, k=1..6 active; `V0=V7=0`. Index map used internally: `1→V0, 2..7→V1..V6` (V7 dropped as a duplicate of V0 → 7 distinct candidates). Park each to `(ud,uq)` at the measured `theta_e`:
```
ud(k) =  Re(V_k)·cos(theta_e) + Im(V_k)·sin(theta_e)
uq(k) = −Re(V_k)·sin(theta_e) + Im(V_k)·cos(theta_e)
```

Prediction coefficients (general form, N4 / §0):
```
ad = 1 − Tsc·Rs/Ld ;   bd = we·Tsc·Lq/Ld
aq = 1 − Tsc·Rs/Lq ;   bq = we·Tsc·Ld/Lq ;   eq = we·psif·Tsc/Lq
```

## `DualVecMPC`  —  signature `[a, b, Top] = dual_vec_mpc(id_ref, iq_ref, id, iq, theta_e, Vdc, we, Tsc)`

```
% K-CRIT: Rs, Ld, Lq, psif are sprintf'd literals == InitFcn digit-for-digit.

build ud(1..8), uq(1..8)            % Park all vectors at theta_e (V0,V7 = 0)
cand = [1 2 3 4 5 6 7]              % 7 distinct candidates

% ---------- STAGE 1: pick first optimal vector V_opt1 over the whole Tsc ----------
g1 = inf ; iopt1 = 1
for k in cand:
    idp = ad*id + bd*iq + (Tsc/Ld)*ud(k)                 % single-vector full-period predict
    iqp = aq*iq − bq*id + (Tsc/Lq)*uq(k) − eq
    g   = |id_ref − idp| + |iq_ref − iqp|                % L1 unweighted cost (paper eq 7)
    if g < g1: g1 = g ; iopt1 = k

% q-axis slopes (N2)
s0    = (1/Lq)*(−Rs*iq − we*Ld*id − we*psif)             % baseline (zero-q-voltage) slope
sopt1 = s0 + uq(iopt1)/Lq

% ---------- STAGE 2: sweep second vector V_j, deadbeat time split ----------
g2 = inf ; jbest = 1 ; Topb = Tsc
for j in cand:
    sj  = s0 + uq(j)/Lq
    Top = (iq_ref − iq − sj*Tsc) / (sopt1 − sj + 1e-10)   % N3 q-deadbeat time alloc (eq 15) + guard
    Top = clamp(Top, 0, Tsc)                              % N5 on-time clamp
    udavg = (ud(iopt1)*Top + ud(j)*(Tsc − Top)) / Tsc     % N4 duty-weighted AVERAGE voltage (÷Tsc)
    uqavg = (uq(iopt1)*Top + uq(j)*(Tsc − Top)) / Tsc
    idp = ad*id + bd*iq + (Tsc/Ld)*udavg                  % two-vector predicted current
    iqp = aq*iq − bq*id + (Tsc/Lq)*uqavg − eq
    g   = |id_ref − idp| + |iq_ref − iqp|                 % L1 unweighted cost
    if g < g2: g2 = g ; jbest = j ; Topb = Top

a = iopt1 ; b = jbest ; Top = Topb                        % indices 1..8 (1=V0, 2..7=V1..V6, 8=V7)
```

**Notes**
- The two-stage `7+7` search (fix `V_opt1`, then sweep `V_j`) replaces the full `7×7=49` pair search — an efficiency choice that does not change the selected pair in practice (a vector that is sub-optimal alone is rarely a useful first vector). Documented in [design_decisions.md](design_decisions.md) #4.
- Reset `g1=inf, g2=inf` at the TOP of every call (no persistent best-cost across periods).
- The chart is **DISCRETE @ Tsc** — its `id/iq/theta_e` inputs are thereby sampled once per control period; outputs `a,b,Top` are held for the period (rule 4 / [crit_conditions.md §G](crit_conditions.md)).

## `TimeSlicer`  —  signature `gate = time_slicer(a, b, Top, t)`

```
Tsc = <sprintf literal>
state = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 1 1; 0 0 1; 1 0 1; 1 1 1]   % rows V0..V7, upper-switch [Sa Sb Sc]
tau = mod(t, Tsc)                          % in-period timer, t from a Digital Clock @ Ts
idx = (tau < Top) ? a : b                  % V_opt1 for Top, then V_j for (Tsc − Top)
idx = clamp(idx, 1, 8)
sa = state(idx,1) ; sb = state(idx,2) ; sc = state(idx,3)
gate = [sa; 1−sa; sb; 1−sb; sc; 1−sc]      % 6-bit pair-adjacent, COLUMN vector (D-CRIT)
```

**Notes**
- `TimeSlicer` is **DISCRETE @ Ts** (the fast plant rate) so it can slice within the period and emit a discrete gate the SimPowerSystems bridge accepts (rule 4, §G/§D).
- `gate` is a **column** `[6 1]`, not a row `[1 6]` — matches the Universal_Bridge gate port `[6]` (D-CRIT / [anti_patterns.md](anti_patterns.md) #3).
- Pair-adjacent order `[Sa+ Sa− Sb+ Sb− Sc+ Sc−]`, upper/lower complementary — matches the library `Universal_Bridge` gate convention (`pmsm_blocks_manifest.md`).

## `ThetaSrc`  —  signature `[theta_e, we] = theta_src(w)`

```
Pn = <literal> ; Tsc = <literal>
persistent th ; if isempty(th): th = 0
we = Pn * w
th = th + Tsc * we
th = atan2(sin(th), cos(th))               % wrap to (−pi, pi]
theta_e = th
```

**DISCRETE @ Tsc** — the persistent integrator advances by `Tsc·we` once per control period; running it at the fast rate over-integrates `theta_e` by the `Tsc/Ts` factor (the original gap-1 bug). The same `theta_e` feeds both `Plark` (measurement) and the controller's vector Park.
