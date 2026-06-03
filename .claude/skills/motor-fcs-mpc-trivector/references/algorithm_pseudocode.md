# Three-Vector FCS-MPC — Algorithm Pseudocode

Two inline MATLAB Function blocks: **`TVMPCC`** (the predictive controller, runs once per `Tsc`) and **`Slicer`** (the 3-segment sequencer, runs at the fast rate `Ts`). Control law = signed formulas **T2–T6** + cost (L1, §F.0) in [pmsm_formulas.md §F](../../../../shared/formulas/pmsm_formulas.md). Prediction = forward-Euler, general `(Ld,Lq)` form (SPMSM = special case `Ld=Lq=Ls`).

## Vector set + sector pairing (shared)

6 active inverter voltage vectors `u_k = (2/3)·Vdc·e^{j(k−1)π/3}`, k=1..6; `u0=u7=0`. Sector→adjacent-active pair (base `pmsm_formulas.md §B.4`, Table 1): `S=1:(1,2) 2:(2,3) 3:(3,4) 4:(4,5) 5:(5,6) 6:(6,1)`. Park each active vector to `(ud,uq)` at the measured `theta_e`:
```
ud(k) =  Re(u_k)·cos(theta_e) + Im(u_k)·sin(theta_e)
uq(k) = −Re(u_k)·sin(theta_e) + Im(u_k)·cos(theta_e)
```
Prediction coefficients (general form, §0):
```
ad = 1 − Tsc·Rs/Ld ;   bd = we·Tsc·Lq/Ld
aq = 1 − Tsc·Rs/Lq ;   bq = we·Tsc·Ld/Lq ;   ke = we·psif·Tsc/Lq
```

## `TVMPCC` — signature `[a, b, ti, tj] = tvmpcc(id_ref, iq_ref, id, iq, theta_e, we)`
(`Vdc, Rs, Ld, Lq, psif, Tsc` are sprintf'd literals == InitFcn digit-for-digit, K-CRIT.)

```
build ud(1..6), uq(1..6)                 % Park the 6 active vectors at theta_e

% --- baseline (zero-vector) slopes, computed ONCE (T2) ---
s_d0 = (1/Ld)*(−Rs*id + we*Lq*iq)        % T2(b) NEW (d-axis)
s_q0 = (1/Lq)*(−Rs*iq − we*Ld*id − we*psif)   % T2(a) reused (= dualvector N2)
A = (id_ref − id) − s_d0*Tsc             % deadbeat residual after removing zero-vector drift
B = (iq_ref − iq) − s_q0*Tsc

pairs = [1 2;2 3;3 4;4 5;5 6;6 1]        % the 6 sectors' adjacent active pairs
gbest = inf ; abest=1; bbest=2; tibest=0; tjbest=0
for S in 1..6:
    i = pairs(S,1) ; j = pairs(S,2)
    % --- dual-axis deadbeat 2x2 solve (T3) ---
    det = (ud(i)*uq(j) − ud(j)*uq(i)) / (Ld*Lq) + 1e-12      % det guard (adjacent pair never collinear)
    ti  = ( A*uq(j)/Lq − B*ud(j)/Ld ) / det                 % T3 eq23
    tj  = ( B*ud(i)/Ld − A*uq(i)/Lq ) / det                 % T3 eq24
    tz  = Tsc − ti − tj
    % --- feasibility clamp / vector-count reduction (T5, 3 cases) ---
    [ti, tj, tz] = clamp3(ti, tj, tz, Tsc)                  % drop out-of-range vector, renormalize Sum=Tsc
    % --- expected-voltage synthesis (T4①) + candidate prediction (T4② = §0) ---
    udq_d = (ud(i)*ti + ud(j)*tj) / Tsc                     % eq26 ; zero vector contributes 0 V
    udq_q = (uq(i)*ti + uq(j)*tj) / Tsc                     % eq27
    idp = ad*id + bd*iq + (Tsc/Ld)*udq_d
    iqp = aq*iq − bq*id + (Tsc/Lq)*udq_q − ke
    g   = |id_ref − idp| + |iq_ref − iqp|                   % L1 unweighted (§0.1, paper eq7)
    if g < gbest: gbest=g; abest=i; bbest=j; tibest=ti; tjbest=tj
a = abest ; b = bbest ; ti = tibest ; tj = tjbest          % active indices 1..6 ; times in seconds
```

**`clamp3(ti,tj,tz,Tsc)`** — literal reading of T5's three cases (graceful degradation, never silent collapse):
```
if ti,tj,tz all in [0,Tsc]:            return as-is                    % nominal 3-vector
elseif tz<0 (or >Tsc) and ti,tj in [0,Tsc]:  drop zero → scale ti,tj to sum Tsc   % Case 1: two actives
elseif tz in [0,Tsc] and exactly one of ti,tj in range:  keep that active + zero  % Case 2
elseif tz out and exactly one of ti,tj in range:  that active fills the period     % Case 3
else:  clamp each to [0,Tsc], renormalize so Σ=Tsc                                 % defensive
```

**Notes**
- All 6 sectors' expected vectors are evaluated unconditionally (T6b) — the search set is **6 composite candidates**, not the 8 basic vectors.
- Reset `gbest=inf` at the TOP of every call (no persistent best-cost across periods).
- The chart is **DISCRETE @ Tsc** — id/iq/theta_e sampled once per period; outputs `a,b,ti,tj` held for the period (rule 4 / [crit_conditions.md §G](crit_conditions.md)).

## `Slicer` — signature `gate = slicer(t, a, b, ti, tj)`
```
Tsc = <sprintf literal>
tbl = [0 0 0;1 0 0;1 1 0;0 1 0;0 1 1;0 0 1;1 0 1;1 1 1]   % rows u0,u1..u6,u7 ; upper [Sa Sb Sc]
tau = mod(t, Tsc)                          % in-period timer, t from a Digital Clock @ Ts
tib = min(max(ti,0),Tsc) ; tjb = min(max(tj,0),Tsc)
if     tau < tib:          idx = a + 1     % u_i  (active k → row k+1)
elseif tau < tib + tjb:    idx = b + 1     % u_j
else:                      idx = 1         % zero u0 = row 1
idx = clamp(idx, 1, 8)
s1=tbl(idx,1); s2=tbl(idx,2); s3=tbl(idx,3)
gate = [s1; 1−s1; s2; 1−s2; s3; 1−s3]      % 6-bit pair-adjacent, COLUMN vector (D-CRIT)
```

**Notes**
- `Slicer` is **DISCRETE @ Ts** so it slices three segments within the period and emits a discrete gate the SPS bridge accepts (rule 4, §G/§D). This is the three-vector generalization of the dual-vector's two-segment slicer.
- `gate` is a **column** `[6 1]`, not a row (D-CRIT / [anti_patterns.md](anti_patterns.md) #4).
- Zero vector defaults to `u0=[000]` (row 1); a loss-minimizing `u0/u7` choice is an optional refinement (design_decisions #6), not required for reproduction.

## `ThetaSrc` — signature `[theta_e, we] = theta_src(w)`
```
Pn = <literal> ; Tsc = <literal>
persistent th ; if isempty(th): th = 0
we = Pn * w
th = th + Tsc * we
th = atan2(sin(th), cos(th))               % wrap to (−pi, pi]
theta_e = th
```
**DISCRETE @ Tsc** — the persistent integrator advances by `Tsc·we` once per control period; at the fast rate it over-integrates by `Tsc/Ts`. The same `theta_e` feeds both `Plark` and the controller's vector Park.
