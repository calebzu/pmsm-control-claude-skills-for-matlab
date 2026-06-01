# Layout metric algorithms

All metrics are computed on the immediate (`SearchDepth==1`) contents of a system. Position convention is `[left top right bottom]` with **y increasing downward**. Implementation lives in `scripts/layout_metrics.m`; this file is the spec.

## Key Simulink API

```matlab
blocks = find_system(sys,'SearchDepth',1,'Type','Block');       % cell of paths
pos    = get_param(b,'Position');                                % [l t r b]
lines  = find_system(sys,'SearchDepth',1,'FindAll','on','Type','Line');
pts    = get_param(lh,'Points');                                 % Nx2 polyline
src    = get_param(lh,'SrcBlockHandle');  dst = get_param(lh,'DstBlockHandle');
```

A polyline of N points yields N-1 segments. Branch lines appear as extra line handles whose trunk has `Dst/SrcBlockHandle == -1`; skip those when building the connectivity graph.

## 1. Block-block overlap count

`P` is `Nx4`, each row `[left top right bottom]`. Count unordered pairs whose interiors overlap (touching edges do **not** count):

```matlab
for i, for j>i:
  overlap if NOT (P(i,3)<=P(j,1) || P(j,3)<=P(i,1) || P(i,4)<=P(j,2) || P(j,4)<=P(i,2))
```

This is the separating-axis test on axis-aligned boxes. Always drivable to 0 by moving blocks, so it is a **hard gate**.

## 2. Line-through-block count

For each block, inset its rectangle by `margin = 2` px (so lines legitimately touching a block's ports at the boundary are not counted), then count `(line, block)` pairs where any segment intersects the inset rectangle's interior. Segment-vs-rectangle = "either endpoint strictly inside" OR "segment properly crosses any of the 4 rectangle edges" (using the crossing test below). Also a **hard gate** (or tiny threshold).

## 3. Line-line proper crossing count

Split every line into segments tagged by owner line id. For two segments with **different** owners, count a crossing only when they *properly* cross — strictly through, not merely sharing an endpoint (branch fan-out) and not collinear. Cross-product orientation with `eps = 1e-6` tolerance:

```matlab
cross_o(a,b,c) = (b1-a1)*(c2-a2) - (b2-a2)*(c1-a1);   % orientation of c vs a->b
d1=cross_o(p3,p4,p1); d2=cross_o(p3,p4,p2);
d3=cross_o(p1,p2,p3); d4=cross_o(p1,p2,p4);
proper crossing  iff  (d1,d2 strictly opposite sign) AND (d3,d4 strictly opposite sign)
```

The strict-sign requirement with tolerance means endpoint contact and shared branch points give `d ≈ 0` and are **not** counted — so a fan-out point is never mistaken for a crossing. This is a **soft, reported** metric; see `gates.md` for why it must never be hard-gated.

## 4. Extent / compactness

`extent = [max(right)-min(left), max(bottom)-min(top)]`. A smaller extent for the same block count = more compact. Report before→after; do not gate (compactness trades off against readability).

## 5. Planarity diagnosis (`extract_graph.m` + `planarity_check.py`)

Abstract blocks → nodes, lines → edges (dedupe parallel edges to a simple graph). Decide planarity:

- **Preferred — `networkx.check_planarity(G, counterexample=True)`.** Returns `(is_planar, cert)`. When non-planar, `cert` is a Kuratowski subgraph (a `K5` or `K3,3` subdivision) — a *proof*. `counterexample=True` is required; the default returns `None` for the certificate.
- **Fallback (no networkx) — explicit K5/K3,3 subgraph search + Euler bound.** A literal `K5` or `K3,3` *subgraph* is itself a definitive non-planarity certificate, and it catches the dominant real pattern (N modules fanning out to the same ≥3 collectors == a `K3,3`). Detection: K3,3 via common-neighbour (any 3 nodes sharing ≥3 common neighbours), K5 via 5-clique enumeration (capped for cost). Then Euler's `E ≤ 3V-6` as a last-resort quick rejector. **Only when none fire is the result `INCONCLUSIVE`** — a K3,3/K5 *subdivision* (paths instead of edges) still needs networkx, so until then do NOT claim planarity. Euler alone is too weak: degree-1 nodes (terminators) inflate `V` and mask an embedded `K3,3`.

Install for full diagnosis (subdivisions too): `pip install networkx` (PEP 668 systems: use a venv or `--break-system-packages`).

### Why this matters

A typical non-planar pattern in auto-built models: **N parallel modules each fanning out to ≥2 shared collectors** (e.g. a selector + a logger) — this readily contains a `K3,3`. By Kuratowski's theorem such a graph has **crossing number > 0**: zero crossings is mathematically unreachable by moving blocks alone. Tell the user this honestly; reach true top-level zero only via L2 (Goto/From or Subsystem) — see `l2_contract.md`.
