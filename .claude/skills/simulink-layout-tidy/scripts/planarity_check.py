#!/usr/bin/env python3
"""Planarity diagnosis for a Simulink block/line graph.

Reads the JSON written by extract_graph.m ({"nodes": [...], "edges": [[i,j],...]})
and decides whether the graph is planar. A planar graph CAN be drawn with zero
line-line crossings by moving blocks alone (L1). A non-planar graph CANNOT
(Kuratowski's theorem) — the honest floor on crossings is > 0, so zero-crossing
must never be a hard gate.

Exact planarity uses networkx.check_planarity, which also returns a Kuratowski
subgraph (a K5 or K3,3 subdivision) as proof when non-planar. If networkx is
absent, falls back to a self-contained search for an explicit K5 or K3,3
*subgraph* (which is itself a definitive non-planarity certificate — it catches
the dominant real-world pattern: N modules each fanning out to the same >=3
collectors == a literal K3,3), plus Euler's necessary condition (E <= 3V-6) as a
last-resort quick rejector. Only when none of these fire is the result reported
as INCONCLUSIVE (a K3,3/K5 *subdivision* via paths needs networkx to catch).

Usage:
    python3 planarity_check.py graph.json
Exit code 0 = planar (or inconclusive), 2 = proven non-planar.
"""

import json
import sys


def load(path):
    with open(path) as f:
        g = json.load(f)
    nodes = g.get("nodes", [])
    edges = [tuple(e) for e in g.get("edges", [])]
    return nodes, edges


def with_networkx(nodes, edges):
    import networkx as nx

    G = nx.Graph()
    G.add_nodes_from(range(len(nodes)))
    G.add_edges_from(edges)
    # counterexample=True makes the non-planar return a Kuratowski subgraph.
    is_planar, cert = nx.check_planarity(G, counterexample=True)
    if is_planar:
        return True, None
    kura_edges = list(cert.edges())
    return False, kura_edges


def fallback(nodes, edges):
    """networkx-free non-planarity check.

    Returns (planar, witness):
      (False, ('K3,3'|'K5', [node indices]))  proven non-planar by subgraph
      (False, None)                            proven non-planar by Euler bound
      (None, None)                             inconclusive (need networkx)
    Never returns True: a subgraph search can prove non-planarity but cannot
    prove planarity.
    """
    from itertools import combinations

    V, E = len(nodes), len(edges)
    adj = [set() for _ in range(V)]
    for a, b in edges:
        if a != b:
            adj[a].add(b)
            adj[b].add(a)

    # K3,3 subgraph via common-neighbour: any 3 nodes sharing >=3 common
    # neighbours form a complete bipartite K3,3. C(V,3) * set-intersection,
    # cheap for the block counts Simulink diagrams realistically have.
    if V <= 200:
        for a, b, c in combinations(range(V), 3):
            common = (adj[a] & adj[b] & adj[c]) - {a, b, c}
            if len(common) >= 3:
                x, y, z = sorted(common)[:3]
                return False, ("K3,3", [a, b, c, x, y, z])

    # K5 subgraph: 5 mutually adjacent nodes. Capped — combinatorially heavier
    # and rare in block diagrams, where fan-in yields K3,3 not K5.
    if V <= 60:
        for combo in combinations(range(V), 5):
            if all(j in adj[i] for i, j in combinations(combo, 2)):
                return False, ("K5", list(combo))

    # Euler necessary condition E <= 3V-6: a cheap last-resort rejector.
    if V >= 3 and E > 3 * V - 6:
        return False, None

    return None, None  # genuinely undecided without networkx


def name(nodes, i):
    n = nodes[i]
    return n.rsplit("/", 1)[-1] if isinstance(n, str) else str(i)


def main():
    if len(sys.argv) < 2:
        print("usage: planarity_check.py graph.json", file=sys.stderr)
        return 1
    nodes, edges = load(sys.argv[1])
    print(f"graph: {len(nodes)} nodes, {len(edges)} edges")

    try:
        planar, kura = with_networkx(nodes, edges)
        backend = "networkx"
    except ImportError:
        planar, kura = fallback(nodes, edges)
        backend = "subgraph-fallback"
        print(
            "networkx not installed -> using self-contained K5/K3,3 subgraph "
            "search + Euler bound (proves non-planarity only; cannot prove "
            "planarity). For full planarity + Kuratowski subdivisions: "
            "pip install networkx"
        )

    if planar is True:
        print(
            f"PLANAR (backend={backend}): zero line-line crossings are "
            "achievable by L1 block moves alone."
        )
        return 0
    if planar is False:
        print(
            f"NON-PLANAR (backend={backend}): zero crossings are "
            "mathematically UNREACHABLE on a 2-D plane (Kuratowski). L1 "
            "moves can only reach this graph's crossing-number floor (>0). "
            "Reach true zero only via L2 (Goto/From or Subsystem) to remove "
            "the offending edges from the top-level drawing."
        )
        if (
            kura
            and isinstance(kura, tuple)
            and len(kura) == 2
            and isinstance(kura[0], str)
        ):
            kind, ns = kura
            pretty = ", ".join(name(nodes, i) for i in ns)
            print(f"{kind} witness nodes: {pretty}")
        elif kura:
            pretty = ", ".join(f"{name(nodes, a)}-{name(nodes, b)}" for a, b in kura)
            print(f"Kuratowski witness edges: {pretty}")
        return 2
    print(
        "INCONCLUSIVE: no explicit K5/K3,3 subgraph and Euler bound not "
        "violated, but a K3,3/K5 subdivision (via paths) cannot be ruled out "
        "without networkx. pip install networkx for a definite answer; until "
        "then do NOT claim the graph is planar / zero-crossing achievable."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
