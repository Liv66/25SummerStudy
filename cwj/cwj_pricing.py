from typing import List, Tuple, Dict
import math
import random
from cwj_utils import *
from cwj_initial_patterns import *
from cwj_rmp import *


# -----------------------------
# Pricing (Dual-guided greedy heuristic)
# -----------------------------
def pricing_greedy(duals: Dict[int, float], mu: float,
                   node_types: List[int], node_demands: List[int],
                   dist_mat: List[List[float]], Q: int,
                   restarts: int = 20) -> List[Tuple[List[int], float]]:
    """
    Build candidate routes minimizing reduced cost:
      rc = distance - sum(pi_i) - mu
    Heuristic:
      - pick seed linehaul with large dual pi
      - grow L-phase by local gain (pi_i - dist)
      - switch to B-phase, grow similarly (capacity <= Q)
      - evaluate rc, keep best if rc < -1e-6
    Returns up to a handful of good routes.
    """
    N = len(node_types)
    Ls = [i for i in range(1, N) if node_types[i] == 1]
    Bs = [i for i in range(1, N) if node_types[i] == 2]
    if not Ls:
        return []

    # rank seeds by dual (top-k)
    topL = sorted(Ls, key=lambda i: -duals.get(i, 0.0))[:min(8, len(Ls))]
    candidates: List[Tuple[List[int], float]] = []

    for _ in range(restarts):
        seed = random.choice(topL)
        # L-phase
        L_sel: List[int] = [seed]
        capL = node_demands[seed]
        cur = seed

        while True:
            best_gain = -math.inf
            best_j = None
            for j in Ls:
                if j in L_sel:
                    continue
                dj = node_demands[j]
                if capL + dj > Q:
                    continue
                # local gain: dual benefit - travel to j
                gain = duals.get(j, 0.0) - dist_mat[cur][j]
                if gain > best_gain:
                    best_gain = gain
                    best_j = j
            if best_j is None or best_gain < 0:
                break
            L_sel.append(best_j)
            capL += node_demands[best_j]
            cur = best_j

        # If no L was selected (shouldn't happen), skip
        if not L_sel:
            continue

        # Order L by NN from depot
        L_seq = nearest_neighbor_order(L_sel, 0, dist_mat)

        # B-phase
        B_sel: List[int] = []
        capB = 0
        cur = L_seq[-1]

        while True:
            best_gain = -math.inf
            best_j = None
            for j in Bs:
                if j in B_sel:
                    continue
                dj = node_demands[j]
                if capB + dj > Q:
                    continue
                gain = duals.get(j, 0.0) - dist_mat[cur][j]
                if gain > best_gain:
                    best_gain = gain
                    best_j = j
            # allow slightly negative gain to try to close loop shorter
            if best_j is None or best_gain < -0.0:
                break
            B_sel.append(best_j)
            capB += node_demands[best_j]
            cur = best_j

        route = [0] + L_seq + B_sel + [0]
        if not is_feasible_vrpb(route, node_types, node_demands, Q):
            continue

        rc = compute_reduced_cost(route, dist_mat, duals, mu)
        if rc < -1e-6:
            candidates.append((route, route_cost(route, dist_mat)))

    # Deduplicate by node set
    best_by_set: Dict[frozenset, Tuple[List[int], float]] = {}
    for r, c in candidates:
        nodeset = frozenset(r[1:-1])
        if nodeset not in best_by_set or c < best_by_set[nodeset][1]:
            best_by_set[nodeset] = (r, c)

    # return up to a few
    out = [(r, c) for (r, c) in best_by_set.values()]
    out.sort(key=lambda x: x[1])
    return out[:10]