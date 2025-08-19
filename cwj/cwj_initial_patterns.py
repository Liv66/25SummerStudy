from typing import List, Tuple, Dict
import math
import random
from .cwj_utils import *

# -----------------------------
# Initial Pattern Generation
# -----------------------------
def generate_initial_patterns(K: int, Q: int, node_types: List[int], node_demands: List[int],
                              dist_mat: List[List[float]], seed: int = 0) -> List[Tuple[List[int], float]]:
    """
    Build a covering set of feasible routes:
    1) Pack Ls greedily (<=Q) from depot by nearest-neighbor.
    2) After last L, greedily add Bs (<=Q).
    Ensures all customers (L and B) are covered at least once by creating extra routes as needed.
    """
    random.seed(seed)
    N = len(node_types)
    Ls = [i for i in range(1, N) if node_types[i] == 1]
    Bs = [i for i in range(1, N) if node_types[i] == 2]

    unL = set(Ls)
    unB = set(Bs)
    routes: List[List[int]] = []

    # Phase A: create routes centered on linehauls
    while unL:
        # pivot: pick L closest to depot or random among k-best
        k_best = sorted(unL, key=lambda i: dist_mat[0][i])[:min(5, len(unL))]
        pivot = random.choice(k_best)
        # build L cluster
        L_pack = [pivot]
        capL = node_demands[pivot]
        cur = pivot
        cand = list(unL - {pivot})
        while cand:
            # pick nearest L that fits
            cand.sort(key=lambda j: dist_mat[cur][j])
            chosen = None
            for j in cand:
                if capL + node_demands[j] <= Q:
                    chosen = j
                    break
            if chosen is None:
                break
            L_pack.append(chosen)
            capL += node_demands[chosen]
            cur = chosen
            cand.remove(chosen)
        # order Ls NN from depot
        L_seq = nearest_neighbor_order(L_pack, 0, dist_mat)

        # Phase B: add Bs greedily from last L
        capB = 0
        cur = L_seq[-1]
        B_seq: List[int] = []
        # try to favor Bs near the L cluster tail
        candB = list(unB)
        while candB:
            candB.sort(key=lambda j: dist_mat[cur][j])
            added = False
            for j in candB:
                if capB + node_demands[j] <= Q:
                    B_seq.append(j)
                    capB += node_demands[j]
                    cur = j
                    candB.remove(j)
                    added = True
                    break
            if not added:
                break

        route = [0] + L_seq + B_seq + [0]
        if is_feasible_vrpb(route, node_types, node_demands, Q):
            routes.append(route)
            for i in L_seq:
                unL.discard(i)
            for j in B_seq:
                unB.discard(j)
        else:
            # fallback minimal L-only route
            route = [0] + L_seq + [0]
            if is_feasible_vrpb(route, node_types, node_demands, Q):
                routes.append(route)
                for i in L_seq:
                    unL.discard(i)

    # Phase C: cover remaining Bs by pairing with small L packs
    # Find small Ls near each remaining B
    for b in list(unB):
        # choose small L set (nearest few) that fits and makes a feasible route
        nearLs = sorted([i for i in range(1, N) if node_types[i] == 1],
                        key=lambda i: dist_mat[b][i])[:5]
        built = False
        for l in nearLs:
            if node_demands[l] <= Q and node_demands[b] <= Q:
                route = [0, l, b, 0]
                if is_feasible_vrpb(route, node_types, node_demands, Q):
                    routes.append(route)
                    unB.discard(b)
                    built = True
                    break
        if not built:
            # last resort: attach b to the closest existing route that has B capacity
            best_idx = None
            best_increase = math.inf
            for idx, r in enumerate(routes):
                # compute current B load
                curB = sum(node_demands[v] for v in r if node_types[v] == 2)
                if curB + node_demands[b] <= Q and node_types[r[-2]] == 2:
                    # try insert before depot
                    increase = dist_mat[r[-2]][b] + dist_mat[b][0] - dist_mat[r[-2]][0]
                    if increase < best_increase:
                        best_increase = increase
                        best_idx = idx
            if best_idx is not None:
                routes[best_idx].insert(-1, b)
                unB.discard(b)
            else:
                # as a final fallback, create [0, l*, b, 0] with l* closest to depot that fits
                lstar = min([i for i in range(1, N) if node_types[i] == 1],
                            key=lambda i: dist_mat[0][i])
                route = [0, lstar, b, 0]
                if is_feasible_vrpb(route, node_types, node_demands, Q):
                    routes.append(route)
                    unB.discard(b)

    # remove duplicates and keep least-cost route for identical node-sets
    best_by_set: Dict[frozenset, Tuple[List[int], float]] = {}
    for r in routes:
        nodeset = frozenset(r[1:-1])
        c = route_cost(r, dist_mat)
        if nodeset not in best_by_set or c < best_by_set[nodeset][1]:
            best_by_set[nodeset] = (r, c)

    route_pool = [(r, c) for (r, c) in best_by_set.values()]
    route_pool.sort(key=lambda x: x[1])
    return route_pool