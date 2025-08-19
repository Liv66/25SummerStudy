from typing import List, Tuple, Dict
import gurobipy as gp
from gurobipy import GRB

# -----------------------------
# Utility
# -----------------------------
def route_cost(route: List[int], dist_mat: List[List[float]]) -> float:
    """Sum of distances along route [0,...,0]."""
    return sum(dist_mat[route[i]][route[i+1]] for i in range(len(route)-1))

def is_feasible_vrpb(route: List[int], node_types: List[int], node_demands: List[int], Q: int) -> bool:
    """Check VRPB feasibility: start at 0, end at 0, Ls first then Bs, capacity on both L/B segments."""
    if route[0] != 0 or route[-1] != 0 or len(route) < 3:
        return False
    seen_backhaul = False
    DL = 0
    DB = 0
    visited = set()
    for v in route[1:-1]:
        if v in visited:  # avoid duplicates
            return False
        visited.add(v)
        if node_types[v] == 1:  # L
            if seen_backhaul:
                return False
            DL += node_demands[v]
            if DL > Q:
                return False
        elif node_types[v] == 2:  # B
            seen_backhaul = True
            DB += node_demands[v]
            if DB > Q:
                return False
        else:
            return False
    # no B-only route: must include at least one L
    if all(node_types[v] != 1 for v in route[1:-1]):
        return False
    return True

def nearest_neighbor_order(nodes: List[int], start: int, dist_mat: List[List[float]]) -> List[int]:
    """Simple NN order for TSP-like ordering."""
    if not nodes:
        return []
    rem = set(nodes)
    cur = start
    seq = []
    while rem:
        nxt = min(rem, key=lambda j: dist_mat[cur][j])
        seq.append(nxt)
        rem.remove(nxt)
        cur = nxt
    return seq

def compute_reduced_cost(route: List[int], dist_mat, duals: Dict[int, float], mu: float) -> float:
    """rc = cost - sum(pi_i) - mu"""
    c = route_cost(route, dist_mat)
    pi_sum = sum(duals.get(i, 0.0) for i in route if i != 0)
    return c - pi_sum - mu

def build_sub_pool(route_pool: List[Tuple[List[int], float]],
                   incumbent: List[Tuple[List[int], float]] | None,
                   max_size: int = 400) -> List[Tuple[List[int], float]]:
    """incumbent 경로는 반드시 포함하고, 나머지는 route_pool의 저비용 위주로 채워 제한된 크기의 sub-pool 구성."""
    keep = []
    seen = set()
    # 1) incumbent 우선 포함
    if incumbent:
        for r, c in incumbent:
            key = frozenset(r[1:-1])
            if key not in seen:
                keep.append((r, c)); seen.add(key)
    # 2) 저비용 위주로 채움
    for r, c in sorted(route_pool, key=lambda rc: rc[1]):
        if len(keep) >= max_size:
            break
        key = frozenset(r[1:-1])
        if key not in seen:
            keep.append((r, c)); seen.add(key)
    return keep


def solve_integer_rmp_with_time(route_pool_sub: List[Tuple[List[int], float]],
                                node_types: List[int],
                                K: int,
                                time_limit_sec: float | None,
                                incumbent: List[Tuple[List[int], float]] | None = None):
    """제한된 sub-pool로 정수 RMP를 시간 제한과 함께 풉니다. incumbent가 있으면 MIP start로 심어줍니다.
       Returns: (solution_list, model) — 해가 없으면 solution_list=[]
    """
    
    customers = [i for i, t in enumerate(node_types) if t in (1, 2)]
    R = len(route_pool_sub)
    m = gp.Model("VRPB_SubMIP")
    m.Params.OutputFlag = 0
    if time_limit_sec is not None and time_limit_sec > 0:
        m.Params.TimeLimit = max(0.0, float(time_limit_sec))

    x = m.addVars(R, vtype=GRB.BINARY, name="x")
    costs = [c for (_, c) in route_pool_sub]
    m.setObjective(gp.quicksum(costs[r] * x[r] for r in range(R)), GRB.MINIMIZE)

    # cover 제약
    for i in customers:
        m.addConstr(gp.quicksum(x[r] for r in range(R) if i in route_pool_sub[r][0]) == 1, name=f"cover_{i}")
    # 차량수
    m.addConstr(gp.quicksum(x[r] for r in range(R)) <= K, name="veh_count")

    # MIP start: incumbent 경로를 1로 설정
    if incumbent:
        # 노드집합→인덱스 매핑
        idx_by_set = {frozenset(route_pool_sub[r][0][1:-1]): r for r in range(R)}
        for r_inc, _c in incumbent:
            key = frozenset(r_inc[1:-1])
            if key in idx_by_set:
                x[idx_by_set[key]].Start = 1.0

    m.optimize()
    sol = []
    if m.SolCount > 0:
        for r in range(R):
            if x[r].X > 0.5:
                sol.append(route_pool_sub[r])
    return sol, m
