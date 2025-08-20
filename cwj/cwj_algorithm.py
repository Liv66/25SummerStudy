from typing import List, Tuple, Dict, Set
import math
import time
import gurobipy as gp
from gurobipy import GRB
from .cwj_utils import *
from .cwj_initial_patterns import *
from .cwj_rmp import *
from .cwj_pricing import *
##
# -----------------------------
# Strong Diving Heuristic
# -----------------------------
def strong_diving_with_residual_cg(
    problem_info: Dict,
    route_pool: List[Tuple[List[int], float]],
    max_candidates: int = 3,
    res_max_iters: int = 10,
    log_print: bool = False                     # 로그 확인
) -> List[Tuple[List[int], float]] | None:
    """
    Strong diving with residual CG + pool-restore:
      - At each dive level:
        1) Build residual pool (disjoint with covered); if some remaining customers
           have zero coverage, restore FULL pool for the residual LP (RLP).
        2) Run a small CG loop on the RLP to enrich columns.
        3) Solve the RLP to get current bound z_base and fractional solution.
        4) Select up to 'max_candidates' best fractional columns (closest to 1).
        5) For each candidate g, simulate fixing it (covered', K-1), rebuild residual,
           run a tiny CG loop, re-solve RLP, and evaluate
                total_bound_g = cost(g) + obj(RLP | fixed g).
           Pick the candidate with the smallest total_bound_g, then commit it.
      - Repeat until all customers are covered or vehicles are exhausted.
    """
    K = problem_info['K']
    Q = problem_info['capa']
    node_types = problem_info['node_types']
    node_demands = problem_info['node_demands']
    dist_mat = problem_info['dist_mat']

    customers_all = [i for i, t in enumerate(node_types) if t in (1, 2)]
    covered: Set[int] = set()
    incumbent: List[Tuple[List[int], float]] = []
    K_rem = K

    # 유틸: 잔여 풀 구성 + 커버리지 체크
    def build_residual_pool(rpool: List[Tuple[List[int], float]], covered_set: Set[int]):
        pres = [(r, c) for (r, c) in rpool if set(r[1:-1]).isdisjoint(covered_set)]
        return pres

    # 유틸: 잔여 고객 리스트
    def remaining_list(covered_set: Set[int]) -> List[int]:
        return [i for i in customers_all if i not in covered_set]

    # 유틸: 커버리지 부족 고객 파악
    def lacking_customers(pool_res: List[Tuple[List[int], float]], remaining: List[int]) -> List[int]:
        rem_set = set(remaining)
        covered_map = {i: False for i in remaining}
        for rr, _ in pool_res:
            S = set(rr[1:-1])
            for i in S & rem_set:
                covered_map[i] = True
        return [i for i, ok in covered_map.items() if not ok]

    # 유틸: residual CG 루프 (주어진 base_pool과 remaining, K_rem에 대해)
    def residual_cg_loop(base_pool: List[Tuple[List[int], float]], remaining: List[int], Kcap: int):
        pool_local = list(base_pool)
        for _ in range(res_max_iters):
            # RLP 풀기
            _, duals_res, mu_res, obj_res, _m_res = solve_master_problem_gurobi(
                pool_local, node_types, Kcap, relax=True, customers_override=remaining
            )
            if duals_res is None or obj_res == float('inf'):
                return None, None, float('inf'), pool_local
            # 프라이싱
            d4p = duals_res if duals_res is not None else {}
            new_cols = pricing_greedy(d4p, mu_res if mu_res is not None else 0.0,
                                      node_types, node_demands, dist_mat, Q, restarts=12)
            added, replaced = 0, 0
            for r_new, c_new in new_cols:
                S_new = frozenset(r_new[1:-1])
                idx_same = None
                for idx, (rr, cc) in enumerate(route_pool):
                    if frozenset(rr[1:-1]) == S_new:
                        idx_same = idx
                        break
                if idx_same is None:
                    route_pool.append((r_new, c_new))
                    pool_local.append((r_new, c_new))
                    added += 1
                else:
                    if c_new + 1e-6 < route_pool[idx_same][1]:
                        route_pool[idx_same] = (r_new, c_new)
                        for j, (rr, cc) in enumerate(pool_local):
                            if frozenset(rr[1:-1]) == S_new:
                                pool_local[j] = (r_new, c_new)
                                break
                        replaced += 1
            if added == 0 and replaced == 0:

                return duals_res, mu_res, obj_res, pool_local

        _, duals_res, mu_res, obj_res, _m_res = solve_master_problem_gurobi(
            pool_local, node_types, Kcap, relax=True, customers_override=remaining
        )
        return duals_res, mu_res, obj_res, pool_local

    step = 0
    while True:
        remaining = remaining_list(covered)
        if not remaining:
            total = sum(c for _, c in incumbent)
            if log_print:
                print(f"[StrongDive] Feasible solution. Cost={int(total)}")
            return incumbent
        if K_rem <= 0:
            if log_print:
                print("[StrongDive] Out of vehicles.")
            return None

        # 1) 잔여 풀 구성 + 부족 시 풀 복원
        pool_res = build_residual_pool(route_pool, covered)
        base_pool = pool_res
        lacking = lacking_customers(pool_res, remaining)
        if lacking:
            if log_print:
                print(f"[StrongDive] Residual pool lacks {len(lacking)} customers → restore FULL pool for RLP.")
            base_pool = list(route_pool)

        # 2) 잔여 CG로 컬럼 보강 & 현재 바운드 계산
        duals_base, mu_base, obj_base, base_pool = residual_cg_loop(base_pool, remaining, K_rem)
        if duals_base is None or obj_base == float('inf'):
            seeds = generate_initial_patterns(K_rem, Q, node_types, node_demands, dist_mat, seed=77 + step)
            added = 0
            rem_set = set(remaining)
            for r, c in seeds:
                S = set(r[1:-1])
                if not (S & rem_set):
                    continue
                if any(frozenset(S) == frozenset(rr[1:-1]) for rr, _ in route_pool):
                    continue
                route_pool.append((r, c))
                added += 1
                if added >= 10:
                    break
            if added == 0:
                if log_print:
                    print("[StrongDive] RLP infeasible and enrichment failed.")
                return None
            pool_res = build_residual_pool(route_pool, covered)
            base_pool = pool_res if not lacking_customers(pool_res, remaining) else list(route_pool)
            duals_base, mu_base, obj_base, base_pool = residual_cg_loop(base_pool, remaining, K_rem)
            if duals_base is None or obj_base == float('inf'):
                if log_print:
                    print("[StrongDive] RLP infeasible even after enrichment.")
                return None

        # 3) 후보 생성: 현재 base RLP의 분수해에서 x가 큰/1에 가까운 순으로 상위 k
        _, _, _, _, m_read = solve_master_problem_gurobi(
            base_pool, node_types, K_rem, relax=True, customers_override=remaining
        )
        vals = []
        for i in range(len(base_pool)):
            v = m_read.getVarByName(f"x[{i}]")
            if v is None:
                continue
            xi = v.X
            if not set(base_pool[i][0][1:-1]).isdisjoint(covered):
                continue
            if xi > 1e-9:

                vals.append((i, xi, abs(1.0 - xi)))
        if not vals:
            if log_print:
                print("[StrongDive] No fractional columns to dive on.")
            return None

        vals.sort(key=lambda t: t[2])
        candidates = vals[:min(max_candidates, len(vals))]

        # 4) 후보별 룩어헤드 평가
        best_idx = None
        best_score = math.inf
        best_bundle = None  # (route, cost, chosen_var_value)
        for ci, xi, _cl in candidates:
            route_cand, cost_cand = base_pool[ci]
            covered_sim = covered | set(route_cand[1:-1])
            K_sim = K_rem - 1
            if K_sim < 0:
                continue
            remaining_sim = remaining_list(covered_sim)
            if not remaining_sim:
                total_bound = cost_cand
                if total_bound < best_score:
                    best_score = total_bound
                    best_idx = ci
                    best_bundle = (route_cand, cost_cand, xi)
                continue

            pool_res_sim = build_residual_pool(route_pool, covered_sim)
            base_pool_sim = pool_res_sim
            if lacking_customers(pool_res_sim, remaining_sim):
                base_pool_sim = list(route_pool)


            duals_sim, mu_sim, obj_sim, base_pool_sim = residual_cg_loop(base_pool_sim, remaining_sim, K_sim)
            if duals_sim is None or obj_sim == float('inf'):

                total_bound = math.inf
            else:
                total_bound = cost_cand + obj_sim

            if total_bound < best_score:
                best_score = total_bound
                best_idx = ci
                best_bundle = (route_cand, cost_cand, xi)

        if best_idx is None or best_bundle is None:
            if log_print:
                print("[StrongDive] All candidate lookaheads infeasible.")
            return None

        # 5) 베스트 후보를 실제로 고정
        chosen_route, chosen_cost, chosen_val = best_bundle
        step += 1
        if log_print:
            print(f"[StrongDive] Step {step}: choose route with x={chosen_val:.6f} "
                  f"| cost={int(chosen_cost)} | route={chosen_route} | score={int(best_score) if math.isfinite(best_score) else 'inf'}")

        incumbent.append((chosen_route, chosen_cost))
        covered |= set(chosen_route[1:-1])
        K_rem -= 1

# -----------------------------
# Top-level Algorithm
# -----------------------------
def VRPB_CG_Heuristic(problem_info: Dict) -> List[Tuple[List[int], float]]:
    """
    Column-Generation Heuristic for VRPB (no discounts, single vehicle type):
      1) Generate initial feasible patterns covering all customers.
      2) Global CG on full problem (LP).
      3) STRONG DIVING with RESIDUAL CG + POOL RESTORE:
         - Build residual pool disjoint with 'covered'
         - If some remaining customers have zero coverage in residual pool,
           restore to FULL pool for RLP to get duals and run pricing,
           but still pick/fix only DISJOINT routes.
      4) After diving:
         - If diving succeeded: run Sub-MIP (integer RMP on a reduced pool) to polish.
           If total runtime > 60s while polishing, return diving incumbent.
         - If diving failed: still try Sub-MIP within remaining time.
           If total runtime > 60s while polishing, print 'time out' and return [].
      5) If Sub-MIP finds nothing in time, fall back to a greedy cover.
    Returns: list of (route, cost)
    """

    K = problem_info['K']
    Q = problem_info['capa']
    node_types = problem_info['node_types']
    node_demands = problem_info['node_demands']
    dist_mat = problem_info['dist_mat']

    t0 = time.time()

    # 0) Initial route pool
    route_pool = generate_initial_patterns(K, Q, node_types, node_demands, dist_mat, seed=0)
    rmp_init_cost = None

    # 1) Global Column Generation (LP)
    MAX_ITERS = 50
    for it in range(1, MAX_ITERS + 1):
        _, duals, mu, obj, _ = solve_master_problem_gurobi(route_pool, node_types, K, relax=True)
        if rmp_init_cost is None:
            rmp_init_cost = obj
            print(f"Initial RMP(LP) ObjVal = {rmp_init_cost:.2f}")

        # Pricing on full problem
        new_cols = pricing_greedy(duals if duals is not None else {},
                                  mu if mu is not None else 0.0,
                                  node_types, node_demands, dist_mat, Q,
                                  restarts=20)
        # Add if new (by node set)
        added = 0
        for r, c in new_cols:
            S = frozenset(r[1:-1])
            if not any(S == frozenset(rr[0][1:-1]) for rr in route_pool):
                route_pool.append((r, c))
                added += 1
        if added == 0:
            break

    # --- Sub-MIP 유틸 ---
    def _build_sub_pool(pool: List[Tuple[List[int], float]],
                        incumbent: List[Tuple[List[int], float]] | None,
                        max_size: int = 400) -> List[Tuple[List[int], float]]:

        keep, seen = [], set()
        if incumbent:
            for r, c in incumbent:
                key = frozenset(r[1:-1])
                if key not in seen:
                    keep.append((r, c)); seen.add(key)
        for r, c in sorted(pool, key=lambda rc: rc[1]):
            if len(keep) >= max_size:
                break
            key = frozenset(r[1:-1])
            if key not in seen:
                keep.append((r, c)); seen.add(key)
        return keep

    def _solve_submip(pool_sub: List[Tuple[List[int], float]],
                      time_limit: float | None,
                      incumbent: List[Tuple[List[int], float]] | None = None) -> List[Tuple[List[int], float]]:
        m = gp.Model("VRPB_SubMIP")
        m.Params.OutputFlag = 0
        if time_limit is not None and time_limit > 0:
            m.Params.TimeLimit = max(0.0, float(time_limit))

        R = len(pool_sub)
        x = m.addVars(R, vtype=GRB.BINARY, name="x")
        costs = [c for (_, c) in pool_sub]
        m.setObjective(gp.quicksum(costs[r] * x[r] for r in range(R)), GRB.MINIMIZE)

        customers = [i for i, t in enumerate(node_types) if t in (1, 2)]
        for i in customers:
            m.addConstr(gp.quicksum(x[r] for r in range(R) if i in pool_sub[r][0]) == 1, name=f"cover_{i}")
        m.addConstr(gp.quicksum(x[r] for r in range(R)) <= K, name="veh_count")

        if incumbent:
            idx_by_set = {frozenset(pool_sub[r][0][1:-1]): r for r in range(R)}
            for r_inc, _ in incumbent:
                key = frozenset(r_inc[1:-1])
                if key in idx_by_set:
                    x[idx_by_set[key]].Start = 1.0

        m.optimize()
        sol = []
        if m.SolCount > 0:
            for r in range(R):
                if x[r].X > 0.5:
                    sol.append(pool_sub[r])
        return sol

    # 2) STRONG DIVING (with residual CG + pool restore)
    dive_sol = strong_diving_with_residual_cg(
        problem_info, route_pool,
        max_candidates=10,     # 후보 개수
        res_max_iters=30,      # 잔여 CG 반복 수
        log_print= False       # 로그 출력
    )

    # ----- (A) strong diving 성공 -----
    if dive_sol is not None:
        elapsed = time.time() - t0
        remaining = 60.0 - elapsed
        if remaining <= 0:
            total = sum(c for _, c in dive_sol)
            return dive_sol

        pool_sub = _build_sub_pool(route_pool, dive_sol, max_size=400)
        sol_sub = _solve_submip(pool_sub, time_limit=remaining, incumbent=dive_sol)

        elapsed2 = time.time() - t0
        if elapsed2 > 60.0:
            total = sum(c for _, c in dive_sol)
            return dive_sol

        if sol_sub:
            total = sum(c for _, c in sol_sub)
            return sol_sub
        else:
            total = sum(c for _, c in dive_sol)
            return dive_sol

    # ----- (B) strong diving 실패/중단 -----
    elapsed = time.time() - t0
    remaining = 60.0 - elapsed
    if remaining <= 0:
        print("[TIMEOUT] Sub-MIP skipped: total runtime already > 60s.")
        print("time out")
        return []

    pool_sub = _build_sub_pool(route_pool, incumbent=None, max_size=400)
    sol_sub = _solve_submip(pool_sub, time_limit=remaining, incumbent=None)

    elapsed2 = time.time() - t0
    if elapsed2 > 60.0:
        print("time out")
        return []

    if sol_sub:
        return sol_sub

    # 마지막 폴백: 그리디
    customers = [i for i, t in enumerate(node_types) if t in (1, 2)]
    uncovered = set(customers)
    chosen: List[Tuple[List[int], float]] = []
    for r, c in sorted(route_pool, key=lambda rc: rc[1]):
        if uncovered & set(r[1:-1]):
            chosen.append((r, c))
            uncovered -= set(r[1:-1])
        if not uncovered:
            break
    if len(chosen) > K:
        chosen = chosen[:K]
    return chosen