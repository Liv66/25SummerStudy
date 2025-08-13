# cwj_master_problem.py
from gurobipy import Model, GRB, quicksum

def solve_master_problem(route_pool, node_types, dist_mat, K, relax=False):
    N = len(node_types)
    customer_indices = list(range(1, N))

    model = Model("VRPB_Master")
    model.setParam("OutputFlag", 1)

    z = []
    vtype = GRB.CONTINUOUS if relax else GRB.BINARY
    for idx, (route, cost) in enumerate(route_pool):
        z_var = model.addVar(vtype=vtype, obj=cost, name=f"z_{idx}")
        z.append(z_var)

    # 커버 제약: depot(0) 제외
    for i in customer_indices:
        model.addConstr(
            quicksum(z[j] for j, (route, _) in enumerate(route_pool) if i in route[1:-1]) >= 1,
            name=f"visit_{i}"
        )

    model.addConstr(quicksum(z) <= K, name="vehicle_limit")
    model.ModelSense = GRB.MINIMIZE
    model.optimize()

    def _coverage(selected):
        covered = set()
        for r, _ in selected:
            covered.update(r[1:-1])
        required = set(customer_indices)
        return required - covered  # missed set

    def _lp_rounding():
        """LP 해(z_j)를 이용해 커버 보장 정수해를 탐욕으로 복원."""
        # 1) 초기: z 값 큰 경로부터 담아 커버를 채움
        zval = [z_j.X for z_j in z]
        order = sorted(range(len(route_pool)), key=lambda j: zval[j], reverse=True)

        selected_idx = []
        covered = set()
        remaining = set(customer_indices)

        for j in order:
            if len(selected_idx) >= K:
                break
            route, _ = route_pool[j]
            custs = set(route[1:-1])
            gain = len(custs & remaining)
            if gain > 0:
                selected_idx.append(j)
                covered |= custs
                remaining -= custs
                if not remaining:
                    break

        # 2) 아직 남았으면: 신규 커버/증분비용이 좋은 경로를 추가
        if remaining and len(selected_idx) < K:
            # 간단히 "신규 커버 수 / 비용" 휴리스틱
            while remaining and len(selected_idx) < K:
                best_j, best_score = None, -1
                for j in range(len(route_pool)):
                    if j in selected_idx:
                        continue
                    route, cost = route_pool[j]
                    gain_set = set(route[1:-1]) & remaining
                    if not gain_set:
                        continue
                    # score: 신규 커버 수 / (1 + 비용)
                    score = len(gain_set) / (1.0 + cost)
                    if score > best_score:
                        best_score = score
                        best_j = j
                if best_j is None:
                    break
                selected_idx.append(best_j)
                covered |= set(route_pool[best_j][0][1:-1])
                remaining -= set(route_pool[best_j][0][1:-1])

        selected = [route_pool[j] for j in selected_idx]
        missed = set(customer_indices) - covered
        return selected, missed

    selected_routes = []
    duals = [0.0] * N
    missed_customers = set()

    if model.status == GRB.OPTIMAL:
        if relax:
            # LP 라운딩으로 정수해 복원
            selected_routes, missed_customers = _lp_rounding()
        else:
            # 정수해: 0/1로 선택
            for j, z_j in enumerate(z):
                if z_j.X > 0.5:
                    selected_routes.append(route_pool[j])
            missed_customers = _coverage(selected_routes)

        # 듀얼(라그랑주 승수) 추출
        for i in customer_indices:
            c = model.getConstrByName(f"visit_{i}")
            if c is not None:
                duals[i] = c.Pi
        vc = model.getConstrByName("vehicle_limit")
        if vc is not None:
            duals[0] = vc.Pi

    elif model.status == GRB.INFEASIBLE:
        print("[ERROR] Master problem is infeasible!")
        model.computeIIS()
        model.write("infeasible.ilp")
        # route_pool이 커버하지 못하는 고객 목록을 반환
        covered = set()
        for route, _ in route_pool:
            covered.update(route[1:-1])
        missed_customers = set(customer_indices) - covered

    else:
        print(f"[WARN] Gurobi status={model.status}. 빈 해 반환.")
        missed_customers = set(customer_indices)

    return selected_routes, duals, missed_customers