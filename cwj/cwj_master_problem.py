from gurobipy import Model, GRB, quicksum

def solve_master_problem_lp(patterns, costs, num_nodes, force_use_ids, num_vehicles):
    print("[DEBUG] Solving LP Master Problem with Set Covering...")

    model = Model("RMP_LP")
    model.setParam("OutputFlag", 0)

    x = [model.addVar(vtype=GRB.CONTINUOUS, name=f"x_{j}") for j in range(len(patterns))]
    model.update()

    print(f"[DEBUG] Number of patterns (columns): {len(patterns)}")
    print(f"[DEBUG] Number of nodes (excluding depot): {num_nodes - 1}")

    constraints = []
    for i in range(1, num_nodes):  # depot 제외
        expr = quicksum(x[j] for j, pat in enumerate(patterns) if i in pat)
        c = model.addConstr(expr >= 1, name=f"cover_{i}")
        constraints.append(c)

    for j in force_use_ids:
        print(f"[DEBUG] Pattern {j} is forced to be used.")
        model.addConstr(x[j] == 1, name=f"force_use_{j}")

    # 🔸 차량 수 제약 추가
    model.addConstr(quicksum(x) >= num_vehicles, name="vehicle_limit")

    obj = quicksum(costs[j] * x[j] for j in range(len(patterns)))
    model.setObjective(obj, GRB.MINIMIZE)
    model.optimize()

    if model.status != GRB.OPTIMAL:
        print(f"[ERROR] LP did not solve to optimality. Status code = {model.status}")
        
        # Infeasibility 분석 시작
        if model.status == GRB.INFEASIBLE:
            print("[INFO] Model is infeasible. Computing IIS (Irreducible Infeasible Subsystem)...")
            model.computeIIS()
            model.write("infeasible.ilp")  # or infeasible.lp, infeasible.mps

            # 어떤 제약이 infeasible에 기여했는지 출력
            for c in model.getConstrs():
                if c.IISConstr:
                    print(f"[IIS] Constraint: {c.ConstrName}")
            for v in model.getVars():
                if v.IISLB or v.IISUB:
                    print(f"[IIS] Variable Bound: {v.VarName} (LB: {v.IISLB}, UB: {v.IISUB})")

        raise RuntimeError("LP did not solve to optimality.")
    else:
        print(f"[DEBUG] LP Objective Value: {model.ObjVal:.4f}")

    duals = [c.getAttr(GRB.Attr.Pi) for c in constraints]
    for i, pi in enumerate(duals):
        print(f"[DEBUG] Dual value for node {i+1}: {pi:.4f}")

    print("========== [DEBUG] LP Solution Summary ==========")
    for j in range(len(patterns)):
        val = x[j].X
        if val > 1e-6:
            print(f"  x_{j} = {val:.4f} | Pattern: {patterns[j]}")
    print("==================================================")
    
    return model, x, duals


def solve_master_problem_ip(patterns, costs, num_nodes, num_vehicles):
    print("[DEBUG] Solving IP Master Problem with Set Partitioning...")

    model = Model("RMP_IP")
    model.setParam("OutputFlag", 1)
    model.setParam("MIPGap", 0.01)

    x = [model.addVar(vtype=GRB.BINARY, name=f"x_{j}") for j in range(len(patterns))]
    model.update()

    print(f"[DEBUG] Number of patterns (columns): {len(patterns)}")
    print(f"[DEBUG] Number of nodes (excluding depot): {num_nodes - 1}")

    for i in range(1, num_nodes):
        expr = quicksum(x[j] for j, pat in enumerate(patterns) if i in pat)
        model.addConstr(expr == 1, name=f"partition_{i}")

    # 🔸 차량 수 제약 추가
    model.addConstr(quicksum(x) == num_vehicles, name="vehicle_limit")

    obj = quicksum(costs[j] * x[j] for j in range(len(patterns)))
    model.setObjective(obj, GRB.MINIMIZE)
    model.optimize()

    if model.status != GRB.OPTIMAL:
        print(f"[ERROR] IP did not solve to optimality. Status code = {model.status}")
        
        # Infeasibility 분석 시작
        if model.status == GRB.INFEASIBLE:
            print("[INFO] Model is infeasible. Computing IIS (Irreducible Infeasible Subsystem)...")
            model.computeIIS()
            model.write("infeasible.ilp")  # or infeasible.lp, infeasible.mps

            # 어떤 제약이 infeasible에 기여했는지 출력
            for c in model.getConstrs():
                if c.IISConstr:
                    print(f"[IIS] Constraint: {c.ConstrName}")
            for v in model.getVars():
                if v.IISLB or v.IISUB:
                    print(f"[IIS] Variable Bound: {v.VarName} (LB: {v.IISLB}, UB: {v.IISUB})")

        raise RuntimeError("IP did not solve to optimality.")
    else:
        print(f"[DEBUG] IP Objective Value: {model.ObjVal:.4f}")

    for j in range(len(patterns)):
        val = x[j].X
        if val > 0.5:
            print(f"[DEBUG] Pattern {j} used in final solution (x = {val:.2f}): {patterns[j]}")

    return model, x