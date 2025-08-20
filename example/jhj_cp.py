"""
    cutting stock
"""
import random
import time

import gurobipy as gp


def cutting_stock_instance_generate(L, S):
    ws = [max(int(random.randint(1, L) * 0.7), 1) for _ in range(S)]
    ds = [random.randint(30, 120) for _ in range(S)]

    return ws, ds


def solve_cutting_stock_mip(L, S, ws, ds):
    B = sum(ds)
    M = 987654321
    model = gp.Model()
    y = model.addVars(range(B), vtype=gp.GRB.BINARY, name='y')
    x = model.addVars(range(S), range(B), vtype=gp.GRB.INTEGER, name='x')
    model.addConstrs(x[s, b] <= M * y[b] for s in range(S) for b in range(B))
    model.addConstrs(gp.quicksum(ws[s] * x[s, b] for s in range(S)) <= L for b in range(B))
    model.addConstrs(gp.quicksum(x[s, b] for b in range(B)) >= ds[s] for s in range(S))
    model.setObjective(gp.quicksum(y[b] for b in range(B)))
    model.setParam('OutputFlag', 0)
    model.setParam('TimeLimit', 30)
    model.optimize()
    print(f"OBJval : {model.ObjVal}")


def solve_master_ip(S, ds, patterns):
    model = gp.Model()
    x = model.addVars(range(len(patterns)), vtype=gp.GRB.INTEGER, name='y')
    model.addConstrs(gp.quicksum(patterns[j][i] * x[j] for j in range(len(patterns))) >= ds[i] for i in range(S))
    # model.addConstrs(gp.quicksum(x[j] for j in range(len(patterns)) if patterns[j][i]) >= ds[i] for i in range(S))
    model.setObjective(gp.quicksum(x[i] for i in range(len(patterns))))
    model.setParam('OutputFlag', 0)
    model.optimize()
    return [i for i in range(len(patterns)) if x[i].X > 0.5], model.ObjVal


"""
RCSP는 LP 완화 시켜서 품
"""


def solve_master(S, ds, patterns):
    model = gp.Model()
    x = model.addVars(range(len(patterns)), vtype=gp.GRB.CONTINUOUS, name='y')
    model.addConstrs(gp.quicksum(patterns[j][i] * x[j] for j in range(len(patterns))) >= ds[i] for i in range(S))
    # model.addConstrs(gp.quicksum(x[j] for j in range(len(patterns)) if patterns[j][i]) >= ds[i] for i in range(S))
    model.setObjective(gp.quicksum(x[i] for i in range(len(patterns))))
    model.setParam("OutputFlag", 0)
    model.optimize()
    return model.Pi, model.ObjVal


def solve_pricing(duals, ws, L):
    model = gp.Model()
    a = model.addVars(range(len(ws)), vtype=gp.GRB.INTEGER, name='a')
    model.addConstr(gp.quicksum(ws[i] * a[i] for i in range(len(ws))) <= L)
    model.setObjective(1 - gp.quicksum(duals[i] * a[i] for i in range(len(ws))))
    model.setParam("OutputFlag", 0)
    model.optimize()
    return [a[i].X for i in range(len(ws))], model.ObjVal


def solve_cutting_stock_cg(L, S, ws, ds, log=False):
    patterns = []
    for i in range(S):
        pattern = [0 for _ in range(S)]
        pattern[i] = 1
        patterns.append(pattern)

    cnt = 0
    while True:
        cnt += 1
        duals, master_obj = solve_master(S, ds, patterns)
        _, ip_obj = solve_master_ip(S, ds, patterns)
        new_pattern, pricing_obj = solve_pricing(duals, ws, L)
        if log:
            print(f"rep {cnt} : master_obj = {master_obj}, pricing_obj = {pricing_obj}, ip = {ip_obj}")
        if pricing_obj >= -0.5:
            break
        patterns.append(new_pattern)
    sol, ip_obj = solve_master_ip(S, ds, patterns)
    print(f"CG optimal obj {ip_obj}")
    if log:
        for i in sol:
            print([int(j) for j in patterns[i]])


def run():
    print("---------------")
    L = 250
    S = 10
    ws, ds = cutting_stock_instance_generate(L, S)
    start1 = time.time()
    solve_cutting_stock_mip(L, S, ws, ds)
    elapsed1 = time.time() - start1
    print(elapsed1)
    print("-------------------------------------------")
    print("-------------------------------------------")

    start2 = time.time()
    solve_cutting_stock_cg(L, S, ws, ds)
    elapsed2 = time.time() - start2
    print(elapsed2)



if __name__ == "__main__":
    run()
