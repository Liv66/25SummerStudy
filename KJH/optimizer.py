import gurobipy as gp
from gurobipy import GRB


def solv_SC(spool, dist_mat, N, K):
    route_list = []
    cost_list = []
    solution = []
    for h in spool.pool:
        route_list.append(spool.sol_hash[h])
        cost_list.append(spool.cost_hash[h])

    break_flag = False
    iter = 0
    while not break_flag:
        node_route = [[] for _ in range(N)]

        model = gp.Model()
        y = model.addVars(range(len(route_list)), vtype=GRB.BINARY, name='y')
        model.addConstrs(
            gp.quicksum(y[j] for j in range(len(route_list)) if i in route_list[j]) >= 1 for i in range(1, N))
        model.addConstr(gp.quicksum(y[j] for j in range(len(route_list))) <= K)
        model.setObjective(gp.quicksum(cost_list[j] * y[j] for j in range(len(route_list))), GRB.MINIMIZE)
        model.optimize()
        print(f"Optimize iter {iter}, obj : {model.ObjVal}")

        solution = [(route_list[i], i) for i in range(len(route_list)) if y[i].X > 0.5]
        for i in range(len(solution)):
            route, idx = solution[i]
            for j in route:
                node_route[j].append(idx)
        break_flag = True
        for i in range(1, N):
            if len(node_route[i]) == 0:
                print(f"ERROR 미방문 노드 {i}")
            elif len(node_route[i]) == 1:
                continue
            elif len(node_route[i]) >= 2:
                break_flag = False
                for j in node_route[i]:
                    new_route = route_list[j].copy()
                    new_route.remove(i)
                    new_cost = sum(dist_mat[new_route[i - 1]][new_route[i]] for i in range(1, len(new_route)))
                    route_list.append(new_route)
                    cost_list.append(new_cost)
        iter += 1

    return [sol[0] for sol in solution]
