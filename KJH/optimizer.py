import time

import gurobipy as gp
from gurobipy import GRB


class GRB_solver:
    def __init__(self):
        pass

    def solv_SP(self, spool, N, K, tl=50, log=False):
        route_list = []
        cost_list = []
        start_time = time.time()

        for h in spool.pool:
            route_list.append(spool.sol_hash[h])
            cost_list.append(spool.cost_hash[h])
        iter = 0

        model = gp.Model()
        model.setParam('OutputFlag', 0)
        model.setParam('TimeLimit', tl)
        y = model.addVars(range(len(route_list)), vtype=GRB.BINARY, name='y')
        model.addConstrs(
            gp.quicksum(y[j] for j in range(len(route_list)) if i in route_list[j]) == 1 for i in range(1, N))
        model.addConstr(gp.quicksum(y[j] for j in range(len(route_list))) <= K)
        model.setObjective(gp.quicksum(cost_list[j] * y[j] for j in range(len(route_list))), GRB.MINIMIZE)
        model.optimize()

        solution = [(route_list[i], i) for i in range(len(route_list)) if y[i].X > 0.5]
        iter += 1
        objVal = model.ObjVal
        if log:
            print(f"Optimize iter {iter}, obj : {objVal}, elapsed time {time.time() - start_time:2f}")

        return [sol[0] for sol in solution], round(objVal)

    def optimize_sc(self, route_list, cost_list, N, K):
        model = gp.Model()
        model.setParam('OutputFlag', 0)
        y = model.addVars(range(len(route_list)), vtype=GRB.BINARY, name='y')
        model.addConstrs(
            gp.quicksum(y[j] for j in range(len(route_list)) if i in route_list[j]) >= 1 for i in range(1, N))
        model.addConstr(gp.quicksum(y[j] for j in range(len(route_list))) <= K)
        model.setObjective(gp.quicksum(cost_list[j] * y[j] for j in range(len(route_list))), GRB.MINIMIZE)
        model.optimize()
        new_route_list = []
        new_cost_list = []
        for i in range(len(route_list)):
            if y[i].X > 0.5:
                new_route_list.append(route_list[i].copy())
                new_cost_list.append(cost_list[i])

        return new_route_list, new_cost_list

    def solv_SC(self, spool, dist_mat, N, K, log=False):
        route_list = []
        cost_list = []

        start_time = time.time()

        for h in spool.pool:
            route_list.append(spool.sol_hash[h])
            cost_list.append(spool.cost_hash[h])

        iter = 0

        new_route_list, new_cost_list = self.optimize_sc(route_list, cost_list, N, K)
        break_flag = True
        while break_flag:
            node_route = [[] for _ in range(N)]
            for i in range(len(new_route_list)):
                for j in new_route_list[i]:
                    node_route[j].append(i)

            break_flag = False
            for i in range(1, N):
                if len(node_route[i]) >= 2:
                    break_flag = True
                    for j in node_route[i]:
                        new_route = new_route_list[j].copy()
                        new_route.remove(i)
                        new_cost = sum(dist_mat[new_route[k - 1]][new_route[k]] for k in range(1, len(new_route)))
                        new_route_list.append(new_route)
                        new_cost_list.append(new_cost)
                        spool.add_pool(new_route, new_cost)

            if break_flag:
                new_route_list, new_cost_list = self.optimize_sc(new_route_list, new_cost_list, N, K)
                iter += 1

        # if log:
        #     print(f"Optimize iter {iter}, obj : {objVal}, elapsed time {time.time() - start_time:2f}")
        objVal = sum(new_cost_list)
        print(f"Optimize iter {iter}, obj : {objVal}, elapsed time {time.time() - start_time:2f}")
        return new_route_list, objVal
