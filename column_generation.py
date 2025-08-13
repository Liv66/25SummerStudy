import random
from itertools import combinations

import gurobipy as gp
from gurobipy import GRB

from KJH.KJH_vrpb import Construction


class SolPool:
    def __init__(self, N, dist_mat):
        self.N = N
        self.dist_mat = dist_mat
        self.random_cost = [0] + [random.random() for _ in range(N)]
        self.sol_map = [[] for _ in range(N)]
        self.sol_pool = []
        self.sol_cost = []
        self.hash_map = {}
        self.sol_num = 0

    def get_hash(self, route):
        return sum(self.random_cost[i] for i in route)

    def cal_dist(self, route):
        return sum(self.dist_mat[route[i - 1]][route[i]] for i in range(1, len(route)))

    def add_pool(self, route):
        route_hash = self.get_hash(route)
        cost = self.cal_dist(route)
        if route_hash in self.hash_map:
            print("중복")
            idx = self.hash_map[route_hash]
            if cost < self.sol_cost[idx]:
                self.sol_pool[idx] = route.copy()
                self.sol_cost[idx] = cost
        else:
            self.hash_map[route_hash] = self.sol_num
            for i in route[1:-1]:
                self.sol_map[i].append(self.sol_num)
            self.sol_num += 1
            self.sol_pool.append(route.copy())
            self.sol_cost.append(cost)


class CG_Optimizer:
    def __init__(self, problem_info, spool):
        self.N = problem_info['N']
        self.K = problem_info['K']
        self.node_type = problem_info['node_types']
        self.node_demand = problem_info['node_demands']
        self.capa = problem_info['capa']
        self.dist_mat = problem_info['dist_mat']
        self.spool = spool
        self.dual_cost = []
        self.x_list = []
        self.graph_initialize()

    def optimize_sp(self):
        # master
        model = gp.Model()
        y = model.addVars(range(self.spool.sol_num), vtype=GRB.BINARY)
        model.addConstrs(gp.quicksum(y[ii] for ii in self.spool.sol_map[i]) == 1 for i in range(1, self.N))
        model.addConstr(gp.quicksum(y[i] for i in range(self.spool.sol_num)) <= self.K)
        model.setObjective(gp.quicksum(self.spool.sol_cost[i] * y[i] for i in range(self.spool.sol_num)), GRB.MINIMIZE)
        model.optimize()
        print(model.ObjVal)

    def optimize_master(self):
        # master
        model = gp.Model()
        y = model.addVars(range(self.spool.sol_num), vtype=GRB.CONTINUOUS)
        model.addConstrs(gp.quicksum(y[ii] for ii in self.spool.sol_map[i]) >= 1 for i in range(1, self.N))
        model.addConstr(gp.quicksum(y[i] for i in range(self.spool.sol_num)) <= self.K)
        model.setObjective(gp.quicksum(self.spool.sol_cost[i] * y[i] for i in range(self.spool.sol_num)), GRB.MINIMIZE)
        model.setParam("OutputFlag", 0)
        model.optimize()
        print(model.ObjVal)
        print(model.Pi)
        self.dual_cost = [0] + model.Pi[:-1]
        print(self.dual_cost)

    def graph_initialize(self):
        for i in range(self.N):
            for j in range(i + 1, self.N):
                if self.node_type[i] == 1 and self.node_type[j] == 2:
                    self.x_list.append((i, j))
                elif self.node_type[i] == 2 and self.node_type[j] == 1:
                    self.x_list.append((j, i))
                elif self.node_type[i] == 1 and self.node_type[j] == 1:
                    self.x_list.append((i, j))
                    self.x_list.append((j, i))
                elif self.node_type[i] == 2 and self.node_type[j] == 2:
                    self.x_list.append((i, j))
                    self.x_list.append((j, i))
                elif self.node_type[i] == 0 and self.node_type[j] == 1:
                    self.x_list.append((i, j))
                elif self.node_type[i] == 0 and self.node_type[j] == 2:
                    self.x_list.append((j, i))
                else:
                    print(i, j, self.node_type[i], self.node_type[j])
                    print("ERROR")

    def get_subtours(self, sol):
        # sol = [(i, j) for i, j in vals.keys() if vals[i, j] > 0.5]
        print("!!!", sol)
        seq_list = [-1] * self.N
        for i, j in sol:
            seq_list[i] = j

        tours = []
        while True:
            route = []
            pre = 0
            for i in range(self.N):
                if seq_list[i] >= 0:
                    route.append(i)
                    pre = i
                    break
            if not route:
                break
            while True:
                nxt = seq_list[pre]
                if nxt == -1:
                    tours.append(route)
                    break
                else:
                    route.append(nxt)
                    seq_list[pre] = -1
                    pre = nxt
        print(tours)
        return tours

    def callback(self, model, where):
        if where == GRB.Callback.MIPSOL:
            vals = model.cbGetSolution(model._x)
            sol = [(i, j) for i, j in vals.keys() if vals[i, j] > 0.5]
            tours = self.get_subtours(sol)

            if len(tours) > 1:
                # Save the subtours for future use
                model._subtours.append(tours)
            else:
                # Save the tour for future use
                model._tours.append(tours[0])

            for tours in model._subtours:
                # add a subtour elimination constraint for all but largest subtour
                for tour in tours[:-1]:
                    if 0 not in tour:
                        print("@@@@", tour)
                        model.cbLazy(gp.quicksum(model._x[i, j]
                                                 for i, j in combinations(tour, 2) if (i, j) in model._x)
                                     <= len(tour) - 2)
            print(model._subtours)
            # Reset the subtours
            model._subtours = []

    def optimize_sub(self):
        model = gp.Model()
        model._subtours = []
        model._tours = []
        x = model.addVars(self.x_list, vtype=GRB.BINARY)
        model._x = x
        # u = model.addVars(range(1, self.N), vtype=GRB.BINARY)
        model.addConstr(x.sum(0, '*') == 1)
        model.addConstr(x.sum('*', 0) == 1)
        model.addConstrs(x.sum('*', i) <= 1 for i in range(1, self.N))
        model.addConstrs(x.sum('*', i) - x.sum(i, '*') == 0 for i in range(1, self.N))
        model.addConstr(
            gp.quicksum(self.node_demand[i] * x[i, j] for i, j in self.x_list if self.node_type[i] == 1) <= self.capa)
        model.addConstr(
            gp.quicksum(self.node_demand[i] * x[i, j] for i, j in self.x_list if self.node_type[i] == 2) <= self.capa)
        # model.addConstr(
        #     gp.quicksum(self.node_demand[i] * u[i] for i in range(1, self.N) if self.node_type[i] == 1) <= self.capa)
        # model.addConstr(
        #     gp.quicksum(self.node_demand[i] * u[i] for i in range(1, self.N) if self.node_type[i] == 2) <= self.capa)
        # model.addConstrs(x.sum('*', i) <= u[i] for i in range(1, self.N))
        model.setObjective(gp.quicksum((self.dist_mat[i][j] - self.dual_cost[i]) * x[i, j] for i, j in self.x_list),
                           GRB.MINIMIZE)
        # model.setParam("OutputFlag", 0)
        model.Params.lazyConstraints = 1
        model.optimize(self.callback)
        # model.optimize()

        sol = [(i, j) for i, j in self.x_list if x[i, j].X > 0.5]
        print(self.get_subtours(sol))

        quit()
        self.spool.add_pool(route)
        print(sol)
        print(route)
        print(seq_list)
        print(sol, route, model.ObjVal, self.capa)


def run_master(problem_info, time_limit=60, log=False):
    N = problem_info['N']
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    sol_pool = SolPool(N, dist_mat)
    c = Construction(K, node_type, node_demand, capa, dist_mat)
    for _ in range(1):
        initial_routes = c.construct()
        for route in initial_routes:
            sol_pool.add_pool(route.hist)
    # sol_pool.add_pool([0, 66, 10, 0])
    optimizer = CG_Optimizer(problem_info, sol_pool)
    optimizer.optimize_master()
    optimizer.optimize_sub()
    print("--------------------------------")
    optimizer.optimize_master()
    optimizer.optimize_sub()
    # sol_pool.add_pool([0, 66, 10, 0])
    # optimizer.optimize_master()
    # optimizer.optimize_sub()
    quit()
