import time
from itertools import combinations

from matplotlib import pyplot as plt
import gurobipy as gp
from gurobipy import GRB


def tourcost(dist, tour):
    return sum(dist[tour[k - 1]][tour[k]] for k in range(len(tour)))


def tourcost_dict(dist, tour):
    return sum(dist[tour[k - 1], tour[k]] for k in range(len(tour)))


def subtours(vals):
    # make a list of edges selected in the solution
    edges = gp.tuplelist((i, j) for i, j in vals.keys()
                         if vals[i, j] > 0.5)
    cycles = []
    while edges:
        # Trace edges until we find a loop
        i, j = edges[0]
        thiscycle = [i]
        while j != thiscycle[0]:
            thiscycle.append(j)
            i, j = next((i, j) for i, j in edges.select(j, '*')
                        if j != thiscycle[-2])
        cycles.append(thiscycle)
        for j in thiscycle:
            edges.remove((i, j))
            edges.remove((j, i))
            i = j
    return sorted(cycles, key=lambda x: len(x))


class pytsp:
    def __init__(self, n, dist, logging=False):
        self.n = n
        self.dist = dist
        self.logging = logging

    # Construct a heuristic tour via greedy insertion
    def greedy(self, dist=None, sense=1):
        if not dist:
            dist = self.dist
        unexplored = list(range(self.n))
        tour = []
        prev = 0
        while unexplored:
            best = min((i for i in unexplored if i != prev), key=lambda k: sense * dist[prev, k])
            tour.append(best)
            unexplored.remove(best)
            prev = best
        if self.logging:
            print("**** greedy heuristic tour=%f, obj=%f" % (tourcost(self.dist, tour), tourcost_dict(dist, tour)))
        return tour

    # Construct a heuristic tour via Karp patching method from subtours
    def patch(self, subtours):
        if self.logging:
            print("**** patching %i subtours" % len(subtours))
        tours = list(subtours)  # copy object to avoid destroying it
        while len(tours) > 1:
            # t1,t2 are tours to merge
            # k1,k2 are positions to merge in the tours
            # d is the direction - forwards or backwards
            t2 = tours.pop()
            # Find best merge
            j1, k1, k2, d, obj = min(((j1, k1, k2, d,
                                       self.dist[tours[j1][k1 - 1]][t2[k2 - d]] +
                                       self.dist[tours[j1][k1]][t2[k2 - 1 + d]] -
                                       self.dist[tours[j1][k1 - 1]][tours[j1][k1]] -
                                       self.dist[t2[k2 - 1]][t2[k2]])
                                      for j1 in range(len(tours))
                                      for k1 in range(len(tours[j1]))
                                      for k2 in range(len(t2))
                                      for d in range(2)),  # d=0 is forward, d=1 is reverse
                                     key=lambda x: x[-1])
            t1 = tours[j1]
            k1 += 1  # include the position
            k2 += 1
            if d == 0:  # forward
                tour = t1[:k1] + t2[k2:] + t2[:k2] + t1[k1:]
            else:  # reverse
                tour = t1[:k1] + list(reversed(t2[:k2])) + list(reversed(t2[k2:])) + t1[k1:]
            tours[j1] = tour  # replace j1 with new merge
        if self.logging:
            print("**** patched tour=%f" % tourcost(self.dist, tour))
        return tours[0]

    # Improve a tour via swapping
    # This is simple - just do 2-opt
    def swap(self, tour):
        if self.logging:
            beforecost = tourcost(self.dist, tour)

        for j1 in range(len(tour)):
            for j2 in range(j1 + 1, len(tour)):
                if self.dist[tour[j1 - 1]][tour[j1]] + self.dist[tour[j2 - 1]][tour[j2]] > \
                        self.dist[tour[j1 - 1]][tour[j2 - 1]] + self.dist[tour[j1]][tour[j2]]:
                    # swap
                    tour = tour[:j1] + list(reversed(tour[j1:j2])) + tour[j2:]
        if self.logging:
            print("**** swapping: before=%f after=%f" % (beforecost, tourcost(self.dist, tour)))
        return tour


def tspcb(heurcb=None):
    def basecb(model, where):

        # Check MIP solution
        if where == GRB.Callback.MIPSOL:

            vals = model.cbGetSolution(model._x)
            tours = subtours(vals)
            if len(tours) > 1:
                # Save the subtours for future use
                model._subtours.append(tours)
            else:
                # Save the tour for future use
                model._tours.append(tours[0])

        # Call inner heuristic callback function, if specified
        try:
            heurcb(model, where)
        except TypeError:  # no heuristic callback specified
            pass

        # Add subtour constraints if there are any subtours
        if where == GRB.Callback.MIPSOL:
            for tours in model._subtours:
                # add a subtour elimination constraint for all but largest subtour
                for tour in tours[:-1]:
                    model.cbLazy(gp.quicksum(model._x[i, j]
                                             for i, j in combinations(tour, 2) if (i, j) in model._x)
                                 <= len(tour) - 1)
            # Reset the subtours
            model._subtours = []

        # Inject a heuristic solution, if there is a saved one
        if where == GRB.Callback.MIPNODE:
            try:
                # There may be multiple tours - find the best one
                tour, cost = min(((tour, tourcost(model._dist, tour))
                                  for tour in model._tours),
                                 key=lambda x: x[-1])
                # Only apply if the tour is an improvement
                if cost < model.cbGet(GRB.Callback.MIPNODE_OBJBST):
                    # Set all variables to 0.0 - optional but helpful to suppress some warnings
                    model.cbSetSolution(list(model._x.values()), [0.0] * len(model._x))
                    # Now set variables in tour to 1.0
                    model.cbSetSolution([model._x[tour[k - 1], tour[k]] for k in range(len(tour))],
                                        [1.0] * len(tour))
                    # Use the solution - optional but a slight performance improvement
                    model.cbUseSolution()
                # Reset the tours
                model._tours = []
            except ValueError:  # tours list was already empty
                pass

    return basecb  # the generated function


def swapcb(model, where):
    if where == GRB.Callback.MIPNODE:
        pt = pytsp(model._n, model._dist)
        for k in range(len(model._tours)):
            model._tours[k] = pt.swap(model._tours[k])


def greedycb(model, where):
    if where == GRB.Callback.MIPNODE:
        if model.cbGet(GRB.Callback.MIPNODE_STATUS) == GRB.OPTIMAL:
            x = model.cbGetNodeRel(model._x)
            for k in x:
                if x[k] < 0.001:
                    x[k] = -model._dist[k[0]][k[1]]

            pt = pytsp(model._n, model._dist)
            model._tours.append(pt.greedy(dist=x, sense=-1))  # maximize using the x values


def patchcb(model, where):
    if where == GRB.Callback.MIPSOL:
        pt = pytsp(model._n, model._dist)
        for subtour in model._subtours:
            model._tours.append(pt.patch(subtour))


def solve_subtour_heur(dist_mat, heur=None, timelimit=20):
    start = time.time()

    N = len(dist_mat)
    model = gp.Model()
    # 각 간선에 대해 변수를 1개만 생성
    x_list = [(i, j) for i in range(N) for j in range(i + 1, N)]
    x = model.addVars(x_list, vtype=GRB.BINARY, name='x')
    for i, j in x_list:
        x[j, i] = x[i, j]  # x[i, j]든 x[j, i]든 같은 변수를 가리킴
    model.addConstrs(x.sum(i, '*') == 2 for i in range(N))
    model.setObjective(gp.quicksum(dist_mat[i][j] * x[i, j] for i, j in x_list))
    model._n = len(dist_mat)
    model._x = x  # model 객체에서 결정변수 x 호출가능하도록 함
    model._subtours = []  # subtour 리스트 초기화
    model._tours = []  # feasible solution 리스트
    model._dist = dist_mat
    model.Params.lazyConstraints = 1
    model.Params.TimeLimit = timelimit

    model.optimize(tspcb(heur))

    if model.SolCount > 0:
        vals = model.getAttr('x', model._x)
        tours = subtours(vals)
        if len(tours) == 1:
            if model.Status == GRB.OPTIMAL:
                status = "Optimal TSP tour"
            else:
                status = "Suboptimal TSP tour"
            sol = tours[0]
        else:
            status = f"{len(tours)} TSP subtours"
            sol = tours
        print(f'{status}: {str(sol)}')
        sol = sol + [sol[0]]
        return sol, model.ObjVal, round(time.time() - start, 2)
    else:
        print('No solution!')
