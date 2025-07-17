import time
from itertools import combinations

from matplotlib import pyplot as plt
import gurobipy as gp
from gurobipy import GRB


def solve_MTZ(dist_mat, timelimit=20):
    start = time.time()
    N = len(dist_mat)
    model = gp.Model()

    x_list = [(i, j) for i in range(N) for j in range(N) if i != j]
    x = model.addVars(x_list, vtype=GRB.BINARY, name='x')
    u = model.addVars(range(N), vtype=GRB.INTEGER, name='u')

    model.addConstrs(gp.quicksum(x[i, j] for i in range(N) if i != j) == 1 for j in range(N))
    model.addConstrs(gp.quicksum(x[j, i] for i in range(N) if i != j) == 1 for j in range(N))

    # MTZ 제약, i 이후 j를 방문한다면 j의 방문 순서는 i 보다 크다
    model.addConstrs(
        u[i] - u[j] + 1 <= (N - 1) * (1 - x[i, j]) for i, j in x_list if 1 <= i <= N - 1 and 1 <= j <= N - 1)
    model.addConstrs(1 <= u[i] for i in range(1, N))
    model.addConstrs(u[i] <= N - 1 for i in range(1, N))
    model.setObjective(gp.quicksum(dist_mat[i][j] * x[i, j] for i, j in x_list))
    model.Params.TimeLimit = timelimit
    model.optimize()

    if model.status == GRB.OPTIMAL or model.status == GRB.TIME_LIMIT:
        obj = model.ObjVal
        sol = [i for i in range(N)]
        sol.sort(key=lambda i: int(u[i].X))
        sol = sol + [sol[0]]
        return sol, obj, round(time.time() - start, 2)
    else:
        print("Infeasible or Unbounded")
        quit()


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


def subtour_callback(model, where):
    if where == GRB.Callback.MIPSOL:
        x = model.cbGetSolution(model._x)
        tours = subtours(x)
        if len(tours) > 1:
            # Save the subtours for future use
            model._subtours = tours
        else:
            model._tours.append(tours[0])

        for tour in model._subtours:
            subtour_idx = [(tour[i], tour[j]) for i in range(len(tour)) for j in
                           range(i + 1, len(tour))]  # combinations(tour, 2)
            model.cbLazy(gp.quicksum(model._x[i, j]
                                     for i, j in subtour_idx if (i, j) in model._x)
                         <= len(tour) - 1)
        # # Reset the subtours
        model._subtours = []


def solve_subtour(dist_mat, timelimit=20):
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

    model._x = x  # model 객체에서 결정변수 x 호출가능하도록 함
    model._subtours = []  # subtour 리스트 초기화
    model._tours = []  # feasible solution 리스트
    model.Params.TimeLimit = timelimit

    model.Params.lazyConstraints = 1  # use lazy constraint
    model.optimize(subtour_callback)

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


def two_opt(dist_matrix):
    improved = True
    nodes = [i for i in range(len(dist_matrix))]
    sol = [i for i in nodes] + [0]
    start = time.time()
    obj = sum(dist_matrix[sol[i]][sol[i + 1]] for i in range(len(nodes)))
    while improved:
        improved = False
        for i in range(1, len(nodes) - 1):
            for j in range(i + 1, len(nodes)):
                # before : 기존 경로 / after : 변경된 부분
                before = (dist_matrix[sol[i - 1]][sol[i]] + dist_matrix[sol[j]][
                    sol[j + 1]])
                after = (dist_matrix[sol[i - 1]][sol[j]] + dist_matrix[sol[i]][
                    sol[j + 1]])
                if after - before < 0:
                    obj += after - before
                    sol = sol[:i] + list(reversed(sol[i:j + 1])) + sol[j + 1:]
                    improved = True

    return sol, obj, round(time.time() - start, 2)


def plot_tsp(dist_mat, nodes_coord, sol=[], obj=0, elapsed=0, title=''):
    check_obj = sum(dist_mat[sol[i]][sol[i + 1]] for i in range(len(sol) - 1))
    if round(obj) != int(check_obj):
        print(f"obj와 check_obj 값이 다름 obj {round(obj)} check obj {check_obj}")
        # return
    title += f' obj = {check_obj} elapsed={elapsed}'
    points_x = [nodes_coord[i][0] for i in range(len(nodes_coord))]
    points_y = [nodes_coord[i][1] for i in range(len(nodes_coord))]
    plt.scatter(points_x, points_y)
    plt.plot([points_x[i] for i in sol], [points_y[i] for i in sol], linestyle='-',
             label='Line', )
    plt.title(title)
    plt.xticks([])  # x축 눈금 없애기
    plt.yticks([])  # y축 눈금 없애기
    plt.show()
