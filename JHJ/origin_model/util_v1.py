
def get_routes(solution):
    """solution이 리스트라면 그대로, CVRPBSolution 객체라면 내부 리스트 추출"""
    try:
        return solution.get_routes()
    except AttributeError:
        return list(solution)

def get_total_cost(solution):
    """routes 리스트 또는 CVRPBSolution 객체의 총 비용을 계산"""
    routes = get_routes(solution)
    return sum(r.get_cost() for r in routes)

def shuffle_routes(solution):
    """routes 리스트를 제자리에서 섞음"""
    import random
    routes = get_routes(solution)
    random.shuffle(routes)
    return routes

def unmark_all_routes(solution):
    """각 Route 객체의 unmark_all 호출(정의된 경우만)"""
    for r in get_routes(solution):
        if hasattr(r, 'unmark_all'):
            r.unmark_all()
    return solution

def routes_to_str(solution):
    """각 Route.__str__() 결과를 줄바꿈으로 연결"""
    return "\n".join(str(r) for r in get_routes(solution))

from copy import deepcopy
from math import sqrt

from matplotlib import pyplot as plt
from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model

def get_distance(nodes_coord):
    N = len(nodes_coord)
    return [[int(sqrt((nodes_coord[i][0] - nodes_coord[j][0]) ** 2 + (nodes_coord[i][1] - nodes_coord[j][1]) ** 2)) for i in range(N)] for j in
            range(N)]

def route_cost(route, problem_info):
    dist = problem_info['dist_mat']
    depot = 0
    cost = 0.0
    prev = depot
    for n in route.hist:
        cost += dist[prev][n];
        prev = n
    cost += dist[prev][depot]
    route.cost = cost
    return cost

def total_cost(routes, problem_info):
    return sum(route_cost(r, problem_info) for r in routes if r.use)

def route_load(route, problem_info):
    demands = problem_info['node_demands']
    return sum(demands[i] for i in route)

def bin_packing(weights, capa, log=False):
    num_items = len(weights)
    num_bins = num_items  # 최대 필요 bin 수 (각 아이템이 개별 bin에 들어갈 수도 있으므로)

    solver = pywraplp.Solver.CreateSolver("SCIP")
    if not solver:
        print("Solver를 생성할 수 없습니다.")
        return

    # 변수: x[i][j] = 아이템 i가 bin j에 배정되었는지 여부 (0 또는 1)
    x = {}
    for i in range(num_items):
        for j in range(num_bins):
            x[i, j] = solver.BoolVar(f'x_{i}_{j}')

    # 변수: y[j] = bin j가 사용되었는지 여부 (0 또는 1)
    y = {}
    for j in range(num_bins):
        y[j] = solver.BoolVar(f'y_{j}')

    # 제약조건 1: 각 아이템은 정확히 하나의 bin에만 배정되어야 함
    for i in range(num_items):
        solver.Add(solver.Sum(x[i, j] for j in range(num_bins)) == 1)

    # 제약조건 2: 각 bin의 총 무게는 bin 용량을 넘을 수 없음
    for j in range(num_bins):
        solver.Add(
            solver.Sum(weights[i] * x[i, j] for i in range(num_items)) <= capa * y[j]
        )

    # 목적함수: 사용된 bin 수 최소화
    solver.Minimize(solver.Sum(y[j] for j in range(num_bins)))

    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        if log:
            print(f'최소 사용 bin 수: {round(solver.Objective().Value())}')
            for j in range(num_bins):
                if y[j].solution_value() > 0.5:
                    bin_items = [i for i in range(num_items) if x[i, j].solution_value() > 0.5]
                    total_weight = sum(weights[i] for i in bin_items)
                    print(f'Bin {j}: items={bin_items}, total_weight={total_weight}')
        return round(solver.Objective().Value())
    else:
        print('해를 찾을 수 없습니다.')
        print(f'weights : {weights}')
        print(f'capa : {capa}')
        quit()


def plot_vrpb(problem_info, best_result_route=[], title=''):
    nodes_coord = problem_info['node_coords']
    nodes_type = problem_info['node_types']
    for i in range(len(nodes_type)):
        if nodes_type[i] == 0:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=150, marker='s', color='black')
        elif nodes_type[i] == 1:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=35, color='blue')
        else:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=35, marker='^', color='red')

        plt.text(nodes_coord[i][0], nodes_coord[i][1], f'{i}', ha='left', va='bottom', fontsize=8)

    for route in best_result_route:
        points_x = [nodes_coord[x][0] for x in route]
        points_y = [nodes_coord[x][1] for x in route]
        # plt.scatter(points_x, points_y)
        plt.plot([points_x[i] for i in range(len(route))], [points_y[i] for i in range(len(route))], linestyle='-',
                 label='Line', )
    plt.title(title)
    plt.xticks([])  # x축 눈금 없애기
    plt.yticks([])  # y축 눈금 없애기
    plt.show()


def check_feasible(problem_info, sol, elapsed, timelimit):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    if elapsed > timelimit + 1:
        print("Time Out")
        return 0

    if len(sol) > K:
        print(f"vehicle 수는 {K}대까지 사용 가능합니다. 현재 : {len(sol)}")
        return 0

    total_cost = 0
    visit = [0] * len(node_type)
    visit[0] = 1

    for idx, route in enumerate(sol):
        if route[0] != 0:
            print(f"depot에서 출발해야 합니다. {idx}번째 차량 경로 {route}")
            return 0

        if route[-1] != 0:
            print(f"depot으로 도착해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if node_type[route[1]] == 2:
            print(f"차량은 backhaul만 갈 수 없습니다. {idx}번째 차량 경로 {route}")
            return 0
        cost = 0
        load = 0

        pre = 0
        flag = False
        route_type = [0] * len(route)

        for i in range(1, len(route) - 1):
            nxt = route[i]
            if visit[nxt]:
                print(f"{nxt} 노드 2번 이상 방문")
                return 0
            visit[nxt] = 1
            cost += dist_mat[pre][nxt]

            route_type[i] = node_type[nxt]
            if nxt == 0:
                print(f"{idx}번째 차량 depot을 3번 이상 방문 {route}")
                return 0

            if node_type[pre] == 1 and node_type[nxt] == 2:
                if flag:
                    print(f"{idx}번째 차량 line -> backhaul -> line 경로 존재")
                    print(node_type)
                    return 0
                flag = True
                load = 0

            load += node_demand[nxt]
            print(f"{nxt} : {load}")
            if load > capa:
                if flag:
                    print(f"{idx}번째 차량의 back 용량 초과 {load}, {capa}")
                else:
                    print(f"{idx}번째 차량의 line 용량 초과 {load}, {capa}")
                return 0
            pre = nxt
        cost += dist_mat[pre][0]
        total_cost += cost

    if sum(visit) < len(node_type):
        yet_visit = [i for i in range(len(node_type)) if not visit[i]]
        print(f"다음 노드들을 방문하지 않았습니다. {yet_visit}")
        return 0

    return total_cost