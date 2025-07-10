import time
from math import sqrt

from matplotlib import pyplot as plt
from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model


class SolContainer:
    def __init__(self):
        self.solutions = []


class VarArraySolutionPrinterWithLimit(cp_model.CpSolverSolutionCallback):
    """Print intermediate solutions."""

    def __init__(self, variables: list[list[cp_model.BoolVarT]], limit: int, container: SolContainer):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.__variables = variables
        self.__num_bin = len(variables[0])
        self.__solution_count = 0
        self.__solution_limit = limit
        self.__container = container

    def on_solution_callback(self) -> None:
        self.__solution_count += 1
        sol = [[] for _ in range(self.__num_bin)]
        for idx, v in enumerate(self.__variables):
            for jdx, vv in enumerate(v):
                if self.value(vv) >= 0.5:
                    sol[jdx].append(idx)
        self.__container.solutions.append(sol)
        if self.__solution_count >= self.__solution_limit:
            # print(f"Stop search after {self.__solution_limit} solutions")
            self.stop_search()

    @property
    def solution_count(self) -> int:
        return self.__solution_count


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


"""
    K : bin 수
    count : 몇개의 해를 구할 것인지
    timelimit : 시간 제약 (초단위)
"""


def multiple_knapsack(weights, capa, K, count=20, timelimit=10, log=False):
    num_items = len(weights)
    all_items = range(num_items)
    all_bins = range(K)

    model = cp_model.CpModel()

    # Variables.
    x = [[model.new_bool_var(f"x_{i}_{b}") for b in all_bins] for i in all_items]
    # Constraints.
    # Each item is assigned to at most one bin.
    for i in all_items:
        model.add(sum(x[i][b] for b in all_bins) == 1)

    # The amount packed in each bin cannot exceed its capacity.
    for b in all_bins:
        model.add(
            sum(x[i][b] * weights[i] for i in all_items)
            <= capa
        )

    # 대칭성 제거 조건
    for b in range(1, K):
        model.add(sum(x[i][b - 1] * weights[i] for i in all_items) >= sum(x[i][b] * weights[i] for i in all_items))

    container = SolContainer()
    solution_printer = VarArraySolutionPrinterWithLimit(x, count, container)
    solver = cp_model.CpSolver()
    solver.parameters.enumerate_all_solutions = True
    solver.parameters.max_time_in_seconds = timelimit
    status = solver.solve(model, solution_printer)

    if solver.status_name(status) == "FEASIBLE":
        if log:
            print(f"  elapsed time: {solver.wall_time} s")
            print(f"  sol found: {solution_printer.solution_count}")
            for idx, sol in enumerate(container.solutions):
                print(f"sol {idx} --------------")
                for jdx, bin in enumerate(sol):
                    print(f"bin {jdx} : {bin}, weight_sum : {sum(weights[i] for i in bin)}, capa : {capa}")
        return container.solutions
    else:
        print("ERROR : find solutions")

"""
    노드들의 좌표가 들어오면 거리행렬 반환
    dist_matrix[i][j] : i에서 j까지 거리
"""
def get_distance(nodes_coord):
    N = len(nodes_coord)
    return [[int(sqrt((nodes_coord[i][0] - nodes_coord[j][0]) ** 2 + (nodes_coord[i][1] - nodes_coord[j][1]) ** 2)) for i in range(N)] for j in
            range(N)]

"""
    depot : 출발 노드
    nodes : 어떤 노드들을 최적화할 것인지, 노드들 인덱스 리스트, ex) [1,2, 5, 7, 8] 
    dist_matrix : 거리 행렬
    nodes_coord : 노드들 좌표
"""
def two_opt(depot, nodes, dist_matrix, nodes_coord, show=False):
    improved = True
    result_route = [i for i in nodes] + [depot]
    st = time.time()
    result_cost = sum(dist_matrix[result_route[i]][result_route[i + 1]] for i in range(len(nodes)))
    while improved:
        improved = False
        for i in range(1, len(nodes) - 1):
            for j in range(i + 1, len(nodes)):
                # before : 기존 경로 / after : 변경된 부분
                before = (dist_matrix[result_route[i - 1]][result_route[i]] + dist_matrix[result_route[j]][result_route[j + 1]])
                after = (dist_matrix[result_route[i - 1]][result_route[j]] + dist_matrix[result_route[i]][result_route[j + 1]])
                if after - before < 0:
                    result_cost += after - before
                    result_route = result_route[:i] + list(reversed(result_route[i:j + 1])) + result_route[j + 1:]
                    improved = True
    if show:
        print("2opt------------------------------")
        print(f"cost : {result_cost}")
        print(f"route : {result_route}")
        print(f"소요시간 {time.time() - st}")
        plot(nodes_coord, result_route, f'ObjVal : {int(result_cost)}')
    return result_cost, result_route


def plot(nodes_coord, route, title=''):
    points_x = [nodes_coord[x][0] for x in route]
    points_y = [nodes_coord[x][1] for x in route]
    plt.scatter(points_x, points_y)
    plt.plot([points_x[i] for i in range(len(route))], [points_y[i] for i in range(len(route))], linestyle='-',
             color='blue',
             label='Line')
    plt.title(title)
    plt.show()

def plot_cvrp(nodes_coord, best_result_route, title=''):
    for route in best_result_route:
        points_x = [nodes_coord[x][0] for x in route]
        points_y = [nodes_coord[x][1] for x in route]
        plt.scatter(points_x, points_y)
        plt.plot([points_x[i] for i in range(len(route))], [points_y[i] for i in range(len(route))], linestyle='-',
                 label='Line')
    plt.title(title)
    plt.show()
