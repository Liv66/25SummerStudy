import json
import random
import time

from JHJ_util import *
from JHJ_construction import *
from JHJ_method import *
from JHJ_solver import *
from JHJ_Solution import *


def JHJ_run(problem_info, time_limit):

    # class 호출
    construct = GreedyConstructionStrategy()
    local_search = FirstImprovementStrategy()

    # 메인 Solver를 ILS로 변경
    solver = IteratedLocalSearchSolver(construct, local_search)


    start_time = time.time()
    solution = solver.solve(problem_info, start_time, time_limit)
    elapsed = time.time() - start_time

    final_routes = [list(route) for route in solution.get_routes()]


    # obj = check_feasible(problem_info, final_routes, elapsed, time_limit)
    # plot_vrpb(problem_info, final_routes, f'Optimized VRPB Solution\nObj: {obj:.2f}')

    return final_routes
