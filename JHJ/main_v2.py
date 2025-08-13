from JHJ.solver import *


def jhj_main(problem_info):
    # N = 50
    # line_p = 0.7
    # capa = 2000
    # problem = f"instances/problem_{N}_{line_p}.json"
    # try:
    #     with open(problem, "r", encoding='utf-8') as f:
    #         problem_info = json.load(f)
    #
    # except FileNotFoundError:
    #     instance_generator(problem, N=N, line_p=line_p, capa=capa)
    #     with open(problem, "r", encoding='utf-8') as f:
    #         problem_info = json.load(f)
            
    start = time.time()

    # ILS를 위한 구성 요소들
    construct = GreedyConstructionStrategy()
    local_search = FirstImprovementStrategy()

    # 메인 Solver를 ILS로 변경
    solver = IteratedLocalSearchSolver(construct, local_search)
    start_time = time.time()
    time_limit = 60
    solution = solver.solve(problem_info, start_time, time_limit) # CVRPBSolution object

    elapsed = time.time() - start_time
    time_limit = 60
    
    final_routes = [list(route) for route in solution.get_routes()]

    print("--- Final Solution ---")
    print(solution)
    distance_cost = sum(r.get_distance_cost() for r in solution.get_routes())
    print(f"Final Distance Cost: {distance_cost:.2f}")

    return final_routes
    # print("\n--- Feasibility Check ---")
    # obj = check_feasible(problem_info, final_routes, elapsed, time_limit)
    # if obj > 0:
    #     print(f"Feasibility Check PASSED. Objective value: {obj:.2f}")
    # else:
    #     print("Feasibility Check FAILED!")
    #
    # plot_vrpb(problem_info, final_routes, f'Optimized VRPB Solution\nObj: {obj:.2f}')

if __name__ == '__main__':
    jhj_main()
