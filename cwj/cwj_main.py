import json
import time
import os, sys
import signal

# import 변환
try:
    from .cwj_algorithm import VRPB_CG_Heuristic
except (ImportError, SystemError):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from cwj.cwj_algorithm import VRPB_CG_Heuristic

# cwj_run
def cwj_run(problem_info):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    print("========== [DEBUG] Problem Info ==========")
    print(f"Number of vehicles (K): {K}")
    print(f"Vehicle capacity (Q): {capa}")
    print(f"Total number of nodes (N): {len(node_type)}")
    print(f"Number of linehaul customers: {node_type.count(1)}")
    print(f"Number of backhaul customers: {node_type.count(2)}")
    print(f"Depot index (always 0): demand={node_demand[0]}, type={node_type[0]}")
    print(f"Distance matrix size: {len(dist_mat)} x {len(dist_mat[0])}")
    print("==========================================")

    solution = VRPB_CG_Heuristic(problem_info)
    routes_only = []
    if solution is None:
        print("[WARN] VRPB_CG_Heuristic returned None.")
    elif isinstance(solution, dict):
        for key in ("routes", "solution", "best_routes", "best_solution"):
            if key in solution:
                routes_only = solution[key]
                break
    elif isinstance(solution, (list, tuple)) and len(solution) > 0:
        first = solution[0]
        if isinstance(first, (list, tuple)) and len(first) >= 2 \
           and isinstance(first[0], (list, tuple)) and isinstance(first[1], (int, float)):
            routes_only = [list(r) for (r, _) in solution]
        elif isinstance(first, (list, tuple)) and all(isinstance(x, int) for x in first):
            routes_only = [list(r) for r in solution]
        elif isinstance(first, int):
            routes_only = [list(solution)]

    print("Final solution:")
    total_cost = 0
    if isinstance(solution, (list, tuple)) and len(solution) > 0 \
       and isinstance(solution[0], (list, tuple)) and len(solution[0]) >= 2 \
       and isinstance(solution[0][0], (list, tuple)) and isinstance(solution[0][1], (int, float)):

        for route, cost in solution:
            print(f"Route: {route} | Cost: {int(cost)}")
            total_cost += cost
        print(f"Total Cost: {int(total_cost)}")
    else:
        for r in routes_only:
            print(f"Route: {r}")

    return routes_only


# 타임아웃 핸들러
def timeout_handler(signum, frame):
    raise TimeoutError("Time limit exceeded")


# 실행부
if __name__ == '__main__':
    with open('/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/problem_70_0.7.json', 'r') as f:
        instance = json.load(f)

    # 60초 제한 설정
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(60)  # 60초 타이머 시작

    try:
        start_time = time.time()
        cwj_run(instance)
        end_time = time.time()
        elapsed = end_time - start_time
        print(f"\n실행 시간: {elapsed:.2f}초")
    except TimeoutError:
        print("\n[ERROR] Time out.")
    finally:
        signal.alarm(0)  # 타이머 해제