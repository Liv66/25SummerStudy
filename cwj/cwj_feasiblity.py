# -*- coding: utf-8 -*-
import json
import time
import os
import sys
import signal
##
# =========================
#  Feasibility Check
# =========================
def check_feasible(problem_info, sol, elapsed, timelimit):
    K = problem_info['K']
    node_type = problem_info['node_types']      # 0: depot, 1: linehaul, 2: backhaul
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    # 시간 제한 확인
    if elapsed > timelimit + 1:
        print("Time Out")
        return 0

    # 차량 대수 제한
    if len(sol) > K:
        print(f"vehicle 수는 {K}대까지 사용 가능합니다. 현재 : {len(sol)}")
        return 0

    total_cost = 0
    visit = [0] * len(node_type)
    visit[0] = 1  # depot

    for idx, route in enumerate(sol):
        # 형식/시작-종료 체크
        if not route or route[0] != 0:
            print(f"depot에서 출발해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if route[-1] != 0:
            print(f"depot으로 도착해야 합니다. {idx}번째 차량 경로 {route}")
            return 0

        # backhaul-only 금지
        if len(route) >= 2 and node_type[route[1]] == 2:
            print(f"차량은 backhaul만 갈 수 없습니다. {idx}번째 차량 경로 {route}")
            return 0

        cost = 0
        load = 0
        pre = 0
        flag = False  # line→backhaul 전환 이후 다시 line 방문 금지

        for i in range(1, len(route) - 1):
            nxt = route[i]

            # 중복 방문 체크(루트 간 포함)
            if visit[nxt]:
                print(f"{nxt} 노드 2번 이상 방문")
                return 0
            visit[nxt] = 1

            # 이동 비용
            cost += dist_mat[pre][nxt]
            # 적재량(라인홀은 적재 감소/백홀은 적재 증가 모델도 있으나
            # 여기서는 주어진 node_demand를 누적하는 방식으로 검증)
            load += node_demand[nxt]

            # depot 재방문 금지
            if nxt == 0:
                print(f"{idx}번째 차량 depot을 3번 이상 방문 {route}")
                return 0

            # line→backhaul 전환 이후 다시 line 금지
            if node_type[pre] == 1 and node_type[nxt] == 2:
                if flag:
                    print(f"{idx}번째 차량 line -> backhaul -> line 경로 존재")
                    print(node_type)
                    return 0
                flag = True
                # 전환 시(백홀 탑재로 간주) 적재 초기화
                load = 0

            # 용량 초과
            if load > capa:
                if flag:
                    print(f"{idx}번째 차량의 back 용량 초과 {load}, {capa}")
                else:
                    print(f"{idx}번째 차량의 line 용량 초과 {load}, {capa}")
                return 0

            pre = nxt

        # depot 귀환 비용
        cost += dist_mat[pre][0]
        total_cost += cost

    # 누락 노드 체크
    if sum(visit) < len(node_type):
        yet_visit = [i for i in range(len(node_type)) if not visit[i]]
        print(f"다음 노드들을 방문하지 않았습니다. {yet_visit}")
        return 0

    return total_cost


# =========================
#  Heuristic Import
# =========================
try:
    from .cwj_algorithm import VRPB_CG_Heuristic
except (ImportError, SystemError):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(this_dir)
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)
    try:
        from cwj.cwj_algorithm import VRPB_CG_Heuristic
    except (ImportError, SystemError):
        # 마지막 시도: 현재 경로 기준
        from cwj_algorithm import VRPB_CG_Heuristic


# =========================
#  Runner
# =========================
def cwj_run(problem_info, timelimit=60):
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

    # 휴리스틱 실행
    solution = VRPB_CG_Heuristic(problem_info)

    # 출력 + routes_only 추출
    routes_only = []
    print("\nFinal solution:")
    total_cost_from_solution = 0

    # 케이스: [(route, cost), ...]
    if isinstance(solution, (list, tuple)) and len(solution) > 0 \
       and isinstance(solution[0], (list, tuple)) and len(solution[0]) >= 2 \
       and isinstance(solution[0][0], (list, tuple)) and isinstance(solution[0][1], (int, float)):
        for route, cost in solution:
            print(f"Route: {route} | Cost: {int(cost)}")
            total_cost_from_solution += float(cost)
            routes_only.append(list(route))
        print(f"Total Cost (solver reported): {int(total_cost_from_solution)}")

    # 케이스: {'routes': [...]} 등 dict
    elif isinstance(solution, dict):
        for key in ("routes", "solution", "best_routes", "best_solution"):
            if key in solution:
                routes_only = solution[key]
                break
        for r in routes_only:
            print(f"Route: {r}")

    # 케이스: [[...], [...]] 혹은 단일 라우트 [...]
    elif isinstance(solution, (list, tuple)) and len(solution) > 0:
        first = solution[0]
        if isinstance(first, (list, tuple)) and len(first) >= 2 \
           and isinstance(first[0], (list, tuple)) and isinstance(first[1], (int, float)):
            routes_only = [list(r) for (r, _) in solution]
        elif isinstance(first, (list, tuple)) and all(isinstance(x, int) for x in first):
            routes_only = [list(r) for r in solution]
        elif isinstance(first, int):
            routes_only = [list(solution)]
        for r in routes_only:
            print(f"Route: {r}")
    else:
        print("[WARN] VRPB_CG_Heuristic returned None or unrecognized format.")

    return routes_only


# =========================
#  Timeout Handler
# =========================
def timeout_handler(signum, frame):
    raise TimeoutError("Time limit exceeded")


# =========================
#  Main
# =========================
if __name__ == '__main__':
    # 인스턴스 경로 수정 가능
    instance_path = '/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/problem_100_0.7.json'
    timelimit = 60  # 초

    with open(instance_path, 'r') as f:
        instance = json.load(f)

    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(timelimit)

    try:
        start_time = time.time()
        routes = cwj_run(instance, timelimit=timelimit)
        end_time = time.time()
        elapsed = end_time - start_time

        print(f"\n실행 시간: {elapsed:.2f}초")

        # -----------------------------
        #  Feasibility 체크 + 총비용
        # -----------------------------
        if not routes:
            print("\n[ERROR] 경로가 비어있습니다. 해를 찾지 못했거나 포맷이 올바르지 않습니다.")
        else:
            print("\n[FEASIBILITY CHECK]")
            total_cost = check_feasible(instance, routes, elapsed, timelimit)
            if total_cost == 0:
                print("=> Infeasible")
            else:
                print(f"=> Feasible | Total Cost (recomputed): {int(total_cost)}")

    except TimeoutError:
        print("\n[ERROR] Time out: 60초 안에 해를 찾지 못했습니다.")
    finally:
        signal.alarm(0)