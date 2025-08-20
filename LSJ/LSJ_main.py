import json
import time
import numpy as np

from LSJ_solver import solve_vrp_with_column_generation
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
        if not route or len(route) < 2: continue
        if route[0] != 0:
            print(f"depot에서 출발해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if route[-1] != 0:
            print(f"depot으로 도착해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if len(route) > 2 and node_type[route[1]] == 2:
            print(f"차량은 backhaul만 갈 수 없습니다. {idx}번째 차량 경로 {route}")
            return 0
        cost = 0; load = 0; pre = 0; flag = False
        for i in range(1, len(route) - 1):
            nxt = route[i]
            if visit[nxt]:
                print(f"{nxt} 노드 2번 이상 방문")
                return 0
            visit[nxt] = 1; cost += dist_mat[pre][nxt]; load += node_demand[nxt]
            if nxt == 0:
                print(f"{idx}번째 차량 depot을 3번 이상 방문 {route}")
                return 0
            if node_type.get(pre, 0) == 1 and node_type[nxt] == 2:
                if flag:
                    print(f"{idx}번째 차량 line -> backhaul -> line 경로 존재")
                    return 0
                flag = True; load = node_demand[nxt]
            if load > capa:
                print(f"{idx}번째 차량의 {'back' if flag else 'line'} 용량 초과 {load}, {capa}")
                return 0
            pre = nxt
        cost += dist_mat[pre][0]
        total_cost += cost
    customers_to_visit = {i for i, n_type in enumerate(node_type) if n_type in [1, 2]}
    visited_customers = {i for i, v_count in enumerate(visit) if v_count > 0 and i != 0}
    if visited_customers != customers_to_visit:
        unvisited = customers_to_visit - visited_customers
        print(f"다음 노드들을 방문하지 않았습니다. {sorted(list(unvisited))}")
        return 0
    return total_cost
def LSJ_run(problem_info):
    """
    평가 스크립트가 호출할 메인 진입점 함수.
    problem_info 딕셔너리를 받아 최종 경로 리스트를 반환합니다.
    """
    # 1. problem_info 딕셔너리에서 필요한 데이터 파싱
    nodes = [{'x': problem_info['node_coords'][i][0], 'y': problem_info['node_coords'][i][1],
              'demand': problem_info['node_demands'][i],
              'type': {0: 'depot', 1: 'linehaul', 2: 'backhaul'}[problem_info['node_types'][i]],
              'id': i} for i in range(len(problem_info['node_coords']))]
    dist_matrix = np.array(problem_info['dist_mat'])
    num_vehicles = problem_info['K']
    capacity = problem_info['capa']

    # 2. 핵심 로직 함수 호출
    final_routes = solve_vrp_with_column_generation(
        nodes=nodes,
        dist_matrix=dist_matrix,
        num_vehicles=num_vehicles,
        capacity=capacity,
        instance_data=problem_info,
        total_time_limit=55,
        final_mip_time_limit=13
    )

    # 3. 최종 경로 반환
    return final_routes

# --------------------------------------------------------------------------------
# 이 파일을 직접 실행할 경우를 위한 테스트 코드
# --------------------------------------------------------------------------------
if __name__ == '__main__':
    
    instance_json_file = 'instances/problem_50_0.5.json'

    try:
        # 1. 평가 스크립트와 동일하게 problem_info를 먼저 로드
        print(f"--- Loading test instance: {instance_json_file} ---")
        with open(instance_json_file, "r", encoding='utf-8') as f:
            problem_info = json.load(f)

        # 2. 새로 만든 LSJ_run을 호출하여 테스트
        print(f"--- Running test for LSJ_run ---")
        start_time = time.time()

        final_solution = LSJ_run(problem_info)

        elapsed = time.time() - start_time
        print(f"--- Test Finished in {elapsed:.2f} seconds ---")

        # 3. 반환된 결과 및 Feasibility Check
        print("\n[Returned Solution]")
        if final_solution:
            # 반환된 해를 표준 checker로 검증
            final_routes_for_check = [r for r in final_solution if len(r) > 2]
            obj = check_feasible(problem_info, final_routes_for_check, elapsed, 60)

            print(f"Feasibility Check Result (Cost): {obj}")
            if obj > 0:
                print("Solution is FEASIBLE.")
            else:
                print("Solution is INFEASIBLE or an error occurred.")

            print("\nRoutes:")
            for route in final_solution:
                print(route)
        else:
            print("No solution was returned.")

    except FileNotFoundError:
        print(f"\nERROR: Test instance file not found at '{instance_json_file}'")
        print("Please check the path in the `if __name__ == '__main__'` block.")
    except Exception as e:
        print(f"An error occurred during the test run: {e}")