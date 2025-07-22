import random
import time
import json

from OSM_util import plot_cvrp
from OSM_ACO import ACO_VRPB

def validate_solution(solution_routes, K, capa, demands, execution_time, time_limit=60):
    """
    최종 해답이 주어진 제약 조건들을 모두 만족하는지 검증하는 함수
    """
    print("\n--- 최종 해답 검증 시작 ---")
    
    demands_dict = {i: d for i, d in enumerate(demands)}
    all_customer_nodes = {k for k, v in demands_dict.items() if v != 0}
    
    # 1. 제한 시간을 넘겼는가?
    if execution_time > time_limit:
        print(f"1. 시간 제한({time_limit}초): [FAIL] - 소요 시간: {execution_time:.2f}초")
    else:
        print(f"1. 시간 제한({time_limit}초): PASS (소요 시간: {execution_time:.2f}초)")

    # 2. 사용한 차량의 수가 K대인가?
    if len(solution_routes) != K:
        print(f"2. 차량 수 K({K}대): [FAIL] - {len(solution_routes)}대 사용")
    else:
        print(f"2. 차량 수 K({K}대): PASS")

    # 방문한 모든 고객 노드 집합 생성 (중복 및 미방문 확인용)
    visited_nodes_list = [node for path in solution_routes for node in path]
    visited_nodes_set = set(visited_nodes_list)

    # 7. Depot을 제외하고 두 번 이상 방문하는 노드가 있는가?
    if len(visited_nodes_list) != len(visited_nodes_set):
        print("7. 노드 중복 방문: [FAIL]")
    else:
        print("7. 노드 중복 방문: PASS")

    # 9. 방문하지 않은 노드가 존재하나?
    if visited_nodes_set != all_customer_nodes:
        unvisited = all_customer_nodes - visited_nodes_set
        print(f"9. 미방문 노드 존재: [FAIL] - 미방문 노드: {unvisited}")
    else:
        print("9. 미방문 노드 존재: PASS")

    # 경로별(차량별) 제약 조건 확인
    all_paths_valid = True
    for i, path in enumerate(solution_routes):
        # 5. 각 차량은 linehaul을 반드시 방문하는가? (경로가 비어있지 않다면)
        if path and not any(demands_dict.get(node, 0) > 0 for node in path):
            print(f"5. Linehaul 필수 방문 (차량 {i+1}): [FAIL]")
            all_paths_valid = False

        # 6. 각 차량에 linehaul -> backhaul -> linehaul 경우가 있는가?
        in_backhaul_phase = False
        for node in path:
            if demands_dict.get(node, 0) < 0:
                in_backhaul_phase = True
            if demands_dict.get(node, 0) > 0 and in_backhaul_phase:
                print(f"6. 방문 순서 (차량 {i+1}): [FAIL] - Backhaul 후 Linehaul 방문")
                all_paths_valid = False
                break
        
        # 8. 각 차량은 Line과 back의 용량을 만족하였나?
        linehaul_load = sum(demands_dict.get(n, 0) for n in path if demands_dict.get(n, 0) > 0)
        backhaul_load = sum(abs(demands_dict.get(n, 0)) for n in path if demands_dict.get(n, 0) < 0)
        if linehaul_load > capa or backhaul_load > capa:
            print(f"8. 용량 만족 (차량 {i+1}): [FAIL] - 배송량:{linehaul_load}, 반송량:{backhaul_load} (용량:{capa})")
            all_paths_valid = False
    
    if all_paths_valid:
        print("3,4,5,6,8. 개별 경로 제약 조건: PASS")
        
    print("--- 검증 종료 ---\n")

def OSM_main():
    """
    VRPB 문제를 JSON 파일에서 로드하고, ACO 솔버로 해결한 뒤, 결과를 검증/시각화합니다.
    """
    # --- 1. JSON 파일에서 문제 데이터 읽어오기 ---
    file_path = 'C:/Users/risklab/Desktop/code/25SummerStudy/instances/problem_100_0.7.json'
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            problem_data = json.load(f)
    except FileNotFoundError:
        print(f"오류: 파일 '{file_path}'을(를) 찾을 수 없습니다. 경로를 확인해주세요.")
        return

    N = problem_data['N']
    K = problem_data['K']
    capa = problem_data['capa']
    all_node_coord = problem_data['node_coords']
    
    demands = []
    for i, node_type in enumerate(problem_data['node_types']):
        demand_value = problem_data['node_demands'][i]
        if node_type == 0: demands.append(0)
        elif node_type == 1: demands.append(demand_value)
        elif node_type == 2: demands.append(-demand_value)

    # --- 2. ACO_VRPB 객체 생성 및 문제 해결 ---
    print(f"문제 파일 로드 완료: {file_path.split('/')[-1]}")
    print("ACO_VRPB 솔버를 시작합니다...")
    
    aco_solver = ACO_VRPB(iterations=500, ants=K, q0=0.9, alpha=1, beta=5)

    start_time = time.time()
    ACO_solution = aco_solver.solve(K, capa, all_node_coord, demands)
    end_time = time.time()
    execution_time = end_time - start_time

    # --- 3. 결과 처리, 검증 및 시각화 ---
    if ACO_solution and ACO_solution[1] != float('inf'):
        best_routes, best_distance = ACO_solution
        
        # 검증 함수 호출
        validate_solution(best_routes, K, capa, demands, execution_time)
        
        print("--- 최종 결과 ---")
        for idx, route in enumerate(best_routes):
            print(f'Vehicle {idx+1} route: {route}')
        print(f'>>> Total objective distance: {best_distance:.2f}')

        print("\n최적 경로를 시각화합니다.")
        plot_cvrp(
            nodes_coord=all_node_coord,
            best_result_route=best_routes,
            demands=demands,
            title=f'ACO_VRPB Solution ({file_path.split("/")[-1]}) | Total Distance: {best_distance:.2f}'
        )
    else:
        print(f"\n알고리즘이 제약 조건을 만족하는 해답을 찾지 못했습니다. (소요 시간: {execution_time:.2f}초)")


if __name__ == '__main__':
    OSM_main()