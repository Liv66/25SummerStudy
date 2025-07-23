import time
import json

from OSM_util import plot_cvrp
from OSM_ACO import ACO_VRPB

def OSM_main():
    """
    VRPB 문제를 정의하고, ACO 솔버로 해결한 뒤, 결과를 출력/시각화합니다.
    """
    # --- 1. JSON 파일에서 문제 데이터 읽어오기 ---
    file_path = 'C:/Users/risklab/Desktop/code/25SummerStudy/instances/problem_100_0.7.json' # JSON 파일 경로
    
    with open(file_path, 'r', encoding='utf-8') as f:
        problem_data = json.load(f)

    # JSON 데이터로부터 파라미터 로드
    N = problem_data['N']
    K = problem_data['K']
    capa = problem_data['capa']
    all_node_coord = problem_data['node_coords']
    
    # node_types를 기반으로 demands 리스트 생성
    demands = []
    for i, node_type in enumerate(problem_data['node_types']):
        demand_value = problem_data['node_demands'][i]
        if node_type == 0: demands.append(0)
        elif node_type == 1: demands.append(demand_value)
        elif node_type == 2: demands.append(-demand_value)

    # --- 2. ACO_VRPB 객체 생성 및 문제 해결 ---
    print(f"문제 파일 로드 완료: {file_path}")
    print("ACO_VRPB 솔버를 시작합니다...")
    
    # 알고리즘 파라미터 설정
    aco_solver = ACO_VRPB(iterations=50, ants=K, q0=0.9, alpha=1, beta=5)

    # solve 메소드에 K를 포함한 모든 정보를 정확히 전달
    ACO_solution = aco_solver.solve(K, capa, all_node_coord, demands)

    # --- 3. 결과 처리 및 시각화 ---
    if ACO_solution and ACO_solution[1] != float('inf'):
        best_routes, best_distance = ACO_solution
        
        print("\n--- 최종 결과 ---")
        modified_best_routes_for_plot = []
        for idx, route in enumerate(best_routes):
            full_route = [0] + route + [0]
            modified_best_routes_for_plot.append(full_route)
            print(f'Vehicle {idx+1} route: {route}')
        print(f'>>> Total objective distance: {best_distance:.2f}')

        print("\n최적 경로를 시각화합니다.")
        plot_cvrp(
            nodes_coord=all_node_coord,
            best_result_route=best_routes, # plot_cvrp가 내부적으로 depot을 추가하므로 depot제외 경로 전달
            demands=demands,
            title=f'ACO_VRPB Solution ({file_path}) | Total Distance: {best_distance:.2f}'
        )
    else:
        print("\n알고리즘이 제약 조건을 만족하는 해답을 찾지 못했습니다.")


if __name__ == '__main__':
    start_time = time.time()
    OSM_main()
    end_time = time.time()
    print(f"\n--- 총 실행 시간: {end_time - start_time:.2f} 초 ---")