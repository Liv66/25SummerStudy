import time # Ensure this is at the very top
import json

from OSM_util import plot_cvrp
from OSM_ACO import ACO_VRPB


def OSM_main():
    """
    VRPB 문제를 정의하고, ACO 솔버로 해결한 뒤, 결과를 출력/시각화합니다.
    """
    # --- 1. JSON 파일에서 문제 데이터 읽어오기 ---
    file_path = 'C:/Users/risklab/Desktop/code/25SummerStudy/instances/problem_150_0.5.json'
    
    with open(file_path, 'r', encoding='utf-8') as f:
        problem_data = json.load(f)

    # JSON 데이터로부터 파라미터 로드
    N = problem_data['N']
    K = problem_data['K']
    capa = problem_data['capa']
    all_node_coord = problem_data['node_coords']
    
    demands = []
    node_types_dict = {}
    for i, node_type_val in enumerate(problem_data['node_types']):
        demand_value = problem_data['node_demands'][i]
        demands.append(demand_value if node_type_val == 1 else -demand_value if node_type_val == 2 else 0)
        node_types_dict[i] = node_type_val

    print(f"문제 파일 로드 완료: {file_path}")
    print("ACO_VRPB 솔버를 시작합니다...")
    
    aco_solver = ACO_VRPB(iterations=200, ants=K, q0=0.9, alpha=1, beta=5)
    
    start_time = time.time() # This should now be fine
    ACO_solution = aco_solver.solve(K, capa, all_node_coord, demands)
    print(f"ACO_solution 결과: {ACO_solution}")
    end_time = time.time()
    execution_time = end_time - start_time 

    # --- 3. 결과 처리 및 시각화 ---
    if ACO_solution and ACO_solution[1] != float('inf'):
        best_routes, best_distance = ACO_solution

        print("\n--- 최종 결과 ---")
        modified_best_routes_for_plot = [] # 이 리스트는 plot_cvrp를 위해 full_route를 저장합니다.
        for idx, route in enumerate(best_routes):
            full_route = [0] + route + [0] # 출발지(Depot 0)와 복귀지(Depot 0)를 추가합니다.
            modified_best_routes_for_plot.append(full_route) # 시각화를 위한 리스트에 추가합니다.
        for idx, route in enumerate(best_routes):
            full_route = [0] + route + [0] # 출발지(Depot 0)와 복귀지(Depot 0)를 추가합니다.
            modified_best_routes_for_plot.append(full_route) # 시각화를 위한 리스트에 추가합니다.
            print(f'Vehicle {idx+1} route: {full_route}') # 여기서 full_route를 출력합니다.
        print(f'>>> Total objective distance: {best_distance:.2f}')
        print("\n최적 경로를 시각화합니다.")
        plot_cvrp(
            nodes_coord=all_node_coord,
            best_result_route=best_routes, # plot_cvrp는 내부적으로 depot을 추가하므로 depot제외 경로 전달
            demands=demands,
            title=f'ACO_VRPB Solution ({file_path}) | Total Distance: {best_distance:.2f}'
        )
    else:
        print("\n알고리즘이 제약 조건을 만족하는 해답을 찾지 못했습니다.")

if __name__ == '__main__':
    start_overall_time = time.time() # This should also be fine
    OSM_main()
    end_overall_time = time.time()
    print(f"\n--- 총 실행 시간: {end_overall_time - start_overall_time:.2f} 초 ---")