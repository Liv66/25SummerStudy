import time # Ensure this is at the very top
import json
import os
import glob
import csv
import matplotlib.pyplot as plt

from OSM_util import plot_cvrp
from OSM_ACO import ACO_VRPB
import numpy

class NumpyEncoder(json.JSONEncoder):
    """ NumPy 타입을 파이썬 기본 타입으로 변환해주는 Custom Encoder """
    def default(self, obj):
        if isinstance(obj, numpy.integer):
            return int(obj)
        elif isinstance(obj, numpy.floating):
            return float(obj)
        elif isinstance(obj, numpy.ndarray):
            return obj.tolist()
        return super(NumpyEncoder, self).default(obj)
    
# Your validate_solution function
def validate_solution(solution_routes, K, capa, demands, node_types, execution_time, time_limit=60):
    """
    최종 해답이 주어진 제약 조건들을 모두 만족하는지 검증하는 함수
    """
    print("\n--- 최종 해답 검증 시작 ---")
    
    # demands 딕셔너리의 키를 이용하여 node_types 재구성
    # JSON에서 읽어온 node_types가 리스트인 경우를 대비하여 딕셔너리로 변환
    # 문제 파일에 따라 node_types의 구조가 달라질 수 있으므로, 유연하게 처리
    if isinstance(node_types, list):
        temp_node_types_dict = {i: node_type for i, node_type in enumerate(node_types)}
    else: # 이미 딕셔너리 형태일 경우
        temp_node_types_dict = node_types

    all_customer_nodes = {k for k, v in temp_node_types_dict.items() if k != 0}
    
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
    visited_nodes_list = []
    for path in solution_routes:
        for node in path:
            visited_nodes_list.append(node)
    visited_nodes_set = set(visited_nodes_list)

    # 7. Depot을 제외하고 두 번 이상 방문하는 노드가 있는가?
    if len(visited_nodes_list) != len(visited_nodes_set):
        print("7. 노드 중복 방문: [FAIL]")
    else:
        print("7. 노드 중복 방문: PASS")

    # 9. 방문하지 않은 노드가 존재하나?
    unvisited = all_customer_nodes - visited_nodes_set
    if unvisited:
        print(f"9. 미방문 노드 존재: [FAIL] - 미방문 노드: {unvisited}")
    else:
        print("9. 미방문 노드 존재: PASS")

    # 경로별(차량별) 제약 조건 확인
    all_paths_valid = True
    for i, path in enumerate(solution_routes):
        # 3, 4. Depot 방문 규칙은 경로 표현 방식상 항상 만족 (고객 노드만 포함)
        
        # 5. 각 차량은 linehaul을 반드시 방문하는가? (경로가 비어있지 않다면)
        has_linehaul = False
        for node in path:
            if temp_node_types_dict.get(node) == 1: # Linehaul
                has_linehaul = True
                break
        if path and not has_linehaul:
            print(f"5. Linehaul 필수 방문 (차량 {i+1}): [FAIL] - 경로에 Linehaul 노드가 없습니다.")
            all_paths_valid = False

        # 6. 각 차량에 linehaul -> backhaul -> linehaul 경우가 있는가?
        in_backhaul_phase = False
        is_order_violated = False
        for node in path:
            if temp_node_types_dict.get(node) == 2: # Backhaul
                in_backhaul_phase = True
            if temp_node_types_dict.get(node) == 1 and in_backhaul_phase: # Backhaul 후 Linehaul
                print(f"6. 방문 순서 (차량 {i+1}): [FAIL] - Backhaul 후 Linehaul 방문 시도 (노드: {node})")
                all_paths_valid = False
                is_order_violated = True
                break
        if not is_order_violated:
            print(f"6. 방문 순서 (차량 {i+1}): PASS")
            
        # 8. 각 차량은 Line과 back의 용량을 만족하였나?
        linehaul_load = sum(demands[n] for n in path if temp_node_types_dict.get(n) == 1)
        backhaul_load = sum(abs(demands[n]) for n in path if temp_node_types_dict.get(n) == 2)
        
        if linehaul_load > capa or backhaul_load > capa:
            print(f"8. 용량 만족 (차량 {i+1}): [FAIL] - 배송량:{linehaul_load}/{capa}, 반송량:{backhaul_load}/{capa})")
            all_paths_valid = False
        else:
            print(f"8. 용량 만족 (차량 {i+1}): PASS (배송량:{linehaul_load}/{capa}, 반송량:{backhaul_load}/{capa})")
    
    if all_paths_valid:
        print("개별 경로 제약 조건 (5, 6, 8): 모두 PASS")
        
    print("--- 검증 종료 ---\n")



def save_experiment_results(instance_name, run_index, execution_time, final_solution, all_node_coord, demands, node_types_dict, K, capa):
    """
    실험 결과를 JSON, CSV, Plot 이미지로 저장합니다.
    """
    # 저장할 기본 이름 생성
    base_filename = f"{instance_name}_run_{run_index}"

    # --- 1. JSON 상세 데이터 저장 ---
    json_dir = "results/json_details"
    os.makedirs(json_dir, exist_ok=True)
    json_path = os.path.join(json_dir, f"{base_filename}.json")
    
    result_data_json = {
        "experiment_name": instance_name,
        "run_index": run_index,
        "execution_time": round(execution_time, 2),
    }
    
    if final_solution and final_solution[1] != float('inf'):
        best_routes, best_distance = final_solution
                # 저장하기 전에 각 경로의 시작과 끝에 차고지(0)를 추가합니다.
        full_routes_with_depot = [[0] + path + [0] for path in best_routes]

        result_data_json["final_objective"] = round(best_distance, 2)
        result_data_json["final_routes"] = full_routes_with_depot
    else:
        result_data_json["final_objective"] = None
        result_data_json["final_routes"] = None

    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(result_data_json, f, ensure_ascii=False, indent=4, cls=NumpyEncoder)
    print(f"결과 JSON 저장 완료: {json_path}")

    # --- 2. CSV 요약 데이터 생성 (나중에 한 번에 저장) ---
    summary_row = {
        "instance_name": instance_name,
        "run_index": run_index,
        "execution_time": round(execution_time, 2),
        "final_objective": result_data_json["final_objective"]
    }

    # --- 3. 경로 시각화 그래프 저장 ---
    if final_solution and final_solution[1] != float('inf'):
        plot_dir = "results/plots"
        os.makedirs(plot_dir, exist_ok=True)
        plot_path = os.path.join(plot_dir, f"{base_filename}.png")
        
        best_routes, best_distance = final_solution
        
        # plot_cvrp 함수를 호출하여 그림을 생성
        plot_cvrp(
            nodes_coord=all_node_coord,
            best_result_route=best_routes,
            demands=demands,
            title=f'{instance_name} (Run {run_index})\nTotal Distance: {best_distance:.2f}'
        )
        # 생성된 그림을 파일로 저장하고 닫기
        plt.savefig(plot_path)
        plt.close()
        print(f"결과 그래프 저장 완료: {plot_path}")
        
        # --- 4. 해답 검증 ---
        validate_solution(best_routes, K, capa, demands, node_types_dict, execution_time)
    else:
        print("유효한 해를 찾지 못하여 그래프 생성 및 검증을 건너뜁니다.")
        validate_solution([], K, capa, demands, node_types_dict, execution_time)

    return summary_row


def run_single_experiment(aco_class, instance_path, run_index):
    """
    단일 인스턴스에 대해 한 번의 실험을 실행하고 결과를 반환합니다.
    """
    print(f"\n--- 실험 시작: {os.path.basename(instance_path)} (실행 {run_index}) ---")

    # 1. JSON 파일에서 문제 데이터 읽어오기
    with open(instance_path, 'r', encoding='utf-8') as f:
        problem_data = json.load(f)

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
    
    # 2. ACO 솔버 생성 및 실행
    aco_solver = aco_class(
        iterations=10000, 
        ants=K, 
        alpha=1, 
        beta=5,
        ro=0.8,
        stagnation_limit=3, 
        perturbation_strength=1
    )
    
    start_time = time.time()
    # 하이브리드 로직이 적용된 solve_mmas 함수를 호출합니다.
    final_solution = aco_solver.solve_mmas(K, capa, all_node_coord, demands)
    end_time = time.time()
    execution_time = end_time - start_time
    
    print(f"--- 실험 종료: {os.path.basename(instance_path)} (실행 {run_index}) | 소요 시간: {execution_time:.2f}초 ---")
    
    return final_solution, execution_time, all_node_coord, demands, node_types_dict, K, capa


def OSM_main():
    """
    'instances' 폴더의 모든 문제에 대해 실험을 자동 실행합니다.
    """
    # --- 1. 실험 환경 설정 ---
    instances_dir = 'instances'
    instance_files = glob.glob(os.path.join(instances_dir, '*.json'))
    num_runs_per_instance = 3
    all_summary_data = []

    if not instance_files:
        print(f"'{instances_dir}' 폴더에 실험할 .json 파일이 없습니다. 스크립트를 종료합니다.")
        return

    print(f"총 {len(instance_files)}개의 인스턴스에 대해 각 {num_runs_per_instance}회 실험을 시작합니다.")
    
    # --- 2. 모든 인스턴스에 대한 실험 루프 ---
    for instance_path in instance_files:
        instance_name = os.path.splitext(os.path.basename(instance_path))[0]
        
        for run_index in range(1, num_runs_per_instance + 1):
            # 단일 실험 실행
            final_solution, exec_time, nodes_coord, demands, node_types, K, capa = run_single_experiment(ACO_VRPB, instance_path, run_index)
            
            # 결과 저장 및 CSV 데이터 수집
            summary_row = save_experiment_results(instance_name, run_index, exec_time, final_solution, nodes_coord, demands, node_types, K, capa)
            all_summary_data.append(summary_row)

    # --- 3. 최종 CSV 요약 파일 저장 ---
    csv_path = "results/experiment_summary.csv"
    os.makedirs("results", exist_ok=True)
    
    if all_summary_data:
        with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=all_summary_data[0].keys())
            writer.writeheader()
            writer.writerows(all_summary_data)
        print(f"\n\n모든 실험 요약 CSV 파일 저장 완료: {csv_path}")
    
    print("\ 모든 실험이 성공적으로 완료되었습니다. ")


if __name__ == '__main__':
    # 메인 함수로 전체 실험 프레임워크를 호출
    start_overall_time = time.time()
    OSM_main()
    end_overall_time = time.time()
    print(f"\n--- 총 실행 시간: {end_overall_time - start_overall_time:.2f} 초 ---")





# def OSM_main():
#     """
#     VRPB 문제를 정의하고, ACO 솔버로 해결한 뒤, 결과를 출력/시각화합니다.
#     """
#     # --- 1. JSON 파일에서 문제 데이터 읽어오기 ---
#     file_path = 'C:/Users/risklab/Desktop/code/25SummerStudy/instances/problem_150_0.5.json'
#     file_path = "instances\problem_150_0.5.json"
#     with open(file_path, 'r', encoding='utf-8') as f:
#         problem_data = json.load(f)

#     # JSON 데이터로부터 파라미터 로드
#     N = problem_data['N']
#     K = problem_data['K']
#     capa = problem_data['capa']
#     all_node_coord = problem_data['node_coords']
    
#     demands = []
#     node_types_dict = {}
#     for i, node_type_val in enumerate(problem_data['node_types']):
#         demand_value = problem_data['node_demands'][i]
#         demands.append(demand_value if node_type_val == 1 else -demand_value if node_type_val == 2 else 0)
#         node_types_dict[i] = node_type_val

#     print(f"문제 파일 로드 완료: {file_path}")
#     print("ACO_VRPB 솔버를 시작합니다...")
    
#     aco_solver = ACO_VRPB(iterations=1000, ants=K, q0= 0.8 , alpha=1, beta=5)
    
#     start_time = time.time() # This should now be fine
#     ACO_solution = aco_solver.solve_mmas(K, capa, all_node_coord, demands)
#     print(f"ACO_solution 결과: {ACO_solution}")
#     end_time = time.time()
#     execution_time = end_time - start_time 

#     # --- 3. 결과 처리 및 시각화 ---
#     if ACO_solution and ACO_solution[1] != float('inf'):
#         best_routes, best_distance = ACO_solution

#         print("\n--- 최종 결과 ---")
#         modified_best_routes_for_plot = [] # 이 리스트는 plot_cvrp를 위해 full_route를 저장합니다.
#         for idx, route in enumerate(best_routes):
#             full_route = [0] + route + [0] # 출발지(Depot 0)와 복귀지(Depot 0)를 추가합니다.
#             modified_best_routes_for_plot.append(full_route) # 시각화를 위한 리스트에 추가합니다.
#         for idx, route in enumerate(best_routes):
#             full_route = [0] + route + [0] # 출발지(Depot 0)와 복귀지(Depot 0)를 추가합니다.
#             modified_best_routes_for_plot.append(full_route) # 시각화를 위한 리스트에 추가합니다.
#             print(f'Vehicle {idx+1} route: {full_route}') # 여기서 full_route를 출력합니다.
#         print(f'>>> Total objective distance: {best_distance:.2f}')
#         print("\n최적 경로를 시각화합니다.")
#         plot_cvrp(
#             nodes_coord=all_node_coord,
#             best_result_route=best_routes, # plot_cvrp는 내부적으로 depot을 추가하므로 depot제외 경로 전달
#             demands=demands,
#             title=f'ACO_VRPB Solution ({file_path}) | Total Distance: {best_distance:.2f}'
#         )
#         # --- 4. 최종 해답 검증 실행 ---
#         validate_solution(best_routes, K, capa, demands, node_types_dict, execution_time)
#     else:
#         print("\n알고리즘이 제약 조건을 만족하는 해답을 찾지 못했습니다.")
#         # Even if no valid solution is found, you might want to log this or perform partial validation
#         validate_solution([], K, capa, demands, node_types_dict, execution_time) # Pass empty routes for failed solution

# if __name__ == '__main__':
#     start_overall_time = time.time() # This should also be fine
#     OSM_main()
#     end_overall_time = time.time()
#     print(f"\n--- 총 실행 시간: {end_overall_time - start_overall_time:.2f} 초 ---")
