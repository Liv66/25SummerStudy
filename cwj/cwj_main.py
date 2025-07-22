import json
from cwj_proposed_algorithm import run_column_generation

def cwj_main(problem_info):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    # 파라미터 확인
    print("========== [DEBUG] Problem Info ==========")
    print(f"Number of vehicles (K): {K}")
    print(f"Vehicle capacity (Q): {capa}")
    print(f"Total number of nodes (N): {len(node_type)}")
    print(f"Number of linehaul customers: {node_type.count(1)}")
    print(f"Number of backhaul customers: {node_type.count(2)}")
    print(f"Depot index (always 0): demand={node_demand[0]}, type={node_type[0]}")
    print(f"Distance matrix size: {len(dist_mat)} x {len(dist_mat[0])}")
    print("==========================================")

    # 전체 route 생성 및 column generation 수행
    solution_routes = run_column_generation(problem_info)

    # Construction 객체 없이 route 리스트 반환
    sol = [route for route in solution_routes]
    return sol

if __name__ == "__main__":
    instance_path = "/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/problem_20_0.7.json"
    print(f"[INFO] Loading instance from: {instance_path}")
    
    with open(instance_path, "r", encoding="utf-8") as f:
        problem_info = json.load(f)

    # 🔍 디버깅: 로드된 JSON 키 확인
    print(f"[DEBUG] Loaded keys: {list(problem_info.keys())}")

    sol = cwj_main(problem_info)

    print("\n========== Final Routes ==========")
    for idx, route in enumerate(sol):
        print(f"Route {idx + 1}: {route}")