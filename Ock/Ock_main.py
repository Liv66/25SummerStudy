from Ock.Ock_ALNS import ALNS
import Ock.Ock_heuristic as heu
from Ock.Ock_heuristic import destroy_solution
from Ock.Ock_heuristic import repair_solution
import Ock.Ock_input_convert as input_convert
import time
from Ock.Ock_draw import *
import Ock.Ock_input_convert as ic
import pandas as pd
import os
import json
import pandas as pd
import glob
import Ock.validation as v


def Ock_run(problem_info, iterations=1000000, start_temperature=1000, cooling_rate=0.99, max_no_improvement=1000):

    # 파라미터 설정
    nodes, NUM_LINEHAUL, NUM_BACKHAUL, NUM_VEHICLES, CAPACITY, inverse_id_map = ic.convert_data(problem_info)

    # 노드와 비용 행렬 생성
    cost_matrix = ic.calculate_cost(nodes = nodes)

    initial_routes = heu.init_solution(nodes, NUM_VEHICLES, CAPACITY, cost_matrix)

    # 3. 파괴 및 재구성 객체 생성
    destroyer = destroy_solution(nodes=nodes, NUM_VEHICLES=NUM_VEHICLES, CAPACITY=CAPACITY,  NUM_LINEHAUL=NUM_LINEHAUL, NUM_BACKHAUL=NUM_BACKHAUL, Cost_matrix=cost_matrix)
    repairer = repair_solution(nodes=nodes, NUM_VEHICLES=NUM_VEHICLES, CAPACITY=CAPACITY, NUM_LINEHAUL=NUM_LINEHAUL, NUM_BACKHAUL=NUM_BACKHAUL, Cost_matrix=cost_matrix)

    solver = ALNS(
        initial_solution=initial_routes,
        nodes=nodes,
        destroyer=destroyer,
        repairer=repairer,
        Cost_matrix = cost_matrix
    )

    best_routes, best_cost = solver.run(iterations, start_temperature, cooling_rate, max_no_improvement = max_no_improvement)
    sol = ic.convert_solution(best_routes, inverse_id_map)
    return sol


if __name__ == "__main__":
    # JSON 파일에서 문제 정보 로드

    search_pattern = os.path.join('instances', '*.json')
    problem_filepaths = glob.glob(search_pattern)

    for i in problem_filepaths:
        problem_name = os.path.basename(i)
        print(f"\n--- 문제 '{problem_name}' 처리 중... ---")
        problem_info = ic.load_from_json(i)
        
        # Ock_main 함수 실행
        sol = Ock_run(problem_info)
        print(f"\n--- {i} 최종 결과 ---")
        print(f"{i} 최적 경로: {sol}")

        if v.check_feasible(problem_info, sol, 57, 60) == 0:
            print(f"Validation failed for {problem_name}")
            print(f'{problem_name} sol is not feasibles') 
