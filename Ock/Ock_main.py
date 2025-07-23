from Ock_ALNS import ALNS
import Ock_heuristic as heu
from Ock_heuristic import destroy_solution
from Ock_heuristic import repair_solution
import Ock_input_convert as input_convert
import time
from Ock_draw import *
import Ock_input_convert as ic
import pandas as pd

def Ock_main(problem_info, iterations=10000, start_temperature=1000, cooling_rate=0.99, stop_no_improvement=1000):

    # 파라미터 설정
    nodes, NUM_LINEHAUL, NUM_BACKHAUL, NUM_VEHICLES, CAPACITY = ic.convert_data(problem_info)

    # 노드와 비용 행렬 생성
    cost_matrix = ic.calculate_cost(nodes = nodes)

    initial_routes = heu.init_solution(nodes, NUM_VEHICLES, CAPACITY, cost_matrix)
    # initial_cost = heu.calculate_total_cost(solution=initial_routes, Cost_matrix = cost_matrix)

    # 3. 파괴 및 재구성 객체 생성
    destroyer = destroy_solution(nodes=nodes, NUM_VEHICLES=NUM_VEHICLES, CAPACITY=CAPACITY, Cost_matrix=cost_matrix)
    repairer = repair_solution(nodes=nodes, NUM_VEHICLES=NUM_VEHICLES, CAPACITY=CAPACITY, NUM_LINEHAUL=NUM_LINEHAUL, NUM_BACKHAUL=NUM_BACKHAUL, Cost_matrix=cost_matrix)

    solver = ALNS(
        initial_solution=initial_routes,
        nodes=nodes,
        destroyer=destroyer,
        repairer=repairer,
        Cost_matrix = cost_matrix
    )

    start_time = time.time()
    # ALNS 실행
    best_routes, best_cost = solver.run(iterations, start_temperature, cooling_rate, max_no_improvement = stop_no_improvement)
    end_time = time.time()

    elapsed_time = end_time - start_time

    # print(f"개선율: {(initial_cost - best_cost) / initial_cost * 100:.2f}%")

    return best_routes, best_cost, elapsed_time, nodes

if __name__ == "__main__":
    # JSON 파일에서 문제 정보 로드
    prob_list = ['problem_20_0.7.json', 'problem_30_0.7.json', 'problem_100_0.7.json']
    for i in prob_list:
        problem_info = ic.load_from_json(rf"C:\Users\옥중석\Desktop\25SummerStudy\instances\{i}")
        
        # Ock_main 함수 실행
        sol, cost, epalsed_time, nodes = Ock_main(problem_info, iterations=10000, start_temperature=1000, cooling_rate=0.99, stop_no_improvement=2000)
        print(f"\n--- {i} 최종 결과 ---")
        print(f"{i} 총 실행 시간: {epalsed_time:.2f}초")
        print(f"{i} 최종 비용: {cost:.2f}")
        print(f"{i} 최적 경로: {sol}")
        draw_routes(nodes, sol)


# 각 node당 30번 반복 실험하려고 만든 코드
# 실행하고 싶다면 __name__ == "__main__"이 되게
if __name__ != "__main__":
    # JSON 파일에서 문제 정보 로드
    prob_list = ['problem_20_0.7.json', 'problem_30_0.7.json', 'problem_100_0.7.json']
    result = []
    for i in prob_list:
        i_result = []
        for j in range(30):
            
            problem_info = ic.load_from_json(rf"C:\Users\user\OneDrive - pusan.ac.kr\바탕 화면\옥중석\학교\대학원\여름 방학 스터디\code\{i}")
            
            # Ock_main 함수 실행
            sol, cost, epalsed_time, nodes = Ock_main(problem_info, iterations=10000, start_temperature=1000, cooling_rate=0.99, stop_no_improvement=2000)
            # print(f"\n--- {i} 최종 결과 ---")
            # print(f"{i} 총 실행 시간: {epalsed_time:.2f}초")
            # print(f"{i} 최종 비용: {cost:.2f}")
            # print(f"{i} 최적 경로: {sol}")
            # draw_routes(nodes, sol)
            i_result.append([cost, epalsed_time])

        result.append(i_result)

    data_for_csv = []

    # 전체 실험 번호를 위한 카운터

    # 각 '입력값 그룹'을 순회
    for input_group_idx, group_of_experiments in enumerate(result):
        # 각 '입력값 그룹' 내의 '실험 결과'들을 순회
        for single_experiment_data in group_of_experiments:
            # 각 실험 데이터에서 비용과 시간을 추출
            cost = single_experiment_data[0] # 첫 번째 값은 비용
            time_taken = single_experiment_data[1] # 두 번째 값은 시간

            data_for_csv.append([
                input_group_idx + 1,          # 몇 번째 입력값 그룹인지 (1부터 시작)
                cost,                         # 비용
                time_taken                    # 시간
            ])
            

    # 리스트를 Pandas DataFrame으로 변환
    df_results = pd.DataFrame(data_for_csv, 
                            columns=['실험_번호', '비용', '시간'])


    output_csv_filename = '실험_결과_데이터_점진적으로 감소하게.csv'
    df_results.to_csv(output_csv_filename, index=False, encoding='utf-8-sig')

    print(f"'{output_csv_filename}' 파일이 성공적으로 생성되었습니다.")
            
    # 통계량 출력
    for value in range(2):  # 0: 비용, 1: 시간
        statistic_result(result, value)