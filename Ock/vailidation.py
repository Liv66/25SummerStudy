import json
import Ock_input_convert as ic
import os
import pandas as pd
import Ock_input_convert as ic

def check_feasible(problem_info, sol, elapsed, timelimit):
    nodes_list, NUM_LINEHAUL, NUM_BACKHAUL, NUM_VEHICLES, CAPACITY = ic.convert_data(problem_info)
    # 2. 받아온 리스트를 pandas DataFrame으로 변환합니다.
    dist_mat = ic.calculate_cost(nodes_list)

    nodes = pd.DataFrame(nodes_list)

    # 3. 이제 기존 코드가 정상적으로 작동합니다.
    type_map = {'depot': 0, 'linehaul': 1, 'backhaul': 2 }
    nodes['type'] = nodes['type'].map(type_map)

    K = NUM_VEHICLES
    node_type = nodes['type']
    node_demand = nodes['demand']
    capa = CAPACITY
    

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
        if route[0] != 0:
            print(f"depot에서 출발해야 합니다. {idx}번째 차량 경로 {route}")
            return 0

        if route[-1] != 0:
            print(f"depot으로 도착해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if node_type[route[1]] == 2:
            print(f"차량은 backhaul만 갈 수 없습니다. {idx}번째 차량 경로 {route}")
            return 0
        cost = 0
        load = 0

        pre = 0
        flag = False
        route_type = [0] * len(route)

        for i in range(1, len(route) - 1):
            nxt = route[i]
            if visit[nxt]:
                print(f"{nxt} 노드 2번 이상 방문")
                return 0
            visit[nxt] = 1
            cost += dist_mat[pre][nxt]
            load += node_demand[nxt]
            route_type[i] = node_type[nxt]
            if nxt == 0:
                print(f"{idx}번째 차량 depot을 3번 이상 방문 {route}")
                return 0

            if node_type[pre] == 1 and node_type[nxt] == 2:
                if flag:
                    print(f"{idx}번째 차량 line -> backhaul -> line 경로 존재")
                    print(node_type)
                    return 0
                flag = True
                load = 0

            if load > capa:
                if flag:
                    print(f"{idx}번째 차량의 back 용량 초과 {load}, {capa}")
                else:
                    print(f"{idx}번째 차량의 line 용량 초과 {load}, {capa}")
                return 0
            pre = nxt
        cost += dist_mat[pre][0]
        total_cost += cost

    if sum(visit) < len(node_type):
        yet_visit = [i for i in range(len(node_type)) if not visit[i]]
        print(f"다음 노드들을 방문하지 않았습니다. {yet_visit}")
        return 0

    return total_cost



if __name__ == "__main__":
    # 1. 검증할 결과 파일 경로 설정
    results_file_path = os.path.join("results", "summary_of_all_results.jsonl")

    # 검증 결과 카운터
    total_runs = 0
    feasible_runs = 0
    infeasible_runs = 0

    print(f"--- '{results_file_path}' 파일 전체 검증 시작 ---")

    try:
        with open(results_file_path, 'r') as f:
            for i, result_line in enumerate(f):
                total_runs += 1
                line_num = i + 1
                
                try:
                    # 2. 한 줄씩 읽어 JSON으로 파싱
                    result_data = json.loads(result_line)
                    problem_name = result_data["problem_name"]
                    run_number = result_data["run_number"]
                    
                    print(f"\n--- [ {line_num}번째 줄 ] 문제: {problem_name}, 실행: {run_number} 검증 ---")

                    # 3. 원본 문제 데이터 로드
                    problem_file_path = problem_name.replace('\\\\', '\\')
                    with open(problem_file_path, 'r') as p_file:
                        problem_data = json.load(p_file)

                    # 4. check_feasible 함수에 필요한 인자 준비
                    solution_routes = result_data["solution_routes"]
                    elapsed_time = result_data["elapsed_time"]
                    time_limit = 60  # 시간 제한 60초로 고정

                    # 5. 검증 함수 실행
                    calculated_cost = check_feasible(
                        problem_info=problem_data,
                        sol=solution_routes,
                        elapsed=elapsed_time,
                        timelimit=time_limit
                    )

                    # 6. 검증 결과에 따라 카운트 및 출력
                    if calculated_cost > 0:
                        feasible_runs += 1
                        print(f"[성공] 제약 조건 만족. 계산된 비용: {calculated_cost:.2f}")
                        
                        # 파일에 기록된 비용과 비교
                        reported_cost = result_data.get("final_cost")
                        if reported_cost is not None:
                            print(f"    (솔루션 기록 비용: {reported_cost})")
                            if abs(calculated_cost - reported_cost) > 1:
                                print("    (!) 주의: 계산된 비용과 기록된 비용이 다릅니다!")
                    else:
                        infeasible_runs += 1
                        # 실패 메시지는 check_feasible 함수 내부에서 출력됨

                except FileNotFoundError:
                    infeasible_runs += 1
                    print(f"[오류] {line_num}번째 줄: 문제 파일 '{problem_file_path}'를 찾을 수 없습니다.")
                except json.JSONDecodeError:
                    infeasible_runs += 1
                    print(f"[오류] {line_num}번째 줄: JSON 형식이 올바르지 않습니다.")
                except KeyError as e:
                    infeasible_runs += 1
                    print(f"[오류] {line_num}번째 줄: 데이터에서 '{e}' 키를 찾을 수 없습니다.")
                except Exception as e:
                    infeasible_runs += 1
                    print(f"[알 수 없는 오류] {line_num}번째 줄: {e}")

    except FileNotFoundError:
        print(f"[치명적 오류] 결과 파일 '{results_file_path}'를 찾을 수 없습니다. 경로를 확인해주세요.")

    # 최종 요약 출력
    print("\n\n" + "="*50)
    print("--- 최종 검증 요약 ---")
    print(f"  - 총 실행 횟수: {total_runs}")
    print(f"  - 성공 (Feasible): {feasible_runs}")
    print(f"  - 실패 (Infeasible): {infeasible_runs}")
    print("="*50)