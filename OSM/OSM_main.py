import random

from OSM_util import plot_cvrp
from OSM_ACO import ACO_VRPB

def OSM_main():
    N = 50  # 방문할 노드 수
    capa = 5000  # 차량들의 용량
    nodes_coord = [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(N)]  # 노드 좌표
    depot_coord = (12000, 16000)  # depot 노드 좌표
    all_node_coord = [depot_coord] + nodes_coord    # depot 노드를 포함한 전체 노드 좌표

    # VRPB용 수요량 (배송: 양수, 반송: 음수)
    linehaul_ratio = 0.80  # 50%, 66%, 80%로 설정
    n_linehaul = int(N * linehaul_ratio)
    n_backhaul = N - n_linehaul

    linehaul_demands = [max(1, int(random.gauss(500, 200))) for _ in range(n_linehaul)]
    backhaul_demands = [-max(1, int(random.gauss(500, 200))) for _ in range(n_backhaul)]
    customer_demands = linehaul_demands + backhaul_demands
    random.shuffle(customer_demands)
    demands = [0] + customer_demands    # 전체 수요량 리스트 (0번 Depot 수요는 0)
    
    #########풀기#########
    ACO_solver = ACO_VRPB(iterations=200, ants=N)

    ACO_solution = ACO_solver.solve(N+1, capa, all_node_coord, demands)  # ACO로 VRPB 문제 해결

    if ACO_solution: # final_solution이 None이 아닌 경우(해를 찾은 경우)
        # .solve()가 반환한 최종 해답을 변수에 저장
        best_routes, best_distance = ACO_solution

        print("\n--- 최종 결과 ---")
        for idx, route in enumerate(best_routes):
            print(f'Vehicle {idx+1} route: {route}') # 차량 번호는 1번부터 시작하도록 idx+1 사용
        print(f'>>> Total objective distance: {best_distance:.2f}')

        # 최종 결과 시각화
        print("\n최적 경로를 시각화합니다.")
        plot_cvrp(
        nodes_coord=all_node_coord,       # 'all_node_coord' -> 'nodes_coord'로 수정
        best_result_route=best_routes,  # 'result_route' -> 'best_result_route'로 수정
        title=f'ACO_VRPB Solution | Total Distance: {best_distance:.2f}'
        )
    else:
        print("알고리즘이 해답을 찾지 못했습니다.")


if __name__ == '__main__':
    OSM_main()
