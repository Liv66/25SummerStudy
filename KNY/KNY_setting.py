import random
from util import bin_packing, multiple_knapsack, get_distance, two_opt, plot_cvrp


def main():
    # ---------- 기본 파라미터 ----------
    N = 50           # 고객 노드 수
    capa = 5000      # 차량 용량
    # ----------------------------------

    # 1) 노드 좌표·수요 생성 (기존 그대로)
    nodes_coord = [(random.uniform(0, 24_000),
                    random.uniform(0, 32_000)) for _ in range(N)]
    demands = [int(random.gauss(500, 200)) for _ in range(N)]

    # 2) 노드 유형 60:40  (1=배송, 0=회수)  ★추가
    num_delivery = int(N * 0.6)
    num_pickup   = N - num_delivery
    node_types = [1] * num_delivery + [0] * num_pickup
    random.shuffle(node_types)

    delivery_idx = [i for i, t in enumerate(node_types) if t == 1]
    pickup_idx   = [i for i, t in enumerate(node_types) if t == 0]

    # 3) 배송/회수 별 최소 차량 수 계산  ★추가
    K_delivery = bin_packing([demands[i] for i in delivery_idx], capa)
    K_pickup   = bin_packing([demands[i] for i in pickup_idx],   capa)
    K = max(K_delivery, K_pickup)    # VRPB 상한선
    print(f'# of vehicles (upper-bound) {K}')

    #--------------------------------------------------------------------------------------------------------------------
    # 밑의 코드는 multiple knapsack 사용해서 해 생성 후 2-opt로 개선/ greedy insertion+ALNS 방법 적용하기 위해서 선 위 코드까지만 활용

    # 4) 기존 로직 그대로 진행
    knapsack_sol = multiple_knapsack(demands, capa, K)   # 경로 조합 탐색
    depot_coord = (12_000, 16_000)
    all_node_coord = nodes_coord + [depot_coord]
    dist_matrix = get_distance(all_node_coord)

    global_obj = 987654321
    best_result_route = []
    for packing_sol in knapsack_sol:
        obj = 0
        result_route = []
        for route in packing_sol:
            cost, opt_route = two_opt(N, [N] + route, dist_matrix, all_node_coord)
            result_route.append(opt_route)
            obj += cost
        if obj < global_obj:
            global_obj = obj
            best_result_route = result_route

    # 5) 결과 출력 (그대로)
    for idx, route in enumerate(best_result_route):
        print(f'vehicle {idx} route : {route}')
    plot_cvrp(all_node_coord, best_result_route, f'CVRP obj : {global_obj}')


if __name__ == '__main__':
    main()
