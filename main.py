import random

from util import bin_packing, multiple_knapsack, get_distance, two_opt, plot_cvrp


def main():
    # capa = 100
    # weights = [random.randint(10, 20) for _ in range(40)]  # 아이템 무게
    # log = True
    # obj = bin_packing(weights, capa, log)
    # print("###############################################")
    # solutions = multiple_knapsack(weights, capa, obj, log=log)
    # print(solutions)

    # N = 50
    # nodes_coord = [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(N)]
    # nodes = [i for i in range(N)]
    # dist_matrix = get_distance(nodes_coord)
    # two_opt(0, nodes, dist_matrix, nodes_coord, True)

    N = 50  # 방문할 노드 수
    capa = 5000  # 차량들의 용량
    nodes_coord = [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(N)]
    demands = [int(random.gauss(500, 200)) for _ in range(N)]
    K = bin_packing(demands, capa)  # 차량 수
    print(f'# of vehicles {K}')
    knapsack_sol = multiple_knapsack(demands, capa, K)  # 경로 조합 탐색
    depot_coord = (12000, 16000)  # depot 노드 좌표
    all_node_coord = nodes_coord + [depot_coord]
    dist_matrix = get_distance(all_node_coord)  # 거리 행렬 계산

    global_obj = 987654321
    best_result_route = []
    for idx, packing_sol in enumerate(knapsack_sol):
        obj = 0
        result_route = []
        for route in packing_sol:
            cost, opt_route = two_opt(N, [N] + route, dist_matrix, all_node_coord)
            result_route.append(opt_route)
            obj += cost
        if obj < global_obj:  # 해가 개선되면 저장
            global_obj = obj
            best_result_route = result_route
    for idx, route in enumerate(best_result_route):
        print(f'vehicle {idx} route : {route} ')
    plot_cvrp(all_node_coord, best_result_route, f'CVRP obj : {global_obj}')


if __name__ == '__main__':
    main()
