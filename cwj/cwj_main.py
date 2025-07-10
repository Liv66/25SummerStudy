import random

from cwj_util import bin_packing, multiple_knapsack, get_distance, two_opt


def main():
    # capa = 100
    # weights = [random.randint(10, 20) for _ in range(40)]  # 아이템 무게
    # log = True
    # obj = bin_packing(weights, capa, log)
    # print("###############################################")
    # solutions = multiple_knapsack(weights, capa, obj, log=log)
    # print(solutions)

    N = 50
    nodes_coord = [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(N)]
    nodes = [i for i in range(N)]
    dist_matrix = get_distance(nodes_coord)
    two_opt(0, nodes, dist_matrix, nodes_coord, True)


if __name__ == '__main__':
    main()

