import random

from util import bin_packing, multiple_knapsack


def main():
    capa = 100
    weights = [random.randint(10, 20) for _ in range(40)]  # 아이템 무게
    log = True
    obj = bin_packing(weights, capa, log)
    print("###############################################")
    solutions = multiple_knapsack(weights, capa, obj, log=log)
    print(solutions)


if __name__ == '__main__':
    main()
