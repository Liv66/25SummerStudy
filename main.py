import json
import random

from KJH.KJH_main import KJH_main
from util import *


def instance_generator(problem, N=50, capa=3000, line_p=0.7):
    # 인덱스 0은 depot
    problem_info = {}
    nodes_coord = [(12000, 16000)] + [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(N - 1)]
    demands = [0] + [int(random.gauss(500, 200)) for _ in range(N - 1)]

    num_line = int(N * line_p)
    node_type = [2 for _ in range(N)]
    node_type[0] = 0
    line_list = random.sample(range(1, N), num_line)
    for i in line_list:
        node_type[i] = 1

    line_K = bin_packing([demands[i] for i in range(N) if node_type[i] == 1], capa)  # 차량 수
    back_K = bin_packing([demands[i] for i in range(N) if node_type[i] == 2], capa)
    K = max(line_K, back_K)
    dist_mat = get_distance(nodes_coord)  # 거리 행렬 계산

    problem_info['N'] = N
    problem_info['K'] = K
    problem_info['capa'] = capa
    problem_info['node_demands'] = demands
    problem_info['node_types'] = node_type
    problem_info['node_coords'] = nodes_coord
    problem_info['dist_mat'] = dist_mat

    with open(problem, "w", encoding='utf-8') as f:
        json.dump(problem_info, f, ensure_ascii=False, indent=4)

def main():
    N = 220
    line_p = 0.7
    capa = 2000
    problem = f"instances/problem_{N}_{line_p}.json"
    try:
        with open(problem, "r", encoding='utf-8') as f:
            problem_info = json.load(f)

    except FileNotFoundError:
        instance_generator(problem, N=N, line_p=line_p, capa=capa)
        with open(problem, "r", encoding='utf-8') as f:
            problem_info = json.load(f)
    start = time.time()
    sol = KJH_main(problem_info)

    elapsed = time.time() - start
    time_limit = 60
    obj = check_feasible(problem_info, sol, elapsed, time_limit)
    plot_vrpb(problem_info, sol, f'obj : {obj}')


if __name__ == '__main__':
    main()
