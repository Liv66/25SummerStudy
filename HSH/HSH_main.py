# main 파일
from HSH.HSH_SP import *
import json, random, time

def HSH_algorithm(problem_info):
    N = problem_info["N"]
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = [abs(d) for d in problem_info["node_demands"]]
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    pool = run_pooling_loop(N, K, node_type, node_demand, capa, dist_mat)
    sol = run_set_partitioning(N, K, dist_mat, pool)
    return sol

def HSH_run(problem_info):
    sol = HSH_algorithm(problem_info)
    return sol

if __name__ == "__main__":
    random.seed(42)
    with open(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_130_0.85.json", encoding="utf-8") as f:
        problem_info = json.load(f)

    start_time = time.time()
    HSH_run(problem_info)
    end_time = time.time()

    elapsed = end_time - start_time
    print(f"\n알고리즘 실행 시간: {elapsed:.2f}초")