import time

from KJH.ILSRVND import *
from KJH.KJH_vrpb import *
from KJH.optimizer import solv_SC


def KJH_run(problem_info, time_limit=60):
    start = time.time()
    N = problem_info['N']
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']
    random_cost = [0] + [random.random() for _ in range(len(node_type) - 1)]
    c = Construction(K, node_type, node_demand, capa, dist_mat)
    initial_routes = c.construct()
    spool = SolPool(initial_routes, capa, node_demand, node_type, random_cost)
    ils_rvnd = ILS_RVND(K, dist_mat)
    ils_rvnd.construct = c.construct
    ils_rvnd.run(N, spool, solv_SC, start, time_limit=time_limit, log=False)
    print(f"ILS-RVND bset cost :{spool.best_cost}")
    return [route.hist for route in spool.best_sol]
    # return solv_SC(spool, dist_mat, N, K)


if __name__ == "__main__":
    KJH_run()

