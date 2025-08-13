from PWB.PWB_vrpb import *
from PWB.ILSRVND import *
from PWB.KJH_vrpb import *
from PWB.optimizer import *



def PWB_run(problem_info, time_limit = 46):
    n = problem_info["N"]
    m = problem_info["K"]
    vehicle_capacity = problem_info["capa"]
    demands = problem_info["node_demands"]
    node_types = problem_info["node_types"]
    coords_dict = problem_info["node_coords"]
    dist_matrix = problem_info["dist_mat"]
    #capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,  kde_backhaul,  mins_backhaul, maxs_backhaul = kde_out(n, m, vehicle_capacity, np.array(demands), node_types, coords_dict, dist_matrix)
    capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,  kde_backhaul,  mins_backhaul, maxs_backhaul = weighted_kde_out(n, m, vehicle_capacity, np.array(demands), node_types, coords_dict, dist_matrix)
    #capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,  kde_backhaul,  mins_backhaul, maxs_backhaul = core_out(n, m, vehicle_capacity, np.array(demands), node_types, coords_dict, dist_matrix)

    start_time = time.time()
    time_limit_initial = 15
    total_cost, best_routes = Iteration_VRPB(problem_info, time_limit_initial, start_time, capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,  kde_backhaul,  mins_backhaul, maxs_backhaul, n, m, vehicle_capacity, demands, node_types, coords_dict, dist_matrix, show=False)

    initial_routes = []
    c = Construction(m, node_types, demands, vehicle_capacity, dist_matrix)

    dist_mat = np.array(dist_matrix)
    for route_nodes in best_routes:
        cost = dist_mat[route_nodes[:-1], route_nodes[1:]].sum()

        # 첫 번째 backhaul이 등장하는 인덱스 찾기
        line_idx = len(route_nodes) - 1  # 기본값 (백홀이 없으면 마지막까지 linehaul)
        for idx, node in enumerate(route_nodes):
            if node in backhaul_ids:
                line_idx = idx
                break

        line_load = sum(demands[node] for node in route_nodes if node in linehaul_ids)
        back_load = sum(demands[node] for node in route_nodes if node in backhaul_ids)

        route = Route(
            hist=route_nodes,
            cost=cost,
            line_load=line_load,
            back_load=back_load,
            line_idx=line_idx
        )

        initial_routes.append(route)

    start_time2 = time.time()
    random_cost = [0] + [random.random() for _ in range(len(node_types) - 1)]
    spool = SolPool(initial_routes, vehicle_capacity, demands, node_types, random_cost)
    ils_rvnd = ILS_RVND(m, dist_matrix)
    ils_rvnd.construct = c.construct
    ils_rvnd.run(n, spool, solv_SC, start_time2, time_limit, log=False)
    print(f"ILS-RVND bset cost :{spool.best_cost}")
    return [route.hist for route in spool.best_sol]
    # return sol

if __name__ == "__main__":
    instance_path = "../instances/problem_40_0.6.json"  # 경로는 본인 위치에 맞게 조정!
    with open(instance_path, "r") as f:
        problem_info = json.load(f)
    time_limit = 60
    start_time = time.time()
    sol = PWB_run(problem_info)
    elapsed_time = time.time() - start_time
    total_cost = check_feasible_wb(problem_info, sol, elapsed_time, time_limit)
    plot_vrpb_wb(problem_info, sol, title=f'Obj: {total_cost}')