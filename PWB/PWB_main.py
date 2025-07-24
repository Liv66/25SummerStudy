from PWB.PWB_vrpb import *


def PWB_run(problem_info):
    n = problem_info["N"]
    m = problem_info["K"]
    vehicle_capacity = problem_info["capa"]
    demands = problem_info["node_demands"]
    node_types = problem_info["node_types"]
    coords_dict = problem_info["node_coords"]
    dist_matrix = problem_info["dist_mat"]

    capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,  kde_backhaul,  mins_backhaul, maxs_backhaul = kde_out(n, m, vehicle_capacity, demands, node_types, coords_dict, dist_matrix)

    start_time = time.time()
    time_limit = 56
    total_cost, sol = Iteration_VRPB(problem_info, time_limit, start_time, capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,  kde_backhaul,  mins_backhaul, maxs_backhaul, n, m, vehicle_capacity, demands, node_types, coords_dict, dist_matrix, show=False)

    return sol

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