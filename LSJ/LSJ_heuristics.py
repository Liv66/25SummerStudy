import random
from LSJ_util import VRPColumn, is_valid_vrpb_route, calculate_path_cost, get_route_type

def generate_solution_robust_final(nodes, dist_matrix, num_vehicles, capacity, max_attempts=10):
    depot = 0
    def _insert_customers(current_routes, customers_to_route):
        routes = [list(r) for r in current_routes]; unrouted = []
        for cust_idx in customers_to_route:
            best_insertion = None; min_score = float('inf')
            for i, route in enumerate(routes):
                for pos in range(len(route) + 1):
                    temp_route = route[:pos] + [cust_idx] + route[pos:]
                    if is_valid_vrpb_route([depot] + temp_route + [depot], nodes, capacity):
                        cost_before = calculate_path_cost([depot] + route + [depot], dist_matrix)
                        cost_after = calculate_path_cost([depot] + temp_route + [depot], dist_matrix)
                        score = cost_after - cost_before
                        if score < min_score:
                            min_score = score; best_insertion = {'route_idx': i, 'pos': pos}
            if best_insertion: routes[best_insertion['route_idx']].insert(best_insertion['pos'], cust_idx)
            else: unrouted.append(cust_idx)
        return routes, unrouted
    linehaul_customers = [i for i, n in enumerate(nodes) if n['type'] == 'linehaul']
    backhaul_customers = [i for i, n in enumerate(nodes) if n['type'] == 'backhaul']
    random.shuffle(linehaul_customers); random.shuffle(backhaul_customers)
    routes = [[] for _ in range(num_vehicles)]
    routes, unrouted_linehaul = _insert_customers(routes, linehaul_customers)
    routes, unrouted_backhaul = _insert_customers(routes, backhaul_customers)
    final_unrouted = unrouted_linehaul + unrouted_backhaul
    if final_unrouted: return []
    return [[depot] + r + [depot] for r in routes if r]

def two_opt_improvement(route, dist_matrix):
    best = list(route); improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best)):
                if j - i == 1: continue
                new_route = best[:i] + best[i:j][::-1] + best[j:]
                if calculate_path_cost(new_route, dist_matrix) < calculate_path_cost(best, dist_matrix):
                    best = new_route; improved = True
    return tuple(best)

def generate_multi_start_initial_columns(nodes, dist_matrix, num_vehicles, capacity, num_starts=20):
    unique_routes = set(); a_valid_initial_solution = None
    for i in range(num_starts):
        initial_routes_single_run = generate_solution_robust_final(nodes, dist_matrix, num_vehicles, capacity, max_attempts=10)
        if not initial_routes_single_run: continue
        optimized_solution = []
        for route in initial_routes_single_run:
            original_route = tuple(route)
            optimized_route = two_opt_improvement(original_route, dist_matrix)
            if is_valid_vrpb_route(optimized_route, nodes, capacity): optimized_solution.append(optimized_route)
            else: optimized_solution.append(original_route)
        unique_routes.update(optimized_solution)
        if a_valid_initial_solution is None and len(optimized_solution) <= num_vehicles:
            a_valid_initial_solution = [list(r) for r in optimized_solution]
    if not unique_routes: return [], None
    initial_columns = [VRPColumn(list(r), calculate_path_cost(r, dist_matrix), get_route_type(r, nodes)) for r in sorted(list(unique_routes))]
    return initial_columns, a_valid_initial_solution