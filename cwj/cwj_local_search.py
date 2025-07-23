# cwj_local_search.py

def improve_solution(selected_routes, route_pool, node_types, node_demands, dist_mat, Q):
    """
    단순한 local search: 선택된 route 중 1개에 대해 고객 swap 후 개선되면 반영
    selected_routes: List of (route, cost)
    """
    improved_routes = []

    for route, cost in selected_routes:
        best_route = route
        best_cost = cost

        if len(route) <= 3:  # depot - customer - depot
            improved_routes.append((best_route, best_cost))
            continue

        for i in range(1, len(route)-2):
            for j in range(i+1, len(route)-1):
                new_route = route[:]
                new_route[i], new_route[j] = new_route[j], new_route[i]
                if not is_feasible(new_route, node_types, node_demands, Q):
                    continue
                new_cost = calc_route_cost(new_route, dist_mat)
                if new_cost < best_cost:
                    best_route = new_route
                    best_cost = new_cost

        improved_routes.append((best_route, best_cost))

    return improved_routes

def calc_route_cost(route, dist_mat):
    return sum(dist_mat[route[i]][route[i+1]] for i in range(len(route)-1))

def is_feasible(route, node_types, node_demands, Q):
    """
    Linehaul -> Backhaul 순서 및 적재량 제약 확인
    """
    load = Q
    backhaul_started = False
    for i in route[1:-1]:
        if node_types[i] == 2:
            backhaul_started = True
            load += node_demands[i]
        else:
            if backhaul_started:
                return False
            load -= node_demands[i]
        if load < 0 or load > Q:
            return False
    return True