# cwj_dual_route_generator.py
from cwj_pricing_problem import labeling_algorithm

def generate_dual_routes(duals, route_pool, node_types, node_demands, dist_mat, Q):
    """
    Dual 값을 반영해 reduced cost < 0 인 새로운 경로 생성
    """
    N = len(node_types)
    depot = 0
    linehauls = [i for i in range(1, N) if node_types[i] == 1]
    backhauls = [i for i in range(1, N) if node_types[i] == 2]

    new_routes = []
    candidates = labeling_algorithm(linehauls, backhauls, dist_mat, node_demands, node_types, Q)

    for route, cost in candidates:
        dual_sum = sum(duals[i] for i in route if i != depot)
        reduced_cost = cost - dual_sum - duals[depot]
        if reduced_cost < -1e-6:  # 유의미한 개선만
            new_routes.append((route, cost))

    return new_routes