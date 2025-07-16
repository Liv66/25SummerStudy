import random
import math
import time

from util import bin_packing, get_distance, plot_cvrp
from KNY_constraint import is_solution_feasible  # 제약 검증 함수


# ─────────────────────────────────────────────────────────────
# Constraint-Aware Greedy Insertion (VRPB)
# ─────────────────────────────────────────────────────────────
def greedy_insertion_vrpb(
    delivery_idx: list[int],
    pickup_idx: list[int],
    demands: list[int],
    capa: int,
    dist: list[list[float]],
    depot_idx: int,
    node_types: list[int],  # 추가
) -> list[list[int]]:
    unassigned_deliv = set(delivery_idx)
    unassigned_pick  = set(pickup_idx)
    routes: list[list[int]] = []

    # ① 배송 노드 먼저
    while unassigned_deliv:
        route, load, cur = [depot_idx], 0, depot_idx
        while unassigned_deliv:
            nxt = min(unassigned_deliv, key=lambda n: dist[cur][n])
            delta = -demands[nxt] if node_types[nxt] == 1 else demands[nxt]
            if load + delta > capa or load + delta < 0:
                break
            route.append(nxt); load += delta
            cur = nxt; unassigned_deliv.remove(nxt)

        # ② 같은 차량에서 회수 노드
        while unassigned_pick:
            nxt = min(unassigned_pick, key=lambda n: dist[cur][n])
            delta = -demands[nxt] if node_types[nxt] == 1 else demands[nxt]
            if load + delta > capa or load + delta < 0:
                break
            route.append(nxt); load += delta
            cur = nxt; unassigned_pick.remove(nxt)

        route.append(depot_idx)
        routes.append(route)

    # ③ 남은 회수 노드
    while unassigned_pick:
        route, load, cur = [depot_idx], 0, depot_idx
        while unassigned_pick:
            nxt = min(unassigned_pick, key=lambda n: dist[cur][n])
            delta = -demands[nxt] if node_types[nxt] == 1 else demands[nxt]
            if load + delta > capa or load + delta < 0:
                break
            route.append(nxt); load += delta
            cur = nxt; unassigned_pick.remove(nxt)
        route.append(depot_idx)
        routes.append(route)

    return routes


# ─────────────────────────────────────────────────────────────
# Utility: cost of one route
# ─────────────────────────────────────────────────────────────
def route_cost(route: list[int], dist: list[list[float]]) -> float:
    return sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))


# ─────────────────────────────────────────────────────────────
# Adaptive Large Neighborhood Search (VRPB)
# ─────────────────────────────────────────────────────────────
def alns_vrpb(
    init_routes: list[list[int]],
    dist: list[list[float]],
    node_types: list[int],
    demands: list[int],
    capa: int,
    depot_idx: int,
    time_limit: float = 60.0,
    p_destroy: float = 0.3,
) -> tuple[list[list[int]], float]:
    best_routes = [r[:] for r in init_routes]
    best_cost   = sum(route_cost(r, dist) for r in best_routes)
    cur_routes  = [r[:] for r in best_routes]

    T0, alpha = 1_000.0, 0.995
    iteration = 0
    start = time.time()

    while time.time() - start < time_limit:
        iteration += 1

        # ── Destroy ──
        removed, new_routes = [], []
        for r in cur_routes:
            if random.random() < p_destroy and len(r) > 3:
                k = random.randint(1, min(2, len(r) - 2))
                idx_to_remove = random.sample(range(1, len(r) - 1), k)
                removed.extend(r[i] for i in idx_to_remove)
                new_r = [v for i, v in enumerate(r) if i not in idx_to_remove]
                if len(new_r) > 2:
                    new_routes.append(new_r)
            else:
                new_routes.append(r)

        # ── Repair ──
        for n in removed:
            best_delta, best_pos, best_r = math.inf, None, None
            for r in new_routes:
                # 순서 제약
                if node_types[n] == 1 and any(node_types[v] == 0 for v in r[1:-1]):
                    continue
                # 용량 제약
                load = 0
                for v in r:
                    if v == depot_idx:
                        continue
                    load += -demands[v] if node_types[v] == 1 else demands[v]

                change = -demands[n] if node_types[n] == 1 else demands[n]
                if load + change > capa or load + change < 0:
                    continue
                # 위치별 비용
                for pos in range(1, len(r)):
                    delta = dist[r[pos - 1]][n] + dist[n][r[pos]] - dist[r[pos - 1]][r[pos]]
                    if delta < best_delta:
                        best_delta, best_pos, best_r = delta, pos, r
            if best_r is None:
                new_routes.append([depot_idx, n, depot_idx])
            else:
                best_r.insert(best_pos, n)

        if not is_solution_feasible(new_routes, node_types, demands, capa, depot_idx):
            continue

        new_cost = sum(route_cost(r, dist) for r in new_routes)
        T = T0 * (alpha ** iteration)
        if new_cost < best_cost or random.random() < math.exp((best_cost - new_cost) / T):
            cur_routes = [r[:] for r in new_routes]
            if new_cost < best_cost:
                best_cost  = new_cost
                best_routes = [r[:] for r in new_routes]

    return best_routes, best_cost


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main() -> None:
    N, capa = 50, 5000

    nodes_coord = [(random.uniform(0, 24_000), random.uniform(0, 32_000)) for _ in range(N)]
    demands     = [int(random.gauss(500, 200)) for _ in range(N)]

    num_delivery = int(N * 0.6)
    num_pickup   = N - num_delivery
    node_types   = [1] * num_delivery + [0] * num_pickup
    random.shuffle(node_types)
    delivery_idx = [i for i, t in enumerate(node_types) if t == 1]
    pickup_idx   = [i for i, t in enumerate(node_types) if t == 0]

    K = max(
        bin_packing([demands[i] for i in delivery_idx], capa),
        bin_packing([demands[i] for i in pickup_idx],  capa),
    )
    print(f"# of vehicles (upper-bound) {K}")

    depot_coord  = (12_000, 16_000)
    all_coord    = nodes_coord + [depot_coord]
    dist_matrix  = get_distance(all_coord)
    depot_idx    = N

    init_routes = greedy_insertion_vrpb(
        delivery_idx, pickup_idx, demands, capa, dist_matrix, depot_idx, node_types  # ← node_types 추가
    )
    assert is_solution_feasible(init_routes, node_types, demands, capa, depot_idx), \
        "Initial solution infeasible!"

    best_routes, best_cost = alns_vrpb(
        init_routes, dist_matrix, node_types, demands, capa,
        depot_idx=depot_idx, time_limit=60,
    )
    assert is_solution_feasible(best_routes, node_types, demands, capa, depot_idx), \
        "Final best solution infeasible!"

    for idx, route in enumerate(best_routes):
        print(f"vehicle {idx} route: {route}")
    plot_cvrp(all_coord, best_routes, f"VRPB obj: {best_cost:.1f}")


if __name__ == "__main__":
    main()
