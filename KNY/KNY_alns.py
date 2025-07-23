import random
import math
import time
from KNY_constraint import is_solution_feasible

def alns_vrpb(
    init_routes: list[list[int]],
    dist: list[list[float]],
    node_types: list[int],
    demands: list[int],
    capa: int,
    depot_idx: int,
    max_vehicles: int,
    time_limit: float = 60.0,
) -> tuple[list[list[int]], float]:
    print("[INFO] Start ALNS (Adaptive Weight)")
    PENALTY_FACTOR = 100000000

    def route_cost(route):
        return sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))

    def calculate_total_cost(routes):
        if not routes:
            return float('inf')
        distance_cost = sum(route_cost(r) for r in routes)
        vehicle_overflow = max(0, len(routes) - max_vehicles)
        return distance_cost + PENALTY_FACTOR * vehicle_overflow

    def roulette_select(weights):
        total = sum(weights)
        r = random.uniform(0, total)
        acc = 0
        for i, w in enumerate(weights):
            acc += w
            if acc >= r:
                return i
        return len(weights) - 1

    def update_weights(weights, scores, counts, reaction=0.2):
        for i in range(len(weights)):
            if counts[i] == 0:
                continue
            weights[i] = (1 - reaction) * weights[i] + reaction * (scores[i] / counts[i])
            scores[i] = 0
            counts[i] = 0

    def random_removal(routes, remove_ratio=0.2):
        flat_nodes = [n for r in routes for n in r[1:-1]]
        if not flat_nodes: return [], routes
        num_remove = max(1, int(len(flat_nodes) * remove_ratio))
        to_remove = set(random.sample(flat_nodes, min(num_remove, len(flat_nodes))))
        new_routes = [[v for v in r if v not in to_remove or v == depot_idx] for r in routes]
        return list(to_remove), [r for r in new_routes if len(r) > 2]

    def shaw_removal(routes, remove_count=5):
        all_nodes = [n for r in routes for n in r[1:-1]]
        if len(all_nodes) <= remove_count:
            return all_nodes, []
        n0 = random.choice(all_nodes)
        removed = {n0}
        def similarity(n1, n2):
            dx = dist[n1][n2]
            dt = abs(demands[n1] - demands[n2])
            tt = node_types[n1] != node_types[n2]
            return dx + 10 * dt + (1000 if tt else 0)
        while len(removed) < remove_count and len(removed) < len(all_nodes):
            candidates = [n for n in all_nodes if n not in removed]
            n_next = min(candidates, key=lambda n: similarity(n0, n))
            removed.add(n_next)
        new_routes = [[v for v in r if v not in removed or v == depot_idx] for r in routes]
        return list(removed), [r for r in new_routes if len(r) > 2]

    def worst_removal(routes, remove_count=5):
        cost_contrib = []
        for r in routes:
            for i in range(1, len(r) - 1):
                prev, curr, nxt = r[i - 1], r[i], r[i + 1]
                added_cost = dist[prev][curr] + dist[curr][nxt] - dist[prev][nxt]
                cost_contrib.append((added_cost, curr))
        cost_contrib.sort(reverse=True)
        removed = set(n for _, n in cost_contrib[:remove_count])
        new_routes = [[v for v in r if v not in removed or v == depot_idx] for r in routes]
        return list(removed), [r for r in new_routes if len(r) > 2]

    def route_removal(routes, remove_routes=1):
        short_routes = sorted([r for r in routes if len(r) <= 4], key=lambda r: len(r))
        candidates = short_routes if short_routes else routes
        if not candidates: return [], []
        k = min(remove_routes, len(candidates))
        to_remove = random.sample(candidates, k)
        removed_nodes = [n for r in to_remove for n in r[1:-1]]
        new_routes = [r for r in routes if r not in to_remove]
        return removed_nodes, new_routes

    def greedy_insert(removed_nodes, partial_routes):
        random.shuffle(removed_nodes)
        for n in removed_nodes:
            best_cost_increase = float('inf')
            best_insertion = None
            for r_idx, r in enumerate(partial_routes):
                for i in range(1, len(r)):
                    temp_route = r[:i] + [n] + r[i:]
                    if is_solution_feasible([temp_route], node_types, demands, capa, depot_idx):
                        cost_increase = dist[r[i-1]][n] + dist[n][r[i]] - dist[r[i-1]][r[i]]
                        if cost_increase < best_cost_increase:
                            best_cost_increase = cost_increase
                            best_insertion = (r_idx, i)
            if best_insertion:
                r_idx, pos = best_insertion
                partial_routes[r_idx].insert(pos, n)
            else:
                new_route = [depot_idx, n, depot_idx]
                if is_solution_feasible([new_route], node_types, demands, capa, depot_idx):
                    partial_routes.append(new_route)
        return partial_routes

    def regret_k_insert(removed_nodes, partial_routes, k=2):
        for _ in range(len(removed_nodes)):
            regrets = []
            for n in removed_nodes:
                costs = []
                for r_idx, r in enumerate(partial_routes):
                    for i in range(1, len(r)):
                        temp_route = r[:i] + [n] + r[i:]
                        if is_solution_feasible([temp_route], node_types, demands, capa, depot_idx):
                            cost_increase = dist[r[i-1]][n] + dist[n][r[i]] - dist[r[i-1]][r[i]]
                            costs.append((cost_increase, r_idx, i))
                if costs:
                    costs.sort()
                    regret_val = sum(costs[j][0] - costs[0][0] for j in range(1, min(k, len(costs))))
                    regrets.append((regret_val, n, costs[0][1], costs[0][2]))
            if not regrets:
                break
            regrets.sort(reverse=True)
            _, node_to_insert, r_idx, pos = regrets[0]
            partial_routes[r_idx].insert(pos, node_to_insert)
            removed_nodes.remove(node_to_insert)
        if removed_nodes:
            partial_routes = greedy_insert(removed_nodes, partial_routes)
        return partial_routes

    destroy_ops = [random_removal, shaw_removal, worst_removal, route_removal]
    repair_ops = [greedy_insert, regret_k_insert]
    d_weights = [1.0] * len(destroy_ops)
    r_weights = [1.0] * len(repair_ops)
    d_scores = [0] * len(destroy_ops)
    r_scores = [0] * len(repair_ops)
    d_counts = [1] * len(destroy_ops)
    r_counts = [1] * len(repair_ops)

    best_routes = [r[:] for r in init_routes]
    best_cost = calculate_total_cost(best_routes)
    cur_routes = [r[:] for r in best_routes]

    T = 1000.0
    target_accept = 0.3
    adjust_rate = 0.05
    accepted, attempted = 0, 0
    iteration = 0
    start = time.time()

    while time.time() - start < time_limit:
        iteration += 1
        elapsed = time.time() - start

        if not cur_routes or not any(len(r) > 2 for r in cur_routes):
            continue

        d_idx = roulette_select(d_weights)
        r_idx = roulette_select(r_weights)

        removed, part = destroy_ops[d_idx](cur_routes)
        if not removed:
            continue

        new_routes = repair_ops[r_idx](removed[:], part[:])

        if not is_solution_feasible(new_routes, node_types, demands, capa, depot_idx):
            continue

        new_cost = calculate_total_cost(new_routes)
        attempted += 1
        d_counts[d_idx] += 1
        r_counts[r_idx] += 1

        if new_cost < best_cost:
            best_cost = new_cost
            best_routes = [r[:] for r in new_routes]
            cur_routes = [r[:] for r in new_routes]
            d_scores[d_idx] += 5
            r_scores[r_idx] += 5
            accepted += 1
        else:
            delta = best_cost - new_cost
            prob = math.exp(delta / T)
            if random.random() < prob:
                cur_routes = [r[:] for r in new_routes]
                d_scores[d_idx] += 1
                r_scores[r_idx] += 1
                accepted += 1

        if attempted > 10 and iteration % 20 == 0:
            actual_accept = accepted / attempted
            T *= math.exp(adjust_rate * (target_accept - actual_accept))
            T = max(1.0, min(T, 100000.0))
            accepted, attempted = 0, 0

        if iteration % 50 == 0:
            update_weights(d_weights, d_scores, d_counts)
            update_weights(r_weights, r_scores, r_counts)

        if iteration % 10 == 0:
            print(f"[ALNS][{elapsed:.1f}s] Iter {iteration:4d}: Current cost={new_cost:.1f} ({len(cur_routes)} routes), Best cost={best_cost:.1f} ({len(best_routes)} routes)")

    print("[INFO] ALNS (Adaptive) finished.")
    return best_routes, best_cost
