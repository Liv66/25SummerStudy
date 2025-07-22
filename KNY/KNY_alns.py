"""
alns_vrpb.py  -  ALNS with Adaptive Weights for VRPB
"""

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

    def route_cost(route):
        return sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))

    def random_removal(routes, remove_ratio=0.2):
        flat_nodes = [n for r in routes for n in r[1:-1]]
        num_remove = max(1, int(len(flat_nodes) * remove_ratio))
        to_remove = set(random.sample(flat_nodes, num_remove))
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

        while len(removed) < remove_count:
            candidates = [n for n in all_nodes if n not in removed]
            n_next = min(candidates, key=lambda n: similarity(n0, n))
            removed.add(n_next)
        new_routes = [[v for v in r if v not in removed or v == depot_idx] for r in routes]
        return list(removed), [r for r in new_routes if len(r) > 2]

    def worst_removal(routes, remove_count=5):
        cost_contrib = []
        for r in routes:
            for i in range(1, len(r) - 1):
                prev, curr, next = r[i - 1], r[i], r[i + 1]
                added_cost = dist[prev][curr] + dist[curr][next] - dist[prev][next]
                cost_contrib.append((added_cost, curr))
        cost_contrib.sort(reverse=True)
        removed = set(n for _, n in cost_contrib[:remove_count])
        new_routes = [[v for v in r if v not in removed or v == depot_idx] for r in routes]
        return list(removed), [r for r in new_routes if len(r) > 2]

    def route_removal(routes, remove_routes=1):
        #  ⓐ 픽업만 또는 노드 수가 2~3개인 짧은 경로를 우선 후보로
        short_routes = sorted(
            [r for r in routes if len(r) <= 4],  # depot 포함 길이 ≤4
            key=lambda r: len(r)
        )
        #  ⓑ 후보가 없으면 랜덤 경로
        candidates = short_routes if short_routes else routes
        k = min(remove_routes, len(candidates))
        to_remove = random.sample(candidates, k)
        removed_nodes = [n for r in to_remove for n in r[1:-1]]
        new_routes = [r for r in routes if r not in to_remove]
        return removed_nodes, new_routes

    def greedy_insert(removed_nodes, partial_routes):
        for n in removed_nodes:
            candidates = []
            for r in partial_routes:
                # 배송/회수 분포 파악 (depot 제외)
                has_delivery = any(node_types[v] == 1 for v in r[1:-1] if v != depot_idx)
                has_pickup = any(node_types[v] == 0 for v in r[1:-1] if v != depot_idx)

                # 마지막 배송 위치 찾기 (depot 제외)
                if has_delivery:
                    last_delivery_idx = max(i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1)
                else:
                    last_delivery_idx = 0  # 배송 없음

                if node_types[n] == 1:  # 배송 노드
                    if has_delivery and has_pickup:
                        continue  # 이미 배송→회수인 경로엔 배송 불가
                    insert_pos = 1
                    next_node = r[1] if len(r) > 1 else depot_idx
                    cost = dist[r[0]][n] + dist[n][next_node] - dist[r[0]][next_node]
                    candidates.append((cost, insert_pos, r))
                else:  # 회수 노드
                    start_pos = last_delivery_idx + 1
                    for i in range(start_pos, len(r)):
                        prev, nxt = r[i - 1], r[i]
                        load = sum((-demands[v] if node_types[v] == 1 else demands[v])
                                   for v in r if v != depot_idx)
                        delta = demands[n]
                        if load + delta > capa or load + delta < 0:
                            continue
                        cost = dist[prev][n] + dist[n][nxt] - dist[prev][nxt]
                        candidates.append((cost, i, r))

            if candidates:
                _, pos, route = min(candidates, key=lambda x: x[0])
                route.insert(pos, n)
            else:
                partial_routes.append([depot_idx, n, depot_idx])
        return partial_routes

    def regret_k_insert(removed_nodes, partial_routes, k=2):
        for _ in range(len(removed_nodes)):
            best_node = None
            best_pos = None
            best_r = None
            max_regret = -float('inf')

            for n in removed_nodes:
                insertion_options = []
                for r in partial_routes:
                    has_delivery = any(node_types[v] == 1 for v in r[1:-1] if v != depot_idx)
                    has_pickup = any(node_types[v] == 0 for v in r[1:-1] if v != depot_idx)

                    if has_delivery:
                        last_delivery_idx = max(i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1)
                    else:
                        last_delivery_idx = 0

                    if node_types[n] == 1:  # 배송 노드
                        if has_delivery and has_pickup:
                            continue
                        insert_pos = 1
                        next_node = r[1] if len(r) > 1 else depot_idx
                        cost = dist[r[0]][n] + dist[n][next_node] - dist[r[0]][next_node]
                        insertion_options.append((cost, insert_pos, r))
                    else:  # 회수 노드
                        start_pos = last_delivery_idx + 1
                        for i in range(start_pos, len(r)):
                            prev, nxt = r[i - 1], r[i]
                            load = sum((-demands[v] if node_types[v] == 1 else demands[v])
                                       for v in r if v != depot_idx)
                            delta = demands[n]
                            if load + delta > capa or load + delta < 0:
                                continue
                            cost = dist[prev][n] + dist[n][nxt] - dist[prev][nxt]
                            insertion_options.append((cost, i, r))

                if len(insertion_options) >= k:
                    insertion_options.sort()
                    regret = insertion_options[k - 1][0] - insertion_options[0][0]
                    if regret > max_regret:
                        max_regret = regret
                        best_node = n
                        best_pos, best_r = insertion_options[0][1], insertion_options[0][2]

            if best_node is not None:
                best_r.insert(best_pos, best_node)
                removed_nodes.remove(best_node)
            else:
                break

        for n in removed_nodes:
            partial_routes.append([depot_idx, n, depot_idx])
        return partial_routes

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
            if counts[i] == 0: continue
            weights[i] = (1 - reaction) * weights[i] + reaction * (scores[i] / counts[i])
            scores[i] = 0
            counts[i] = 0

    destroy_ops = [random_removal, shaw_removal, worst_removal, route_removal]
    repair_ops = [greedy_insert, regret_k_insert]
    d_weights = [1.0 for _ in destroy_ops]
    r_weights = [1.0 for _ in repair_ops]
    d_scores = [0 for _ in destroy_ops]
    r_scores = [0 for _ in repair_ops]
    d_counts = [1 for _ in destroy_ops]
    r_counts = [1 for _ in repair_ops]

    best_routes = [r[:] for r in init_routes]
    best_cost = sum(route_cost(r) for r in best_routes)
    cur_routes = [r[:] for r in best_routes]

    T = 10000.0
    target_accept = 0.3
    adjust_rate = 0.05
    accepted, attempted = 0, 0
    iteration = 0
    start = time.time()

    while time.time() - start < time_limit:
        iteration += 1
        d_idx = roulette_select(d_weights)
        r_idx = roulette_select(r_weights)

        removed, part = destroy_ops[d_idx](cur_routes)
        new_routes = repair_ops[r_idx](removed[:], part[:])

        if len(new_routes) > max_vehicles:
            continue  # 🔴 차량 수가 상한 초과되면 해 버림

        if not is_solution_feasible(new_routes, node_types, demands, capa, depot_idx, max_vehicles):
            attempted += 1
            d_counts[d_idx] += 1
            r_counts[r_idx] += 1
            continue

        new_cost = sum(route_cost(r) for r in new_routes)
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
            # 목표보다 낮으면 (target - actual > 0) → 양수 지수 → T 상승
            T *= math.exp(adjust_rate * (target_accept - actual_accept))
            T = max(1.0, min(T, 100000.0))
            accepted, attempted = 0, 0

        if iteration % 50 == 0:
            update_weights(d_weights, d_scores, d_counts)
            update_weights(r_weights, r_scores, r_counts)

    print("[INFO] ALNS (Adaptive) finished.")
    return best_routes, best_cost
