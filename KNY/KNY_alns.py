# === 개선된 ALNS with 교차 해결 및 클러스터링 강화 (로그 제어 버전) ===
import random
import math
import time
from typing import List


def fast_feasible_check_alns(
        route: List[int],
        node_types: List[int],
        demands: List[int],
        capa: int,
        depot_idx: int,
) -> bool:
    # ── 0. 기본 조건 ────────────────────────
    if route[0] != depot_idx or route[-1] != depot_idx:
        return False
    if len(route) < 3:  # depot-node-depot
        return True

    # ── 1. 적재량 / 상태 초기화 ────────────────
    load = 0
    in_pick = False

    # ── 2. 경로 순회하며 순서 및 용량 검사 ─────────────────
    for i in range(1, len(route) - 1):
        curr = route[i]
        t = node_types[curr]
        d = demands[curr]

        if t == 1:  # delivery (배송)
            if in_pick:
                return False  # 회수 시작 후 배송 불가
            load += d
        else:  # pickup (회수)
            if not in_pick:
                in_pick = True
                load = 0  # 배송 화물 하차 후 적재 시작
            load += d

        if load > capa:
            return False
        if load < 0:  # 만약을 위한 방어 코드
            return False

    # ★★★★★ 핵심 수정 사항 ★★★★★
    # 경로에 배송(delivery, type 1) 노드가 하나라도 있는지 최종 확인합니다.
    # 하나도 없다면, 이 경로는 백홀 전용이므로 규칙 위반입니다.
    if not any(node_types[n] == 1 for n in route[1:-1]):
        return False

    # 모든 검사를 통과한 경우
    return True


def is_route_feasible(route, node_types, demands, capa, depot_idx):
    return fast_feasible_check_alns(route, node_types, demands, capa, depot_idx)


def solution_feasible(routes, node_types, demands, capa, depot_idx, verbose=False):
    """
    각 라우트의 실행 가능 여부 + 전체 노드 방문 여부 + 적재량 오류 여부를 함께 검사
    실패한 경우 원인을 로그로 출력 (verbose=True일 때만)
    """
    all_nodes = set(i for i in range(len(node_types)) if i != depot_idx)
    visited = set(n for r in routes for n in r if n != depot_idx)

    # ── ① 전체 노드 방문 여부 검사 ─────────────────────
    missed = all_nodes - visited
    if missed:
        if verbose:
            print(f"[❌ solution_feasible] 방문하지 않은 노드: {sorted(missed)}")
        return False

    # ── ② 각 라우트의 제약조건 검사 ─────────────────────
    for r_idx, route in enumerate(routes):
        if not fast_feasible_check_alns(route, node_types, demands, capa, depot_idx):
            if verbose:
                print(f"[❌ solution_feasible] route {r_idx}가 fast_feasible_check_alns() 통과 실패: {route}")
            return False

        # ── ③ 적재량 음수 및 용량 초과 검사 ──────────────
        load = 0
        in_pick = False
        for i in range(1, len(route) - 1):
            n = route[i]
            if node_types[n] == 1:  # delivery
                if in_pick:
                    if verbose:
                        print(f"[❌ solution_feasible] route {r_idx}: pickup 이후에 delivery 등장 → node {n}")
                    return False
                load += demands[n]
            else:  # pickup
                if not in_pick:
                    in_pick = True
                    load = 0  # delivery 끝났으면 하차 후 pickup 적재 시작
                load += demands[n]

            if load < 0:
                if verbose:
                    print(f"[❌ solution_feasible] route {r_idx}: 적재량 음수 발생 at node {n}, load={load}")
                return False
            if load > capa:
                if verbose:
                    print(f"[❌ solution_feasible] route {r_idx}: 적재량 초과 at node {n}, load={load}, capa={capa}")
                return False

    # ── 모든 검사 통과 ───────────────────────────────
    return True


def alns_vrpb(
        init_routes: list[list[int]],
        dist: list[list[float]],
        node_types: list[int],
        demands: list[int],
        capa: int,
        depot_idx: int,
        max_vehicles: int,
        deadline: float,  # time_limit 대신 deadline을 받음
        verbose: bool = False,  # 로그 제어 매개변수 추가
# ★★★ 1. 로컬 서치 ON/OFF 스위치 파라미터 추가 ★★★
        # 이 값을 False로 주면 Local Search를 실행하지 않습니다.
        enable_local_search: bool = True
) -> tuple[list[list[int]], float]:

    def log(message: str, force: bool = False):
        """로그 출력 함수 - verbose가 True이거나 force가 True일 때만 출력"""
        if verbose or force:
            print(message)

    log("[INFO] Start Improved ALNS with Enhanced Crossing Resolution")
    PENALTY_FACTOR = 10000000
    _route_cost_cache: dict[tuple[int, ...], float] = {}

    def route_cost(route: list[int]) -> float:
        key = tuple(route)
        if key not in _route_cost_cache:
            _route_cost_cache[key] = sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))
        return _route_cost_cache[key]

    def calculate_total_cost(routes):
        if not routes:
            return float('inf')
        distance_cost = sum(route_cost(r) for r in routes)
        vehicle_overflow = max(0, len(routes) - max_vehicles)
        # 교차 페널티 추가
        crossing_penalty = calculate_crossing_penalty(routes) * 100
        return distance_cost + PENALTY_FACTOR * vehicle_overflow + crossing_penalty

    def calculate_crossing_penalty(routes):
        """경로 교차에 대한 페널티 계산"""
        penalty = 0
        n_routes = len(routes)

        for i in range(n_routes):
            for j in range(i + 1, n_routes):
                route1, route2 = routes[i], routes[j]
                for k in range(len(route1) - 1):
                    p1, p2 = route1[k], route1[k + 1]
                    for l in range(len(route2) - 1):
                        q1, q2 = route2[l], route2[l + 1]
                        if segments_intersect(p1, p2, q1, q2, dist):
                            penalty += 1
        return penalty

    def segments_intersect(p1, p2, q1, q2, dist):
        """두 선분이 교차하는지 확인 (간단한 거리 기반 휴리스틱)"""
        if p1 == q1 or p1 == q2 or p2 == q1 or p2 == q2:
            return False

        # 교차 휴리스틱: 대각선 거리 < 직선 거리 합
        diagonal_sum = dist[p1][q1] + dist[p2][q2]
        straight_sum = dist[p1][p2] + dist[q1][q2]

        return diagonal_sum < straight_sum * 0.95

    def roulette_select(weights):
        total = sum(weights)
        if total == 0:
            return random.randint(0, len(weights) - 1)
        r = random.uniform(0, total)
        acc = 0
        for i, w in enumerate(weights):
            acc += w
            if acc >= r:
                return i
        return len(weights) - 1

    def update_weights(weights, scores, counts, reaction=0.15):
        for i in range(len(weights)):
            if counts[i] > 0:
                avg_score = scores[i] / counts[i]
                weights[i] = (1 - reaction) * weights[i] + reaction * avg_score
            else:
                weights[i] = max(0.1, weights[i] * 0.95)
            scores[i], counts[i] = 0, 0

    def get_remove_count(routes, min_ratio=0.1, max_ratio=0.25, hard_limit_ratio=1):
        """
        제거 비율을 정밀하게 제어하는 버전
        hard_limit_ratio: 제거 개수의 절대적 상한 비율 (기본값: 1/3)
        """
        total_nodes = sum(len(r) - 2 for r in routes)
        if total_nodes == 0:
            return 0

        ratio = random.uniform(min_ratio, max_ratio)

        # total_nodes // 3 대신 hard_limit_ratio 사용
        hard_limit_count = int(total_nodes * hard_limit_ratio)

        # hard_limit_count가 0이 되는 것을 방지
        if hard_limit_count == 0 and total_nodes > 0:
            hard_limit_count = total_nodes

        return max(1, min(hard_limit_count, int(total_nodes * ratio)))

    def random_removal(routes, unassigned, iteration):
        flat_nodes = [n for r in routes for n in r[1:-1]]
        if not flat_nodes:
            return routes
        num_remove = get_remove_count(routes)
        to_remove = set(random.sample(flat_nodes, min(num_remove, len(flat_nodes))))
        unassigned.extend(to_remove)
        new_routes = [[v for v in r if v not in to_remove or v == depot_idx] for r in routes]
        return [r for r in new_routes if len(r) > 2]

    def shaw_removal(routes, unassigned, iteration):
        all_nodes = [n for r in routes for n in r[1:-1]]
        if not all_nodes:
            return routes
        remove_count = get_remove_count(routes)
        if len(all_nodes) <= remove_count:
            unassigned.extend(all_nodes)
            return []

        n0 = random.choice(all_nodes)
        removed = {n0}
        max_dist = max(max(row) for row in dist)
        max_demand = max(demands) if max(demands) > 0 else 1

        def similarity(n1, n2):
            dx = dist[n1][n2] / max_dist
            dt = abs(demands[n1] - demands[n2]) / max_demand
            tt = 1 if node_types[n1] != node_types[n2] else 0
            return 0.4 * dx + 0.4 * dt + 0.2 * tt

        while len(removed) < remove_count:
            candidates = [n for n in all_nodes if n not in removed]
            if not candidates:
                break
            # 더 공격적인 유사성 기반 선택
            if random.random() < 0.9:
                n_next = min(candidates, key=lambda n: similarity(n0, n))
            else:
                n_next = random.choice(candidates)
            removed.add(n_next)

        unassigned.extend(removed)
        new_routes = [[v for v in r if v not in removed or v == depot_idx] for r in routes]
        return [r for r in new_routes if len(r) > 2]

    def worst_removal(routes, unassigned, iteration):
        remove_count = get_remove_count(routes)
        cost_contrib = []
        for r in routes:
            for i in range(1, len(r) - 1):
                prev, curr, nxt = r[i - 1], r[i], r[i + 1]
                added_cost = dist[prev][curr] + dist[curr][nxt] - dist[prev][nxt]
                cost_contrib.append((added_cost, curr))
        if not cost_contrib:
            return routes
        cost_contrib.sort(reverse=True)
        # 더 공격적인 worst 노드 선택
        top_worst = min(remove_count, len(cost_contrib))
        selected = cost_contrib[:top_worst]
        removed = set(n for _, n in selected)
        unassigned.extend(removed)
        new_routes = [[v for v in r if v not in removed or v == depot_idx] for r in routes]
        return [r for r in new_routes if len(r) > 2]

    def route_removal(routes, unassigned, iteration):
        if not routes:
            return routes
        # 길고 비효율적인 루트 우선 제거
        route_scores = []
        for r in routes:
            if len(r) <= 3:
                continue
            efficiency = route_cost(r) / max(1, len(r) - 2)  # 노드당 비용
            route_scores.append((efficiency, r))

        if route_scores:
            route_scores.sort(reverse=True)  # 비효율적인 것부터
            k = min(max(1, len(routes) // 8), len(route_scores))
            to_remove = [r for _, r in route_scores[:k]]
        else:
            k = min(max(1, len(routes) // 6), len(routes))
            to_remove = random.sample(routes, k)

        removed_nodes = [n for r in to_remove for n in r[1:-1]]
        unassigned.extend(removed_nodes)
        new_routes = [r for r in routes if r not in to_remove]
        return new_routes

    def cluster_aware_greedy_insert(unassigned, partial_routes):
        """강화된 클러스터 인식 삽입"""
        removed_nodes = unassigned[:]
        delivery_nodes = [n for n in removed_nodes if node_types[n] == 0]
        pickup_nodes = sorted([n for n in removed_nodes if node_types[n] == 1], key=lambda n: -demands[n])

        inserted = []

        # 클러스터 기반 delivery 삽입
        delivery_clusters = build_clusters(delivery_nodes, dist, cluster_threshold=80)

        for cluster in delivery_clusters:
            cluster.sort(key=lambda n: dist[depot_idx][n])  # depot에서 가까운 순

            for n in cluster:
                if n not in unassigned:
                    continue

                best_cost_increase = float('inf')
                best_insertion = None

                for r_idx, r in enumerate(partial_routes):
                    pickup_start = len(r)
                    for i in range(1, len(r)):
                        if node_types[r[i]] == 1:
                            pickup_start = i
                            break

                    for i in range(1, pickup_start):
                        temp_route = r[:i] + [n] + r[i:]
                        if is_route_feasible(temp_route, node_types, demands, capa, depot_idx):
                            cost_increase = dist[r[i - 1]][n] + dist[n][r[i]] - dist[r[i - 1]][r[i]]

                            # 강화된 클러스터 보너스
                            cluster_bonus = 0
                            for existing_node in r[1:pickup_start]:
                                if node_types[existing_node] == 0:
                                    distance = dist[n][existing_node]
                                    if distance < 60:
                                        cluster_bonus += max(0, 20 - distance / 3)

                            adjusted_cost = cost_increase - cluster_bonus
                            if adjusted_cost < best_cost_increase:
                                best_cost_increase = adjusted_cost
                                best_insertion = (r_idx, i)

                if best_insertion:
                    r_idx, pos = best_insertion
                    partial_routes[r_idx].insert(pos, n)
                    inserted.append(n)
                else:
                    if len(partial_routes) < max_vehicles:
                        new_route = [depot_idx, n, depot_idx]
                        if is_route_feasible(new_route, node_types, demands, capa, depot_idx):
                            partial_routes.append(new_route)
                            inserted.append(n)

        # 픽업 노드 삽입
        for n in pickup_nodes:
            if n not in unassigned:
                continue

            best_cost_increase = float('inf')
            best_insertion = None

            for r_idx, r in enumerate(partial_routes):
                delivery_end = 0
                for i in range(1, len(r)):
                    if node_types[r[i]] == 1:
                        delivery_end = i
                    else:
                        break

                for i in range(delivery_end + 1, len(r)):
                    temp_route = r[:i] + [n] + r[i:]
                    if is_route_feasible(temp_route, node_types, demands, capa, depot_idx):
                        cost_increase = dist[r[i - 1]][n] + dist[n][r[i]] - dist[r[i - 1]][r[i]]

                        # 픽업 클러스터 보너스
                        cluster_bonus = 0
                        for existing_node in r[delivery_end + 1:]:
                            if node_types[existing_node] == 1:
                                distance = dist[n][existing_node]
                                if distance < 40:
                                    cluster_bonus += max(0, 15 - distance / 4)

                        adjusted_cost = cost_increase - cluster_bonus
                        if adjusted_cost < best_cost_increase:
                            best_cost_increase = adjusted_cost
                            best_insertion = (r_idx, i)

            if best_insertion:
                r_idx, pos = best_insertion
                partial_routes[r_idx].insert(pos, n)
                inserted.append(n)
            else:
                if len(partial_routes) < max_vehicles:
                    new_route = [depot_idx, n, depot_idx]
                    if is_route_feasible(new_route, node_types, demands, capa, depot_idx):
                        partial_routes.append(new_route)
                        inserted.append(n)

        for n in inserted:
            if n in unassigned:
                unassigned.remove(n)
        return partial_routes

    def build_clusters(nodes, dist, cluster_threshold=80):
        """거리 기반 클러스터링"""
        if not nodes:
            return []

        clusters = []
        remaining = nodes[:]

        while remaining:
            cluster = [remaining.pop(0)]
            i = 0
            while i < len(remaining):
                node = remaining[i]
                # 클러스터 내 모든 노드와의 평균 거리 계산
                avg_dist = sum(dist[node][c] for c in cluster) / len(cluster)
                if avg_dist <= cluster_threshold:
                    cluster.append(remaining.pop(i))
                else:
                    i += 1
            clusters.append(cluster)

        return clusters

    def regret_k_insert(unassigned, partial_routes, k=3):
        """개선된 regret-k 삽입"""
        delivery_nodes = [n for n in unassigned if node_types[n] == 0]
        pickup_nodes = sorted([n for n in unassigned if node_types[n] == 1], key=lambda n: -demands[n])

        inserted = []

        # Delivery regret-k
        remaining_nodes = delivery_nodes[:]
        while remaining_nodes:
            regrets = []
            for n in remaining_nodes:
                costs = []
                for r_idx, r in enumerate(partial_routes):
                    pickup_start = len(r)
                    for i in range(1, len(r)):
                        if node_types[r[i]] == 1:
                            pickup_start = i
                            break
                    for i in range(1, pickup_start):
                        temp_route = r[:i] + [n] + r[i:]
                        if is_route_feasible(temp_route, node_types, demands, capa, depot_idx):
                            cost_increase = dist[r[i - 1]][n] + dist[n][r[i]] - dist[r[i - 1]][r[i]]
                            costs.append((cost_increase, r_idx, i))

                # 새 루트 생성 비용도 고려
                if len(partial_routes) < max_vehicles:
                    new_route_cost = dist[depot_idx][n] + dist[n][depot_idx]
                    costs.append((new_route_cost, -1, -1))

                if costs:
                    costs.sort()
                    regret = sum(costs[j][0] - costs[0][0] for j in range(1, min(k, len(costs))))
                    regrets.append((regret, n, costs[0][1], costs[0][2]))

            if not regrets:
                break
            regrets.sort(reverse=True)
            _, n, r_idx, pos = regrets[0]

            if r_idx == -1:  # 새 루트
                new_route = [depot_idx, n, depot_idx]
                partial_routes.append(new_route)
            else:
                partial_routes[r_idx].insert(pos, n)

            inserted.append(n)
            remaining_nodes.remove(n)

        # Pickup regret-k (동일한 로직)
        remaining_pickup = pickup_nodes[:]
        while remaining_pickup:
            regrets = []
            for n in remaining_pickup:
                costs = []
                for r_idx, r in enumerate(partial_routes):
                    delivery_end = 0
                    for i in range(1, len(r)):
                        if node_types[r[i]] == 1:
                            delivery_end = i
                        else:
                            break
                    for i in range(delivery_end + 1, len(r)):
                        temp_route = r[:i] + [n] + r[i:]
                        if is_route_feasible(temp_route, node_types, demands, capa, depot_idx):
                            cost_increase = dist[r[i - 1]][n] + dist[n][r[i]] - dist[r[i - 1]][r[i]]
                            costs.append((cost_increase, r_idx, i))

                if len(partial_routes) < max_vehicles:
                    new_route_cost = dist[depot_idx][n] + dist[n][depot_idx]
                    costs.append((new_route_cost, -1, -1))

                if costs:
                    costs.sort()
                    regret = sum(costs[j][0] - costs[0][0] for j in range(1, min(k, len(costs))))
                    regrets.append((regret, n, costs[0][1], costs[0][2]))

            if not regrets:
                break
            regrets.sort(reverse=True)
            _, n, r_idx, pos = regrets[0]

            if r_idx == -1:
                new_route = [depot_idx, n, depot_idx]
                partial_routes.append(new_route)
            else:
                partial_routes[r_idx].insert(pos, n)

            inserted.append(n)
            remaining_pickup.remove(n)

        unassigned[:] = [n for n in unassigned if n not in inserted]
        return partial_routes

    def force_insert_all(unassigned, partial_routes):
        for n in unassigned[:]:
            inserted = False
            for r_idx, r in enumerate(partial_routes):
                for i in range(1, len(r)):
                    temp_route = r[:i] + [n] + r[i:]
                    if is_route_feasible(temp_route, node_types, demands, capa, depot_idx):
                        partial_routes[r_idx].insert(i, n)
                        inserted = True
                        break
                if inserted:
                    break
            if not inserted:
                if len(partial_routes) < max_vehicles * 1.5:
                    new_route = [depot_idx, n, depot_idx]
                    partial_routes.append(new_route)
        unassigned.clear()
        return partial_routes

    def intensive_local_search(routes, deadline):
        """집중적인 지역 탐색 (시간 제한 인지)"""
        improved = True
        search_count = 0

        while improved and search_count < 5:
            # 🛡️ 데드라인을 직접 확인
            if time.time() >= deadline:
                break

            improved = False
            search_count += 1
            # 1. 경로 내 2-opt
            if two_opt_intra_route(routes):
                improved = True

            # 2. Or-opt
            if or_opt_move(routes):
                improved = True

            # 3. 경로 간 재배치
            if relocate_nodes(routes):
                improved = True

            # 4. 교차 해결에 특화된 2-opt
            if enhanced_cross_exchange(routes):
                improved = True

        return search_count > 1

    def enhanced_cross_exchange(routes):
        """교차 해결에 특화된 교환"""
        improved = False
        n_routes = len(routes)

        # 교차하는 경로 쌍을 우선적으로 처리
        crossing_pairs = []
        for i in range(n_routes):
            for j in range(i + 1, n_routes):
                if routes_cross(routes[i], routes[j], dist):
                    crossing_pairs.append((i, j))

        # 교차하는 쌍부터 처리
        for i, j in crossing_pairs:
            route1, route2 = routes[i][:], routes[j][:]
            if len(route1) <= 3 or len(route2) <= 3:
                continue

            best_improvement = 0
            best_r1, best_r2 = route1[:], route2[:]
            original_cost = route_cost(route1) + route_cost(route2)

            # 다양한 교환 시도
            for seg1_len in range(1, min(4, len(route1) - 2)):
                for seg1_start in range(1, len(route1) - seg1_len):
                    for seg2_len in range(1, min(4, len(route2) - 2)):
                        for seg2_start in range(1, len(route2) - seg2_len):

                            seg1 = route1[seg1_start:seg1_start + seg1_len]
                            seg2 = route2[seg2_start:seg2_start + seg2_len]

                            new_r1 = route1[:seg1_start] + seg2 + route1[seg1_start + seg1_len:]
                            new_r2 = route2[:seg2_start] + seg1 + route2[seg2_start + seg2_len:]

                            if (is_route_feasible(new_r1, node_types, demands, capa, depot_idx) and
                                    is_route_feasible(new_r2, node_types, demands, capa, depot_idx)):
                                new_cost = route_cost(new_r1) + route_cost(new_r2)
                                improvement = original_cost - new_cost

                                # 교차 해결 보너스
                                if not routes_cross(new_r1, new_r2, dist):
                                    improvement += 50  # 교차 해결 보너스

                                if improvement > best_improvement:
                                    best_improvement = improvement
                                    best_r1, best_r2 = new_r1[:], new_r2[:]
                                    improved = True

            routes[i], routes[j] = best_r1, best_r2

        return improved

    def routes_cross(route1, route2, dist):
        """두 경로가 교차하는지 확인"""
        for i in range(len(route1) - 1):
            for j in range(len(route2) - 1):
                if segments_intersect(route1[i], route1[i + 1], route2[j], route2[j + 1], dist):
                    return True
        return False

    def or_opt_move(routes):
        """개선된 Or-opt"""
        improved = False
        for r_idx, route in enumerate(routes):
            if len(route) <= 4:
                continue
            best_route = route[:]
            best_cost = route_cost(route)

            # 1-node moves
            for i in range(1, len(route) - 1):
                node = route[i]
                temp_route = route[:i] + route[i + 1:]
                for j in range(1, len(temp_route)):
                    new_route = temp_route[:j] + [node] + temp_route[j:]
                    if is_route_feasible(new_route, node_types, demands, capa, depot_idx):
                        new_cost = route_cost(new_route)
                        if new_cost < best_cost:
                            best_route = new_route
                            best_cost = new_cost
                            improved = True

            # 2-node moves (연속된 노드)
            for i in range(1, len(route) - 2):
                if (node_types[route[i]] == node_types[route[i + 1]]):  # 같은 타입만
                    nodes = route[i:i + 2]
                    temp_route = route[:i] + route[i + 2:]
                    for j in range(1, len(temp_route)):
                        new_route = temp_route[:j] + nodes + temp_route[j:]
                        if is_route_feasible(new_route, node_types, demands, capa, depot_idx):
                            new_cost = route_cost(new_route)
                            if new_cost < best_cost:
                                best_route = new_route
                                best_cost = new_cost
                                improved = True

            routes[r_idx] = best_route
        return improved

    def relocate_nodes(routes):
        """개선된 노드 재배치"""
        improved = False
        n_routes = len(routes)

        for i in range(n_routes):
            route1 = routes[i][:]
            if len(route1) <= 3:
                continue

            for node_idx in range(1, len(route1) - 1):
                node = route1[node_idx]
                temp_route1 = route1[:node_idx] + route1[node_idx + 1:]

                for j in range(n_routes):
                    if i == j:
                        continue
                    route2 = routes[j][:]

                    best_improvement = 0
                    best_pos = -1
                    original_cost = route_cost(route1) + route_cost(route2)

                    # 타입에 맞는 위치 찾기
                    valid_positions = []
                    if node_types[node] == 0:  # delivery
                        for pos in range(1, len(route2)):
                            if pos < len(route2) and node_types[route2[pos]] == 1:
                                break
                            valid_positions.append(pos)
                    else:  # pickup
                        delivery_end = len(route2) - 1
                        for pos in range(1, len(route2)):
                            if node_types[route2[pos]] == 1:
                                delivery_end = pos
                                break
                        valid_positions = list(range(delivery_end + 1, len(route2)))

                    for pos in valid_positions:
                        new_route2 = route2[:pos] + [node] + route2[pos:]
                        if is_route_feasible(new_route2, node_types, demands, capa, depot_idx):
                            new_cost = route_cost(temp_route1) + route_cost(new_route2)
                            improvement = original_cost - new_cost

                            if improvement > best_improvement:
                                best_improvement = improvement
                                best_pos = pos

                    if best_pos != -1 and best_improvement > 0.1:
                        routes[i] = temp_route1
                        routes[j] = route2[:best_pos] + [node] + route2[best_pos:]
                        improved = True
                        break
                if improved:
                    break
        return improved

    def two_opt_intra_route(routes):
        """경로 내 2-opt"""
        improved = False
        for r_idx, route in enumerate(routes):
            if len(route) <= 4:
                continue
            best_route = route[:]
            best_cost = route_cost(route)

            for i in range(1, len(route) - 2):
                for j in range(i + 1, len(route) - 1):
                    new_route = route[:i] + route[i:j + 1][::-1] + route[j + 1:]
                    if is_route_feasible(new_route, node_types, demands, capa, depot_idx):
                        new_cost = route_cost(new_route)
                        if new_cost < best_cost:
                            best_route = new_route
                            best_cost = new_cost
                            improved = True
            routes[r_idx] = best_route
        return improved

    # 초기 해 검증
    if not solution_feasible(init_routes, node_types, demands, capa, depot_idx, verbose):
        log("[ERROR] Initial solution is infeasible!", force=True)
        return init_routes, float('inf')

    best_routes = [r[:] for r in init_routes]
    best_cost = calculate_total_cost(best_routes)
    cur_routes = [r[:] for r in best_routes]
    cur_cost = best_cost

    destroy_ops = [random_removal, shaw_removal, worst_removal, route_removal]
    repair_ops = [cluster_aware_greedy_insert, regret_k_insert]

    d_weights = [1.0] * len(destroy_ops)
    r_weights = [1.0] * len(repair_ops)
    d_scores = [0] * len(destroy_ops)
    r_scores = [0] * len(repair_ops)
    d_counts = [0] * len(destroy_ops)
    r_counts = [0] * len(repair_ops)

    # 더 보수적인 온도 스케줄링
    T = best_cost * 0.1  # 낮은 초기 온도
    T_min, T_max = 0.01, best_cost * 0.3
    target_accept = 0.15  # 낮은 수용률
    log(f"[INFO] Initial solution: cost={best_cost:.1f}, routes={len(best_routes)}")

    # 변수 초기화는 while 문 이전에 한 번만 수행
    unassigned_nodes = []
    iteration = 0
    start_time_for_log = time.time()
    last_improvement = 0
    accepted, attempted = 0, 0

    # ★★★ 3. 메인 루프 조건을 deadline 기준으로 변경 ★★★
    while time.time() < deadline:
        unassigned_nodes.clear()
        iteration += 1
        elapsed = time.time() - start_time_for_log

        # 집중적 지역 탐색 호출 시 deadline 전달
        if iteration % 20 == 0:

            # ★★★ 2-1. 스위치를 확인하여 로컬 서치 실행 여부 결정 ★★★
            # enable_local_search가 True일 때만 주기적인 로컬 서치를 실행합니다.
            if enable_local_search and iteration % 20 == 0:
                if intensive_local_search(cur_routes, deadline):
                    new_cost = calculate_total_cost(cur_routes)
                    if new_cost < cur_cost:
                        cur_cost = new_cost
                        if new_cost < best_cost:
                            best_cost, best_routes = new_cost, [r[:] for r in cur_routes]
                            log(f"[LOCAL] Iter {iteration}: Local search improved to {best_cost:.1f}")

        max_retry = 3  # 재시도 횟수 줄임
        for retry in range(max_retry):
            # Diversification 전략 조정
            if iteration - last_improvement > 150:
                d_idx = roulette_select([1.5, 4.0, 2.0, 3.0])  # shaw와 route_removal 강화
            else:
                d_idx = roulette_select(d_weights)

            # repair 선택 개선
            if iteration < 100:
                r_idx = 0  # 초기에는 cluster_aware_greedy_insert 우선
            else:
                r_idx = roulette_select(r_weights)

            unassigned_nodes.clear()
            # ★★★★★ 핵심 수정 사항 ★★★★★
            # destroy 연산을 시작하기 전에 cur_routes의 '깊은 복사본'을 만들어서 전달합니다.
            # 이렇게 하면 원본 cur_routes가 오염되는 것을 완벽하게 막을 수 있습니다.
            temp_routes = destroy_ops[d_idx]([r[:] for r in cur_routes], unassigned_nodes, iteration)

            if repair_ops[r_idx] == regret_k_insert:
                temp_routes = repair_ops[r_idx](unassigned_nodes, temp_routes, k=3)
            else:
                temp_routes = repair_ops[r_idx](unassigned_nodes, temp_routes)

            if unassigned_nodes:
                temp_routes = force_insert_all(unassigned_nodes, temp_routes)

            if solution_feasible(temp_routes, node_types, demands, capa, depot_idx, verbose):
                break

        else:
            continue

        cur_routes = temp_routes
        new_cost = calculate_total_cost(cur_routes)
        attempted += 1
        d_counts[d_idx] += 1
        r_counts[r_idx] += 1
        accept = False

        if new_cost < best_cost:
            improvement = best_cost - new_cost
            best_cost, best_routes = new_cost, [r[:] for r in cur_routes]
            cur_cost, cur_routes = new_cost, [r[:] for r in cur_routes]
            d_scores[d_idx] += 20
            r_scores[r_idx] += 20
            accept = True
            last_improvement = iteration
            log(f"[BEST] Iter {iteration}: New best cost {best_cost:.1f} (improved by {improvement:.1f})")
        elif new_cost < cur_cost:
            cur_cost, cur_routes = new_cost, [r[:] for r in cur_routes]
            d_scores[d_idx] += 10
            r_scores[r_idx] += 10
            accept = True
        else:
            delta = cur_cost - new_cost
            if T > 0 and random.random() < math.exp(delta / T):
                cur_cost, cur_routes = new_cost, [r[:] for r in cur_routes]
                d_scores[d_idx] += 1
                r_scores[r_idx] += 1
                accept = True

        if accept:
            accepted += 1

        # 온도 조절 개선
        if attempted > 25 and iteration % 15 == 0:
            real_acc = accepted / attempted
            if real_acc < target_accept - 0.05:
                T *= 1.1
            elif real_acc > target_accept + 0.05:
                T *= 0.9
            T = max(T_min, min(T, T_max))
            accepted = attempted = 0

        # 가중치 업데이트
        if iteration % 40 == 0:
            update_weights(d_weights, d_scores, d_counts, reaction=0.2)
            update_weights(r_weights, r_scores, r_counts, reaction=0.2)

        # 진행 상황 출력 (verbose일 때만)
        if iteration % 150 == 0:
            crossings = calculate_crossing_penalty(cur_routes)
            log(
                f"[ALNS][{elapsed:.1f}s] Iter {iteration:4d}: Current={cur_cost:.1f} ({len(cur_routes)} routes, {crossings} crossings), Best={best_cost:.1f} ({len(best_routes)} routes), T={T:.3f}")

    # 최종 집중적 지역 탐색
    log("[INFO] Final intensive local search...")
    # ★★★ 2-2. 스위치를 확인하여 최종 로컬 서치 실행 여부 결정 ★★★
    final_improved = False  # 기본값은 False
    if enable_local_search:
        final_improved = intensive_local_search(best_routes, deadline)

    if final_improved:
        final_cost = calculate_total_cost(best_routes)
        if final_cost < best_cost:
            best_cost = final_cost
            log(f"[FINAL] Final local search improved to {best_cost:.1f}")

    log(f"[INFO] Improved ALNS finished. Final best cost: {best_cost:.1f}")

    # 최종 결과 분석 (verbose일 때만)
    if verbose:
        visited = set(n for r in best_routes for n in r if n != depot_idx)
        expected = set(i for i in range(len(node_types)) if i != depot_idx)
        missed = expected - visited

        if missed:
            log(f"[❌] 방문하지 않은 노드: {sorted(missed)}", force=True)
        else:
            log(f"[✅] 모든 노드 방문 완료")

        final_crossings = calculate_crossing_penalty(best_routes)
        log(f"[INFO] 최종 교차 개수: {final_crossings}")

    return best_routes, best_cost