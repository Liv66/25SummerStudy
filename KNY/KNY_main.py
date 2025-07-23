"""
KNY_main_improved.py  —  기존 KNY_VRPB 파이프라인 + 성능 최적화 패치 + DistanceCache 활용
──────────────────────────────────────────────────────────────────────────
* 달라진 점
  1. fast_feasible_check  : KJH 변환 없이 경량 VRPB 제약 검사
  2. find_best_insertion_fast : O(n) 수준의 pickup 삽입 탐색
  3. improved_initial_solution / improved_greedy_vrpb : delivery‑pickup 페어링 기반
  4. DistanceCache        : 루트 / 삽입 비용 캐싱 (실제 활용)
  5. run_kjh_problem      : 초기해 생성 함수를 개선 버전으로 교체

※ 내부 util · KNY_alns 구조(인터페이스)·check_feasible 는 기존과 같다고 가정.
"""

import json
import random
import math
import time
from pathlib import Path
from typing import List

from util import get_distance, plot_cvrp, check_feasible
from KNY_alns import alns_vrpb

random.seed(42)

# ─────────────────────────────────────────────────────────────
# 0) 공용 도우미 함수 / 클래스
# ─────────────────────────────────────────────────────────────

def route_cost(route: list[int], dist: list[list[float]]) -> float:
    """단일 루트 비용 합산"""
    return sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))


class DistanceCache:
    """거리 계산 캐싱: (route 튜플) → cost"""

    def __init__(self, dist_matrix: List[List[float]]):
        self.dist = dist_matrix
        self.cache = {}
        self.insertion_cache = {}

    def get_route_cost(self, route):
        key = tuple(route)
        if key not in self.cache:
            self.cache[key] = route_cost(route, self.dist)
        return self.cache[key]

    def get_insertion_cost(self, route, node, pos):
        # 삽입 비용 캐싱: (route_tuple, node, pos) → cost
        route_key = tuple(route)
        key = (route_key, node, pos)
        if key not in self.insertion_cache:
            if pos >= len(route):
                return float('inf')
            self.insertion_cache[key] = (
                self.dist[route[pos - 1]][node]
                + self.dist[node][route[pos]]
                - self.dist[route[pos - 1]][route[pos]]
            )
        return self.insertion_cache[key]

    def clear_cache(self):
        """캐시 초기화 (메모리 관리용)"""
        self.cache.clear()
        self.insertion_cache.clear()


# 전역 캐시 인스턴스
cache = None


# ─────────────────────────────────────────────────────────────
# 1) 경량 feasibility 체크 (KJH 변환 제거)
# ─────────────────────────────────────────────────────────────

def fast_feasible_check(
    route: List[int],
    node_types: List[int],
    demands: List[int],
    capa: int,
    depot_idx: int,
):
    if route[0] != depot_idx or route[-1] != depot_idx:
        return False  # depot 출발·복귀 확실히 체크

    if len(route) < 3:
        return True

    if node_types[route[1]] == 0:  # backhaul부터 시작하면 불가
        return False

    load = 0
    in_pickup = False

    for n in route[1:-1]:
        if node_types[n] == 1:  # delivery
            if in_pickup:
                return False  # pickup 이후 delivery 금지
            load += demands[n]
        else:  # pickup
            if not in_pickup:
                in_pickup = True
                load = 0  # pickup 시작 시 적재초기화
            load += demands[n]

        if load > capa:
            return False

    return True


# ─────────────────────────────────────────────────────────────
# 2) O(n) 수준 pickup 삽입 탐색 (캐시 활용)
# ─────────────────────────────────────────────────────────────

def find_best_insertion_fast(
    pickup_node: int,
    routes: List[List[int]],
    node_types: List[int],
    demands: List[int],
    capa: int,
    dist: List[List[float]],
    depot_idx: int,
):
    global cache
    best_route_idx, best_pos, best_inc = None, None, float("inf")

    for ridx, r in enumerate(routes):
        deliv_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
        if deliv_load == 0:
            continue
        pickup_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 0)
        if pickup_load + demands[pickup_node] > deliv_load:
            continue

        last_deliv = max((i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1), default=0)
        for pos in range(last_deliv + 1, len(r)):
            tmp = r[:pos] + [pickup_node] + r[pos:]
            if not fast_feasible_check(tmp, node_types, demands, capa, depot_idx):
                continue

            # 캐시를 사용한 삽입 비용 계산
            if cache:
                inc = cache.get_insertion_cost(r, pickup_node, pos)
            else:
                inc = (
                    dist[r[pos - 1]][pickup_node]
                    + dist[pickup_node][r[pos]]
                    - dist[r[pos - 1]][r[pos]]
                )

            if inc < best_inc:
                best_inc = inc
                best_route_idx, best_pos = ridx, pos

    return best_route_idx, best_pos, best_inc


# ─────────────────────────────────────────────────────────────
# 3) 개선된 초기해 생성 (delivery‑pickup 페어링, 캐시 활용)
# ─────────────────────────────────────────────────────────────

def improved_initial_solution(
    delivery_idx: List[int],
    pickup_idx: List[int],
    demands: List[int],
    capa: int,
    dist: List[List[float]],
    depot_idx: int,
    node_types: List[int],
    K: int,
):
    global cache
    routes = []
    used_deliv, used_pick = set(), set()

    deliv_items = sorted(((n, demands[n]) for n in delivery_idx), key=lambda x: x[1], reverse=True)
    pickup_items = [(n, demands[n]) for n in pickup_idx]

    for d_node, d_dem in deliv_items:
        if d_node in used_deliv:
            continue
        cur_route = [depot_idx, d_node]
        cur_load = d_dem
        used_deliv.add(d_node)

        # 추가 delivery 탐색
        for o_node, o_dem in deliv_items:
            if o_node in used_deliv:
                continue
            if cur_load + o_dem > capa:
                continue
            if dist[d_node][o_node] > dist[d_node][depot_idx] * 1.5:
                continue
            cur_route.insert(-1, o_node)
            cur_load += o_dem
            used_deliv.add(o_node)

        # pickup 삽입
        avail_pick_cap = cur_load
        cand = []
        last_deliv = cur_route[-1] if len(cur_route) > 2 else cur_route[-2]
        for p_node, p_dem in pickup_items:
            if p_node in used_pick or p_dem > avail_pick_cap:
                continue
            cand.append((p_node, p_dem, dist[last_deliv][p_node]))
        cand.sort(key=lambda x: x[2])

        pick_load = 0
        for p_node, p_dem, _ in cand:
            if pick_load + p_dem <= avail_pick_cap:
                cur_route.append(p_node)
                pick_load += p_dem
                used_pick.add(p_node)

        cur_route.append(depot_idx)
        routes.append(cur_route)
        if len(routes) >= K:
            break

    # 남은 delivery → 기존 루트 삽입 또는 새 루트
    rem_deliv = [n for n in delivery_idx if n not in used_deliv]
    for d in rem_deliv:
        placed = False
        for r in routes:
            load = sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
            if load + demands[d] <= capa:
                last_deliv_idx = max((i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1), default=0)
                r.insert(last_deliv_idx + 1, d)
                placed = True
                break
        if not placed and len(routes) < K:
            routes.append([depot_idx, d, depot_idx])

    # 남은 pickup 빠른 삽입
    rem_pick = [n for n in pickup_idx if n not in used_pick]
    for p in rem_pick:
        ridx, pos, _ = find_best_insertion_fast(p, routes, node_types, demands, capa, dist, depot_idx)
        if ridx is not None:
            routes[ridx].insert(pos, p)

    return routes


# ─────────────────────────────────────────────────────────────
# 4) Improved Greedy VRPB Wrapper (캐시 활용)
# ─────────────────────────────────────────────────────────────

def improved_greedy_vrpb(
    delivery_idx,
    pickup_idx,
    demands,
    capa,
    dist,
    depot_idx,
    node_types,
    K,
):
    global cache
    print("[INFO] 개선된 Greedy VRPB 초기화…")

    # 캐시 초기화
    if cache:
        cache.clear_cache()

    routes = improved_initial_solution(
        delivery_idx,
        pickup_idx,
        demands,
        capa,
        dist,
        depot_idx,
        node_types,
        K,
    )

    # 추가로 남은 pickup 확인 (안전)
    for p in pickup_idx:
        if any(p in r for r in routes):
            continue
        ridx, pos, _ = find_best_insertion_fast(p, routes, node_types, demands, capa, dist, depot_idx)
        if ridx is not None:
            routes[ridx].insert(pos, p)
        elif len(routes) < K:
            routes.append([depot_idx, p, depot_idx])
        else:
            # 가장 여유 있는 루트에 강제 삽입
            best_r = max(
                routes,
                key=lambda r: sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
                - sum(demands[v] for v in r[1:-1] if node_types[v] == 0),
            )
            last_deliv = max((i for i, v in enumerate(best_r) if v != depot_idx and node_types[v] == 1), default=0)
            best_r.insert(last_deliv + 1, p)

    print(f"[INFO] 개선된 Greedy 완료 · Route 수 = {len(routes)} (K={K})")
    return routes


# ─────────────────────────────────────────────────────────────
# 5) KJH JSON Adapter (변경 없음)
# ─────────────────────────────────────────────────────────────

def convert_kjh_problem(problem_info: dict):
    capa = problem_info["capa"]
    coords = problem_info["node_coords"]
    demands_all = [abs(d) for d in problem_info["node_demands"]]
    types_all = problem_info["node_types"]

    delivery_idx, pickup_idx = [], []
    node_types_internal = [0] * len(coords)
    demands_internal = [0] * len(coords)

    for j in range(1, len(coords)):
        is_delivery = 1 if types_all[j] == 1 else 0
        node_types_internal[j] = is_delivery
        demands_internal[j] = demands_all[j]
        (delivery_idx if is_delivery else pickup_idx).append(j)

    dist_matrix = problem_info["dist_mat"]
    depot_idx = 0

    return (
        delivery_idx,
        pickup_idx,
        demands_internal,
        capa,
        dist_matrix,
        depot_idx,
        node_types_internal,
        coords,
    )


def load_kjh_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        info = json.load(f)
    return convert_kjh_problem(info)


def to_kjh_routes(routes, depot_idx):
    return routes[:]


def to_kjh_types(node_types_internal):
    return [0] + [1 if t == 1 else 2 for t in node_types_internal]


# ─────────────────────────────────────────────────────────────
# 6) ALNS 후처리: cross‑route 2‑opt* (캐시 활용)
# ─────────────────────────────────────────────────────────────

def cross_route_2opt_star(routes, dist, node_types, demands, capa, depot_idx):
    global cache
    changed = True
    while changed:
        changed = False
        for i in range(len(routes)):
            for j in range(i + 1, len(routes)):
                r1, r2 = routes[i], routes[j]
                for idx1 in range(1, len(r1) - 1):
                    for idx2 in range(1, len(r2) - 1):
                        if node_types[r1[idx1]] != node_types[r2[idx2]]:
                            continue
                        new_r1 = r1[:idx1] + [r2[idx2]] + r1[idx1 + 1:]
                        new_r2 = r2[:idx2] + [r1[idx1]] + r2[idx2 + 1:]
                        if not fast_feasible_check(new_r1, node_types, demands, capa, depot_idx):
                            continue
                        if not fast_feasible_check(new_r2, node_types, demands, capa, depot_idx):
                            continue

                        # 캐시를 사용한 비용 계산
                        if cache:
                            old_cost = cache.get_route_cost(r1) + cache.get_route_cost(r2)
                            new_cost = cache.get_route_cost(new_r1) + cache.get_route_cost(new_r2)
                        else:
                            old_cost = route_cost(r1, dist) + route_cost(r2, dist)
                            new_cost = route_cost(new_r1, dist) + route_cost(new_r2, dist)

                        if old_cost > new_cost:
                            routes[i], routes[j] = new_r1, new_r2
                            changed = True
    return routes


# ─────────────────────────────────────────────────────────────
# 7) 메인 드라이버 (캐시 초기화 추가)
# ─────────────────────────────────────────────────────────────

def run_kjh_problem(problem_path: Path):
    global cache

    with open(problem_path, "r", encoding="utf-8") as f:
        problem_info = json.load(f)
    K = problem_info["K"]

    (
        delivery_idx,
        pickup_idx,
        demands,
        capa,
        dist,
        depot_idx,
        node_types,
        coords,
    ) = load_kjh_json(problem_path)

    # DistanceCache 초기화
    cache = DistanceCache(dist)
    print("[INFO] DistanceCache 초기화 완료")

    total_delivery = sum(demands[i] for i in delivery_idx)
    if math.ceil(total_delivery / capa) > K:
        raise ValueError("Instance infeasible: delivery 총수요가 K·capa 를 초과")

    init_routes = improved_greedy_vrpb(
        delivery_idx,
        pickup_idx,
        demands,
        capa,
        dist,
        depot_idx,
        node_types,
        K,
    )

    print(f"[INFO] Initial route count: {len(init_routes)}")

    # ── ALNS
    start = time.time()
    best_routes, _ = alns_vrpb(
        init_routes,
        dist,
        node_types,
        demands,
        capa,
        depot_idx,
        max_vehicles=K,
        time_limit=60,
    )
    elapsed = time.time() - start

    best_routes = cross_route_2opt_star(best_routes, dist, node_types, demands, capa, depot_idx)

    # 캐시를 사용한 최종 비용 계산
    best_cost = sum(cache.get_route_cost(r) for r in best_routes)

    # ── 최종 검증
    obj = check_feasible(problem_info, to_kjh_routes(best_routes, depot_idx), elapsed, timelimit=60)
    if obj:
        print(f"[✅] check_feasible 통과! obj = {obj:.1f}")
    else:
        print("[❌] check_feasible 실패")

    # 캐시 통계 출력
    print(f"[INFO] 캐시 통계 - Route 캐시: {len(cache.cache)}개, 삽입 캐시: {len(cache.insertion_cache)}개")

    capa_val = capa  # 편의상 별도 변수
    for k, r in enumerate(best_routes):
        # -------------- 적재율 계산 ------------------ #
        delivery_load = sum(demands[n] for n in r if node_types[n] == 1)
        # depot 출발 시 적재량 = 배송 총수요
        utilisation = delivery_load / capa_val  # 0.0 ~ 1.0
        print(f"vehicle {k:2d}: {r}")
        print(f"           ↳ 출발 적재량 = {delivery_load:>6.1f} / {capa_val}  "
              f"(utilisation {utilisation:5.1%})")  # ★

    plot_cvrp(coords, best_routes, f"VRPB obj: {best_cost:.1f}")


if __name__ == "__main__":
    ROOT = Path(__file__).resolve().parents[1]
    run_kjh_problem(ROOT / "instances" / "problem_20_0.7.json")