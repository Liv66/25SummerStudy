"""
KNY_main_improved.py  —  기존 KNY_VRPB 파이프라인 + 성능 최적화 패치
──────────────────────────────────────────────────────────────────────────
* 달라진 점
  1. fast_feasible_check  : KJH 변환 없이 경량 VRPB 제약 검사
  2. find_best_insertion_fast : O(n) 수준의 pickup 삽입 탐색
  3. improved_initial_solution / improved_greedy_vrpb : delivery‑pickup 페어링 기반
  4. DistanceCache        : 루트 / 삽입 비용 캐싱 (실제 활용)
  5. run_kjh_problem      : 초기해 생성 함수를 개선 버전으로 교체

※ 내부 util · KNY_alns 구조(인터페이스)·check_feasible 는 기존과 같다고 가정.
"""

import math
import time
from pathlib import Path
from typing import List
import csv  # <-- 추가
import os   # <-- 추가
import numpy as np # <-- 추가

from KNY.KNY_alns import alns_vrpb

# ▼▼▼ 여기에 스위치를 추가하세요 ▼▼▼
ENABLE_LOGGING = False  # True로 바꾸면 다시 저장이 활성화됩니다.

#random.seed(42)

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
) -> bool:
    # ── 0. 기본 조건 ────────────────────────
    if route[0] != depot_idx or route[-1] != depot_idx:
        return False
    if len(route) < 3:  # depot-node-depot
        return True
    if node_types[route[1]] == 0:  # pickup부터 출발? → 불가
        return False

    # ── 1. 적재량 / 플래그 초기화 ───────────
    load = 0
    in_pick = False  # 아직 pickup 구간 아님

    # ── 2. 경로 순회 ────────────────────────
    for prev, curr in zip(route[:-1], route[1:]):

        # 2-1. 구간 전환 감지 (delivery→pickup)
        if node_types[prev] == 1 and node_types[curr] == 0:
            if in_pick:  # 두 번째 전환? → infeasible
                return False
            in_pick = True
            load = 0  # ★ 배송 화물 모두 하차

        # 2-2. 현재 노드 적재/하차
        load += demands[curr]  # delivery든 pickup이든 +demands
        if load > capa:  # 용량 초과
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
            continue  # 회수만 있는 라우트에는 삽입하지 않음

        # ❌ 아래 두 줄 삭제 (불필요한 잘못된 제약)
        pickup_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 0)
        if pickup_load + demands[pickup_node] > capa:
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
        avail_pick_cap = capa #cur_load
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


# 1. convert_kjh_problem 함수 수정
def convert_kjh_problem(problem_info: dict):
    capa = problem_info["capa"]
    coords = problem_info["node_coords"]
    demands_all = [abs(d) for d in problem_info["node_demands"]]
    types_all = problem_info["node_types"]

    delivery_idx, pickup_idx = [], []
    node_types_internal = [0] * len(coords)  # depot 포함한 전체 크기
    demands_internal = [0] * len(coords)

    # ★ 수정: 인덱스 매핑을 명확히 구분
    for j in range(1, len(coords)):  # 노드 1부터 시작 (depot 제외)
        kjh_type = types_all[j]  # KJH 형식: 0=depot, 1=delivery, 2=pickup

        if kjh_type == 1:  # delivery
            node_types_internal[j] = 1  # internal 형식: 1=delivery
            delivery_idx.append(j)
        elif kjh_type == 2:  # pickup
            node_types_internal[j] = 0  # internal 형식: 0=pickup
            pickup_idx.append(j)
        # kjh_type == 0 (depot)은 이미 초기화에서 0으로 설정됨

        demands_internal[j] = demands_all[j]

    dist_matrix = problem_info["dist_mat"]
    depot_idx = 0

    return (
        delivery_idx,
        pickup_idx,
        demands_internal,
        capa,
        dist_matrix,
        depot_idx,
        node_types_internal,  # 이미 depot 포함한 전체 배열
        coords,
    )


def load_kjh_json(problem_info):
    return convert_kjh_problem(problem_info)


def to_kjh_routes(routes, depot_idx):
    return routes[:]


# 2. to_kjh_types 함수 수정
def to_kjh_types(node_types_internal):
    """internal 형식을 KJH 형식으로 변환"""
    kjh_types = []
    for i, internal_type in enumerate(node_types_internal):
        if i == 0:  # depot
            kjh_types.append(0)
        elif internal_type == 1:  # delivery
            kjh_types.append(1)
        elif internal_type == 0:  # pickup (depot이 아닌 경우)
            kjh_types.append(2)
        else:
            kjh_types.append(2)  # 기본값
    return kjh_types


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


# 3. plot용 데이터 준비 부분 수정 (run_kjh_problem 함수 내)
def prepare_plot_data(coords, node_types_internal, problem_info):
    """플롯용 데이터 준비 - 정확한 매핑 보장"""

    # node_types_internal은 이미 depot 포함한 전체 배열이므로 직접 변환
    kjh_types = to_kjh_types(node_types_internal)

    plot_problem_info = {
        'node_coords': coords,
        'node_types': kjh_types,
        'node_demands': problem_info['node_demands'],
        'capa': problem_info['capa'],
        'K': problem_info['K'],
        'dist_mat': problem_info['dist_mat']
    }

    # 데이터 일관성 검증
    coords_len = len(coords)
    types_len = len(kjh_types)

    print(f"[DEBUG] 매핑 검증:")
    print(f"  - 좌표 배열 크기: {coords_len}")
    print(f"  - KJH 타입 배열 크기: {types_len}")
    print(f"  - 원본 KJH 타입 (처음 10개): {problem_info['node_types'][:10]}")
    print(f"  - 변환된 KJH 타입 (처음 10개): {kjh_types[:10]}")

    # 타입 매핑 검증
    if coords_len == types_len:
        for i in range(min(10, coords_len)):
            orig_type = problem_info['node_types'][i] if i < len(problem_info['node_types']) else 'N/A'
            conv_type = kjh_types[i]
            print(f"  - 노드 {i}: 원본={orig_type} → 변환={conv_type}")

    return plot_problem_info
# ─────────────────────────────────────────────────────────────
def get_solution_stats(routes, dist, demands, capa, node_types):
    """
    해(solution)에 대한 상세 통계 정보를 계산합니다.
    """
    if not routes:
        return {
            'obj': float('inf'), 'num_vehicles': 0, 'mean_dist': 0, 'std_dist': 0,
            'min_dist': 0, 'max_dist': 0, 'mean_load': 0, 'std_load': 0,
            'min_load': 0, 'max_load': 0, 'mean_back_load': 0, 'std_back_load': 0,
            'min_back_load': 0, 'max_back_load': 0
        }

    route_dists = [sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1)) for route in routes]

    # 배송(delivery) 적재율 계산
    delivery_loads = [sum(demands[n] for n in r if node_types[n] == 1) for r in routes]
    delivery_utilization = [load / capa if capa > 0 else 0 for load in delivery_loads]

    # 수거(pickup/backhaul) 적재량 계산
    backhaul_loads = []
    for r in routes:
        in_pick_phase = False
        current_backhaul_load = 0
        for node_idx in range(1, len(r) - 1):
            if node_types[r[node_idx - 1]] == 1 and node_types[r[node_idx]] == 0:  # 배송 -> 수거 전환
                in_pick_phase = True
            if in_pick_phase and node_types[r[node_idx]] == 0:
                current_backhaul_load += demands[r[node_idx]]
        backhaul_loads.append(current_backhaul_load)

    stats = {
        'obj': sum(route_dists),
        'num_vehicles': len(routes),
        'mean_dist': np.mean(route_dists),
        'std_dist': np.std(route_dists),
        'min_dist': np.min(route_dists),
        'max_dist': np.max(route_dists),
        'mean_load': np.mean(delivery_utilization) * 100,  # %단위로
        'std_load': np.std(delivery_utilization) * 100,
        'min_load': np.min(delivery_utilization) * 100,
        'max_load': np.max(delivery_utilization) * 100,
        'mean_back_load': np.mean(backhaul_loads),
        'std_back_load': np.std(backhaul_loads),
        'min_back_load': np.min(backhaul_loads),
        'max_back_load': np.max(backhaul_loads)
    }
    return stats


def log_raw_result(filepath, result_data):
    """
    결과 데이터를 CSV 파일에 한 줄 추가합니다.
    """
    file_exists = os.path.isfile(filepath)

    with open(filepath, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=result_data.keys())
        if not file_exists:
            writer.writeheader()  # 파일이 없으면 헤더를 씁니다.
        writer.writerow(result_data)
# ─────────────────────────────────────────────────────────────
# 7) 메인 드라이버 (완전히 수정된 버전)
# ─────────────────────────────────────────────────────────────

def kny_run(problem_info):
    global cache


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
    ) = load_kjh_json(problem_info)

    # DistanceCache 초기화
    cache = DistanceCache(dist)
    print("[INFO] DistanceCache 초기화 완료")

    total_delivery = sum(demands[i] for i in delivery_idx)
    if math.ceil(total_delivery / capa) > K:
        raise ValueError("Instance infeasible: delivery 총수요가 K·capa 를 초과")

    # 1. 초기 해 생성 시간 측정
    init_start = time.time()
    init_routes = improved_greedy_vrpb(
        delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K
    )
    init_elapsed = time.time() - init_start
    print(f"[⏱️] 초기 해 생성 시간: {init_elapsed:.2f}초")
    print(f"[INFO] Initial route count: {len(init_routes)}")  # 기존 print문 유지
    # ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
    # === (수동) 실험 내용 기록 ===
    # 실행 전에 이 부분에 어떤 실험을 했는지 직접 작성하세요.
    experiment_notes = {
        '수정한 부분': '없음'
    }
    print(f"[🔬] 이번 실행 내용: {experiment_notes['수정한 부분']}")
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
    # 2. ALNS 시간 측정
    alns_start = time.time()
    best_routes, _ = alns_vrpb(
        init_routes, dist, node_types, demands, capa, depot_idx,
        max_vehicles=K, time_limit=57
    )
    alns_elapsed = time.time() - alns_start
    print(f"[⏱️] ALNS 실행 시간: {alns_elapsed:.2f}초")

    # 3. 후처리 시간 측정
    post_proc_start = time.time()
    best_routes = cross_route_2opt_star(best_routes, dist, node_types, demands, capa, depot_idx)
    post_proc_elapsed = time.time() - post_proc_start
    print(f"[⏱️] 후처리(2-opt*) 시간: {post_proc_elapsed:.2f}초")

    # 4. 총 실행 시간 측정
    total_elapsed = init_elapsed + alns_elapsed + post_proc_elapsed

    print(f"[⏱️] 후처리(2-opt*) 시간: {post_proc_elapsed:.2f}초")
    print(f"──────────────────────────────────────────────────────────")
    print(
        f"[✅] 총 실행 시간: {total_elapsed:.2f}초 (초기해: {init_elapsed:.2f}초 + ALNS: {alns_elapsed:.2f}초 + 후처리: {post_proc_elapsed:.2f}초)")
    print(f"──────────────────────────────────────────────────────────")

    return to_kjh_routes(best_routes, depot_idx)
    # obj = check_feasible(problem_info_copy, to_kjh_routes(best_routes, depot_idx), 0, timelimit=60)
    # if obj:
    #     print(f"[✅] check_feasible 통과! obj = {obj:.1f}")
    # else:
    #     print("[❌] check_feasible 실패")
    #     # 💡 추가: 어떤 라우트가 실패 원인인지 fast_feasible_check로 디버깅
    #     print("\n[🛠 check_feasible 디버깅 시작]")
    #     print(f"Internal node_types (0=pickup, 1=delivery): {node_types[:10]}...")
    #     print(f"KJH node_types (0=depot, 1=delivery, 2=pickup): {to_kjh_types(node_types)[:10]}...")
    #     for r_idx, route in enumerate(best_routes):
    #         if not fast_feasible_check(route, node_types, demands, capa, depot_idx):
    #             print(f"[❌] Route {r_idx} violates FFC: {route}")
    #         else:
    #             print(f"[✅] Route {r_idx} passes FFC")
    #     print("[🛠 check_feasible 디버깅 끝]\n")
    #
    # # 캐시 통계 출력
    # print(f"[INFO] 캐시 통계 - Route 캐시: {len(cache.cache)}개, 삽입 캐시: {len(cache.insertion_cache)}개")
    #
    # capa_val = capa  # 편의상 별도 변수
    #
    # visited = set(n for r in best_routes for n in r if n != depot_idx)
    # expected = set(i for i in range(len(node_types)) if i != depot_idx)
    # missed = expected - visited
    # if missed:
    #     print(f"[❌] check_feasible failed - 미방문 노드: {sorted(missed)}")
    # else:
    #     print(f"[✅] 모든 노드 방문 완료")
    #
    # for k, r in enumerate(best_routes):
    #     # -------------- 적재율 계산 ------------------ #
    #     delivery_load = sum(demands[n] for n in r if node_types[n] == 1)
    #     # depot 출발 시 적재량 = 배송 총수요
    #     utilisation = delivery_load / capa_val  # 0.0 ~ 1.0
    #     print(f"vehicle {k:2d}: {r}")
    #     print(f"           ↳ 출발 적재량 = {delivery_load:>6.1f} / {capa_val}  "
    #           f"(utilisation {utilisation:5.1%})")  # ★
    #
    # # 1. 최종 통계 계산
    # final_stats = get_solution_stats(best_routes, dist, demands, capa, node_types)
    #
    # # 2. 로그에 기록할 데이터 구성
    # total_elapsed = init_elapsed + alns_elapsed + post_proc_elapsed  # 전체 시간 계산
    # log_data = {
    #     'instance': problem_path.stem,  # 파일명 (e.g., 'problem_150_0.7')
    #     'obj': final_stats['obj'],
    #     'mean_dist': round(final_stats['mean_dist'], 2),
    #     'std_dist': round(final_stats['std_dist'], 2),
    #     'max_dist': round(final_stats['max_dist'], 2),
    #     'min_dist': round(final_stats['min_dist'], 2),
    #     'std_line_load': round(final_stats['std_load'], 2),
    #     'max_line_load': round(final_stats['max_load'], 2),
    #     'min_line_load': round(final_stats['min_load'], 2),
    #     'std_back_load': round(final_stats['std_back_load'], 2),
    #     'max_back_load': round(final_stats['max_back_load'], 2),
    #     'min_back_load': round(final_stats['min_back_load'], 2),
    #     'num_vehicle': final_stats['num_vehicles'],
    #     'alns_time': round(alns_elapsed, 2),      # ALNS 실행 시간
    #     'total_time': round(total_elapsed, 2),    # 초기해+ALNS+후처리 전체 시간
    #     'method': 1  # 나중에 다른 알고리즘과 비교를 위한 식별자
    #     # 필요하다면 다른 파라미터 (e.g., ALNS 반복 횟수) 등을 추가할 수 있습니다.
    # }
    # log_data.update(experiment_notes)
    #
    # # 3. CSV 파일에 기록
    # if ENABLE_LOGGING:
    #     log_raw_result('raw_results.csv', log_data)
    #     print(f"\n[📝] 'raw_results.csv' 파일에 결과가 기록되었습니다.")
    # else:
    #     print(f"\n[🟡] 로깅 비활성화됨: 'raw_results.csv'에 저장하지 않습니다.")
    #
    # # 기존 수동 매핑 코드를 다음으로 교체
    # plot_problem_info = prepare_plot_data(coords, node_types, problem_info)
    # plot_vrpb(plot_problem_info, best_routes, f"VRPB obj: {best_cost:.0f}")


if __name__ == "__main__":
    ROOT = Path(__file__).resolve().parents[1]

    # 여러 번 실행 예시 (예: 5회)
    instance_path = ROOT / "instances" / "problem_130_0.5.json"
    num_runs = 2
    print(f"'{instance_path.name}' 인스턴스를 {num_runs}회 실행합니다.")

    for i in range(num_runs):
        print(f"\n{'=' * 20} 실행 {i + 1}/{num_runs} {'=' * 20}")
        kny_run(instance_path)