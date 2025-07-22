# ======================================================================
#  KNY_VRPB.py  ―  기존 코드 + KJH JSON 어댑터 + 결과 포맷 역변환
# ======================================================================

import json
import random
import math
import time
from pathlib import Path

from util import bin_packing, get_distance, plot_cvrp, check_feasible
from KNY_constraint import is_solution_feasible # 제약 검증 함수
from KNY_alns import alns_vrpb

# ─────────────────────────────────────────────────────────────
# 0) KJH JSON → 내부(KNY) 포맷 변환
# ─────────────────────────────────────────────────────────────
def convert_kjh_problem(problem_info: dict):
    """
    KJH JSON(dict) → KNY 내부 포맷
    반환:
      delivery_idx, pickup_idx, demands, capa,
      dist_matrix, depot_idx, node_types_internal, all_coords
    """
    capa = problem_info["capa"]

    coords_all   = problem_info["node_coords"]   # depot + 고객
    demands_all  = problem_info["node_demands"]
    types_all    = problem_info["node_types"]    # 0=depot,1,2

    depot_coord  = coords_all[0]
    cust_coords  = coords_all[1:]
    cust_demands = demands_all[1:]
    cust_types   = [1 if t == 1 else 0 for t in types_all[1:]]  # 1=linehaul,0=backhaul

    delivery_idx = [i for i, t in enumerate(cust_types) if t == 1]
    pickup_idx   = [i for i, t in enumerate(cust_types) if t == 0]

    all_coords   = cust_coords + [depot_coord]   # depot을 마지막으로 이동
    dist_matrix  = get_distance(all_coords)
    depot_idx    = len(cust_coords)              # depot = 마지막 인덱스

    return (delivery_idx, pickup_idx, cust_demands, capa,
            dist_matrix, depot_idx, cust_types, all_coords)


def load_kjh_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        info = json.load(f)
    return convert_kjh_problem(info)

# ─────────────────────────────────────────────────────────────
# 0‑B) 내부 → KJH 포맷 변환 (출력용)
# ─────────────────────────────────────────────────────────────
def to_kjh_routes(routes, depot_idx):
    """depot=N → depot=0, 고객 인덱스 +1"""
    return [[0 if v == depot_idx else v + 1 for v in r] for r in routes]

def to_kjh_types(node_types_internal):
    """내부 1/0 → KJH 1/2, 맨 앞에 0(depot) 추가"""
    return [0] + [1 if t == 1 else 2 for t in node_types_internal]


# ─────────────────────────────────────────────────────────────
# 1) Greedy Insertion
# ─────────────────────────────────────────────────────────────
def greedy_insertion_vrpb(delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types):
    print("[INFO] Start greedy_insertion_vrpb()")
    unassigned_deliv = set(delivery_idx)
    unassigned_pick  = set(pickup_idx)
    routes = []

    # ① 배송 노드들을 차량별로 할당
    while unassigned_deliv:
        print(f"[INFO] New vehicle for delivery. Remaining deliveries: {len(unassigned_deliv)}")
        route = [depot_idx]           # 경로 시작 (depot)
        cur = depot_idx
        load = capa                  # 차량을 용량만큼 적재한 상태로 시작
        # 배송 고객 추가 루프
        while True:
            # 남은 용량(load) 내에서 추가 가능한 배송지 중 가장 가까운 노드 선택
            candidates = [n for n in unassigned_deliv if demands[n] <= load]
            if not candidates:
                break  # 남은 용량으로 추가 가능 한 배송 없음 -> 배송 단계 종료
            # 가장 가까운 후보 선택
            nxt = min(candidates, key=lambda n: dist[cur][n])
            route.append(nxt)
            load -= demands[nxt]     # 배송 수행 → 적재량 감소
            cur = nxt
            unassigned_deliv.remove(nxt)
        # ② 동일 차량으로 회수 노드 처리
        while True:
            candidates = [n for n in unassigned_pick if demands[n] <= (capa - load)]
            if not candidates:
                break  # 남은 공간으로 실을 수 있는 회수 없음 -> 회수 단계 종료
            nxt = min(candidates, key=lambda n: dist[cur][n])
            route.append(nxt)
            load += demands[nxt]     # 회수 수행 → 적재량 증가
            cur = nxt
            unassigned_pick.remove(nxt)
        # 경로 완료 (Depot 귀환)
        route.append(depot_idx)
        routes.append(route)

    # ③ 남은 회수 노드들을 별도 차량으로 처리
    while unassigned_pick:
        print(f"[INFO] New vehicle for remaining pickups. Remaining pickups: {len(unassigned_pick)}")
        route = [depot_idx]
        cur = depot_idx
        load = 0                    # 회수만 하는 경우 초기 적재 0 (빈 트럭 출발)
        while True:
            candidates = [n for n in unassigned_pick if demands[n] <= (capa - load)]
            if not candidates:
                break  # 남은 용량으로 실을 수 있는 회수 없음
            nxt = min(candidates, key=lambda n: dist[cur][n])
            route.append(nxt)
            load += demands[nxt]    # 회수 적재
            cur = nxt
            unassigned_pick.remove(nxt)
        route.append(depot_idx)
        routes.append(route)
    print("[INFO] Greedy insertion finished.")
    return routes

# ─────────────────────────────────────────────────────────────
# 2) Utility: cost of one route  ―  (기존 그대로)
# ─────────────────────────────────────────────────────────────
def route_cost(route: list[int], dist: list[list[float]]) -> float:
    return sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))

# ─────────────────────────────────────────────────────────────
# cross route 2-opt 추가
# ─────────────────────────────────────────────────────────────
def cross_route_2opt_star(routes, dist, node_types, demands, capa, depot_idx):
    changed = True
    while changed:
        changed = False
        for i in range(len(routes)):
            for j in range(i + 1, len(routes)):
                r1, r2 = routes[i], routes[j]
                for idx1 in range(1, len(r1) - 1):
                    for idx2 in range(1, len(r2) - 1):
                        if node_types[r1[idx1]] != node_types[r2[idx2]]:
                            continue  # linehaul과 backhaul 섞지 않음

                        # 교환 시 경로 복사
                        new_r1 = r1[:idx1] + [r2[idx2]] + r1[idx1 + 1:]
                        new_r2 = r2[:idx2] + [r1[idx1]] + r2[idx2 + 1:]

                        # 제약 검증
                        if not is_solution_feasible([new_r1, new_r2], node_types, demands, capa, depot_idx):
                            continue

                        # 비용 감소 확인
                        old_cost = route_cost(r1, dist) + route_cost(r2, dist)
                        new_cost = route_cost(new_r1, dist) + route_cost(new_r2, dist)
                        if new_cost < old_cost:
                            routes[i], routes[j] = new_r1, new_r2
                            changed = True
    return routes

# ─────────────────────────────────────────────────────────────
# 5) JSON(KJH) 문제를 바로 돌리는 헬퍼
# ─────────────────────────────────────────────────────────────
def run_kjh_problem(problem_path):
    with open(problem_path, "r", encoding="utf-8") as f:
        problem_info = json.load(f)
    K = problem_info["K"]

    (delivery_idx, pickup_idx, demands, capa,
     dist, depot_idx, node_types, all_coords) = load_kjh_json(problem_path)

    init_routes = greedy_insertion_vrpb(
        delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types
    )
    # 🔽 여기 바로 아래에 추가!
    print(f"[INFO] Initial route count: {len(init_routes)} vehicles (max {K})")

    # ① 배송 없는 경로 병합
    pickup_only_routes = [r for r in init_routes if all(node_types[v] == 0 for v in r[1:-1])]
    for r in pickup_only_routes:
        init_routes.remove(r)
        for n in r[1:-1]:
            placed = False
            for r2 in init_routes:
                load_delivery = sum(demands[v] for v in r2 if v != depot_idx and node_types[v] == 1)
                load_pickup = sum(demands[v] for v in r2 if v != depot_idx and node_types[v] == 0)
                remaining_cap = capa - load_delivery
                if load_pickup + demands[n] <= remaining_cap:
                    if any(node_types[v] == 1 for v in r2[1:-1]):
                        last_delivery_index = max(i for i, v in enumerate(r2) if node_types[v] == 1)
                        insert_pos = last_delivery_index + 1
                    else:
                        insert_pos = 1
                    r2.insert(insert_pos, n)
                    placed = True
                    break
            if not placed:
                init_routes.append([depot_idx, n, depot_idx])

    # (선택) 병합 이후 경로 수 확인
    print(f"[INFO] Route count after merging: {len(init_routes)}")

    # (3) ALNS 최적화
    start = time.time()
    best_routes, best_cost = alns_vrpb(
        init_routes, dist, node_types, demands, capa, depot_idx, max_vehicles=K, time_limit=60
    )
    elapsed = time.time() - start

    # (4) Cross-Route 2-opt* 개선
    best_routes = cross_route_2opt_star(best_routes, dist, node_types, demands, capa, depot_idx)
    best_cost = sum(route_cost(r, dist) for r in best_routes)

    # 내부 → KJH 포맷 변환
    routes_kjh     = to_kjh_routes(best_routes, depot_idx)
    node_types_kjh = to_kjh_types(node_types)

    # (6) check_feasible 호출을 위해 JSON 다시 불러오기

    print("[INFO] Final feasibility check with check_feasible()...")
    obj = check_feasible(problem_info, routes_kjh, elapsed, timelimit=60)
    if obj:
        print(f"[✅] check_feasible 통과! 총 비용: {obj:.1f}")
    else:
        print("[❌] check_feasible 기준에서 유효하지 않은 해입니다.")

    # 출력 & (필요 시) KJH 쪽 검증
    for k, r in enumerate(routes_kjh):
        print(f"vehicle {k}: {r}")

    # 예시: check_feasible(problem_info, routes_kjh, …) 호출하려면
    # JSON을 다시 읽어서 problem_info 만들고 넘기면 됩니다.

    # 그림은 내부 인덱스로 그리는 편이 편해요
    plot_cvrp(all_coords, best_routes, f"VRPB obj: {best_cost:.1f}")

# ─────────────────────────────────────────────────────────────
# 6) 실행 엔트리
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # 프로젝트 루트(25SummerStudy)
    ROOT = Path(__file__).resolve().parents[1]
    PROBLEM_JSON = ROOT / "instances" / "problem_220_0.7.json"

    run_kjh_problem(PROBLEM_JSON)