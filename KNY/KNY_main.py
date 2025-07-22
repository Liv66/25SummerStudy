# ======================================================================
#  KNY_VRPB.py  ―  기존 코드 + KJH JSON 어댑터 + 결과 포맷 역변환
# ======================================================================

import json
import random
import math
import time
from pathlib import Path

from util import bin_packing, get_distance, plot_cvrp
from KNY_constraint import is_solution_feasible  # 제약 검증 함수

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
# 1) Constraint-Aware Greedy Insertion (VRPB)  ―  (기존 그대로)
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
# 3) Adaptive Large Neighborhood Search (VRPB)  ―  (기존 그대로)
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
    print("[INFO] Start ALNS")
    best_routes = [r[:] for r in init_routes]
    best_cost   = sum(route_cost(r, dist) for r in best_routes)
    cur_routes  = [r[:] for r in best_routes]

    T0, alpha = 1_000.0, 0.995
    iteration = 0
    start = time.time()

    while time.time() - start < time_limit:
        iteration += 1
        if iteration % 10 == 0:
            print(f"[INFO] Iteration {iteration}, best cost so far: {best_cost:.2f}")

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

        for n in removed:
            best_delta, best_pos, best_r = math.inf, None, None
            for r in new_routes:
                if node_types[n] == 1 and any(node_types[v] == 0 for v in r[1:-1]):
                    continue
                load = sum(-demands[v] if node_types[v] == 1 else demands[v] for v in r if v != depot_idx)
                change = -demands[n] if node_types[n] == 1 else demands[n]
                if load + change > capa or load + change < 0:
                    continue
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

    print("[INFO] ALNS finished.")
    return best_routes, best_cost


# ─────────────────────────────────────────────────────────────
# 4) 기존 랜덤 인스턴스용 main()  ―  그대로 유지
# ─────────────────────────────────────────────────────────────
def main() -> None:
    print("[INFO] Start main()")
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

    print("[INFO] Calling greedy_insertion_vrpb()")
    init_routes = greedy_insertion_vrpb(
        delivery_idx, pickup_idx, demands, capa, dist_matrix, depot_idx, node_types
    )
    print("[INFO] Initial solution constructed.")

    assert is_solution_feasible(init_routes, node_types, demands, capa, depot_idx), \
        "Initial solution infeasible!"

    print("[INFO] Calling alns_vrpb()")
    best_routes, best_cost = alns_vrpb(
        init_routes, dist_matrix, node_types, demands, capa,
        depot_idx=depot_idx, time_limit=60,
    )
    print("[INFO] ALNS optimization completed.")

    assert is_solution_feasible(best_routes, node_types, demands, capa, depot_idx), \
        "Final best solution infeasible!"

    for idx, route in enumerate(best_routes):
        print(f"vehicle {idx} route: {route}")
    plot_cvrp(all_coord, best_routes, f"VRPB obj: {best_cost:.1f}")


# ─────────────────────────────────────────────────────────────
# 5) JSON(KJH) 문제를 바로 돌리는 헬퍼
# ─────────────────────────────────────────────────────────────
def run_kjh_problem(problem_path):
    (delivery_idx, pickup_idx, demands, capa,
     dist, depot_idx, node_types, all_coords) = load_kjh_json(problem_path)

    init_routes = greedy_insertion_vrpb(
        delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types
    )
    best_routes, best_cost = alns_vrpb(
        init_routes, dist, node_types, demands, capa, depot_idx, time_limit=60
    )

    # 내부 → KJH 포맷 변환
    routes_kjh     = to_kjh_routes(best_routes, depot_idx)
    node_types_kjh = to_kjh_types(node_types)

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