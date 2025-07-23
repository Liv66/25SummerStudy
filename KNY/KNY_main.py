import json
import random
import math
import time
from pathlib import Path

from util import get_distance, plot_cvrp, check_feasible
from KNY_constraint import is_solution_feasible
from KNY_alns import alns_vrpb

def bin_packing_greedy(items, capa, max_bins=None):
    """
    items: List of (node_idx, demand)
    capa: 최대 용량
    max_bins: None 또는 최대 bin(차량) 개수
    반환: [[node_idx, ...], [node_idx, ...], ...]  # 각 bin(차량)별 node index 리스트
    """
    bins = []
    bin_loads = []
    for node_idx, demand in sorted(items, key=lambda x: -x[1]):
        placed = False
        for i, load in enumerate(bin_loads):
            if load + demand <= capa:
                bins[i].append(node_idx)
                bin_loads[i] += demand
                placed = True
                break
        if not placed:
            if max_bins and len(bins) >= max_bins:
                raise ValueError(f"bin_packing: 더 이상 bin 추가 불가. demand = {demand} 남음")
            bins.append([node_idx])
            bin_loads.append(demand)
    return bins

def convert_kjh_problem(problem_info: dict):
    capa = problem_info["capa"]
    coords_all = problem_info["node_coords"]
    demands_all = [abs(d) for d in problem_info["node_demands"]]
    types_all = problem_info["node_types"]

    depot_coord = coords_all[0]
    cust_coords = coords_all[1:]
    cust_demands = demands_all[1:]
    cust_types = [1 if t == 1 else 0 for t in types_all[1:]]

    delivery_idx = [i for i, t in enumerate(cust_types) if t == 1]
    pickup_idx = [i for i, t in enumerate(cust_types) if t == 0]
    all_coords = cust_coords + [depot_coord]
    dist_matrix = get_distance(all_coords)
    depot_idx = len(cust_coords)

    return delivery_idx, pickup_idx, cust_demands, capa, dist_matrix, depot_idx, cust_types, all_coords

def load_kjh_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        info = json.load(f)
    return convert_kjh_problem(info)

def to_kjh_routes(routes, depot_idx):
    return [[0 if v == depot_idx else v + 1 for v in r] for r in routes]

def to_kjh_types(node_types_internal):
    return [0] + [1 if t == 1 else 2 for t in node_types_internal]


def redistribute_and_force_insert(pick, routes, demands, node_types, capa, depot):
    """노드 재분배로 공간 확보 후 pick 삽입. 실패 시 False."""
    # 가장 가벼운 배송노드를 찾아 다른 루트로 이동
    src = min((r for r in routes if any(node_types[v] == 1 for v in r[1:-1] if v != depot)),
              key=lambda r: sum(demands[v] for v in r[1:-1]), default=None)
    if src is None: return False

    cand = min(
        (v for v in src[1:-1] if node_types[v] == 1),
        key=lambda v: demands[v],
        default=None
    )
    if cand is None: return False

    # cand를 수용할 다른 루트 탐색
    for dst in routes:
        if dst is src: continue
        if any(node_types[v] == 1 for v in dst[1:-1] if v != depot):
            if sum(demands[v] for v in dst[1:-1]) + demands[cand] <= capa:
                # 이동 실행
                src.remove(cand)
                pos = max((i for i, v in enumerate(dst) if v != depot and node_types[v] == 1), default=0) + 1
                dst.insert(pos, cand)

                # src에 pick 삽입 시도
                pos = max((i for i, v in enumerate(src) if v != depot and node_types[v] == 1), default=0) + 1
                if sum(demands[v] for v in src[1:-1]) + demands[pick] <= capa:
                    src.insert(pos, pick)
                    return True

                # 복구
                dst.remove(cand)
                src.insert(pos, cand)
    return False


def force_insert_pickup(unassigned, routes, demands, node_types, capa, dist, depot):
    """pickup 노드를 반드시 기존 배송 루트에 삽입(재분배 필요 시 수행)"""
    for n in list(unassigned):
        # 가장 여유 큰 배송루트에 직접 삽입 시도
        best_r, best_pos, best_inc = None, None, float('inf')
        for r in routes:
            if not any(node_types[v] == 1 for v in r[1:-1] if v != depot): continue
            last = max((i for i, v in enumerate(r) if v != depot and node_types[v] == 1), default=0)
            for pos in range(last + 1, len(r)):
                tmp = r[:pos] + [n] + r[pos:]
                if is_solution_feasible([tmp], node_types, demands, capa, depot):
                    inc = dist[r[pos - 1]][n] + dist[n][r[pos]] - dist[r[pos - 1]][r[pos]]
                    if inc < best_inc:
                        best_r, best_pos, best_inc = r, pos, inc

        if best_r:
            best_r.insert(best_pos, n)
            unassigned.remove(n)
        else:
            # 재분배 시도
            if redistribute_and_force_insert(n, routes, demands, node_types, capa, depot):
                unassigned.remove(n)
            else:
                # 🔽🔽 이 두 줄 추가
                print(f"[DEBUG] Node {n} demand: {demands[n]}")
                print(f"[DEBUG] Unassigned node {n} 삽입 실패. 현재 라우트 수: {len(routes)}")
                raise ValueError(f"[❌] 회수노드 {n}은 어떤 차량에도 배정 불가")


def greedy_insertion_vrpb_binpacking(delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K):
    print("[INFO] Bin-packing Greedy 방식 Route 초기화 시작")
    # 1. 배송 노드를 demand순 K-bin으로 packing (Greedy)
    delivery_items = [(n, demands[n]) for n in delivery_idx]
    bins = bin_packing_greedy(delivery_items, capa, max_bins=K)
    if len(bins) > K:
        raise ValueError(f"[❌] bin_packing 결과 K({K})대 내로 배송불가 (배송만 {len(bins)}대 필요)")

    # 2. 배송 노드 bin별로 루트 생성 (depot→배송들→depot)
    routes = []
    for bin_nodes in bins:
        route = [depot_idx] + bin_nodes + [depot_idx]
        routes.append(route)

    # 3. 회수 노드를 기존 route에 feasible하게 삽입
    unassigned_pick = set(pickup_idx)
    for n in list(unassigned_pick):
        best_route = None
        best_pos = None
        best_incr = float('inf')
        for r in routes:
            # 배송 마지막 인덱스 찾기
            last_deliv = max(
                (i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1),
                default=0
            )
            for pos in range(last_deliv + 1, len(r)):
                tmp_r = r[:pos] + [n] + r[pos:]
                if is_solution_feasible([tmp_r], node_types, demands, capa, depot_idx):
                    # 증분 비용 계산
                    dist_incr = dist[r[pos-1]][n] + dist[n][r[pos]] - dist[r[pos-1]][r[pos]]
                    if dist_incr < best_incr:
                        best_incr = dist_incr
                        best_route = r
                        best_pos = pos
        if best_route is not None:
            best_route.insert(best_pos, n)
            unassigned_pick.remove(n)

    # 남은 회수노드 강제 삽입 (재분배 포함)
    if unassigned_pick:
        print(f"[INFO] {len(unassigned_pick)}개 회수노드 강제 삽입 시도")
        force_insert_pickup(unassigned_pick, routes, demands, node_types, capa, dist, depot_idx)

    print(f"[INFO] Bin-packing Greedy 초기화 완료. Route 수: {len(routes)} (제한: {K})")
    return routes

def merge_routes_if_possible(routes, demands, node_types, capa, depot_idx, dist, K):
    def is_feasible(route):
        load = 0
        seen_pick = False
        for i in range(1, len(route) - 1):
            n = route[i]
            if node_types[n] == 1:
                load += demands[n]
                if seen_pick:
                    return False
            else:
                load -= demands[n]
                seen_pick = True
            if load > capa or load < 0:
                return False
        return route[0] == depot_idx and route[-1] == depot_idx

    merged = True
    while merged and len(routes) > K:
        merged = False
        for i in range(len(routes)):
            for j in range(i + 1, len(routes)):
                r1, r2 = routes[i], routes[j]
                new_route = r1[:-1] + r2[1:]
                if is_feasible(new_route):
                    routes[i] = new_route
                    del routes[j]
                    merged = True
                    break
            if merged:
                break
    return routes

def route_cost(route: list[int], dist: list[list[float]]) -> float:
    return sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))

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

                        new_r1 = r1[:idx1] + [r2[idx2]] + r1[idx1 + 1:]
                        new_r2 = r2[:idx2] + [r1[idx1]] + r2[idx2 + 1:]

                        if not is_solution_feasible([new_r1, new_r2], node_types, demands, capa, depot_idx):
                            continue

                        old_cost = route_cost(r1, dist) + route_cost(r2, dist)
                        new_cost = route_cost(new_r1, dist) + route_cost(new_r2, dist)
                        if new_cost < old_cost:
                            routes[i], routes[j] = new_r1, new_r2
                            changed = True
    return routes

def run_kjh_problem(problem_path):
    with open(problem_path, "r", encoding="utf-8") as f:
        problem_info = json.load(f)
    K = problem_info["K"]

    (delivery_idx, pickup_idx, demands, capa,
     dist, depot_idx, node_types, all_coords) = load_kjh_json(problem_path)

    total_delivery = sum(d for i, d in enumerate(demands) if node_types[i] == 1)
    min_possible = math.ceil(total_delivery / capa)
    if min_possible > K:
        raise ValueError(f"Instance infeasible: needs at least {min_possible} "
                         f"vehicles (K={K}).")

    init_routes = greedy_insertion_vrpb_binpacking(
        delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K
    )
    print(f"[INFO] Initial route count: {len(init_routes)} vehicles (max {K})")

    init_routes = merge_routes_if_possible(
        init_routes, demands, node_types, capa, depot_idx, dist, K
    )
    print(f"[INFO] Route count after merging (feasible merge): {len(init_routes)}")

    pickup_only_routes = [r for r in init_routes if all(node_types[v] == 0 for v in r[1:-1])]
    for r in pickup_only_routes:
        init_routes.remove(r)
        for n in r[1:-1]:
            best_cost = float("inf")
            best_r = None
            best_pos = None

            for r2 in init_routes:
                delivered = sum(demands[v] for v in r2[1:-1] if node_types[v] == 1)
                picked = sum(demands[v] for v in r2[1:-1] if node_types[v] == 0)

                if delivered > 0:
                    avail_pick = delivered - picked
                else:
                    avail_pick = capa - picked

                if demands[n] > avail_pick:
                    continue

                insert_pos = max(i for i, v in enumerate(r2)
                                 if v == depot_idx or node_types[v] == 1) + 1

                prev, nxt = r2[insert_pos - 1], r2[insert_pos]
                added = dist[prev][n] + dist[n][nxt] - dist[prev][nxt]

                if added < best_cost:
                    best_cost = added
                    best_r = r2
                    best_pos = insert_pos

            if best_r is not None:
                best_r.insert(best_pos, n)
            else:
                init_routes.append([depot_idx, n, depot_idx])

    print(f"[INFO] Route count after merging: {len(init_routes)}")

    start = time.time()
    best_routes, best_cost = alns_vrpb(
        init_routes, dist, node_types, demands, capa, depot_idx, max_vehicles=K, time_limit=60
    )
    elapsed = time.time() - start

    best_routes = cross_route_2opt_star(best_routes, dist, node_types, demands, capa, depot_idx)
    best_cost = sum(route_cost(r, dist) for r in best_routes)

    routes_kjh     = to_kjh_routes(best_routes, depot_idx)
    node_types_kjh = to_kjh_types(node_types)

    print("[INFO] Final feasibility check with check_feasible()...")
    obj = check_feasible(problem_info, routes_kjh, elapsed, timelimit=60)
    if obj:
        print(f"[✅] check_feasible 통과! 총 비용: {obj:.1f}")
    else:
        print("[❌] check_feasible 기준에서 유효하지 않은 해입니다.")

    for k, r in enumerate(routes_kjh):
        print(f"vehicle {k}: {r}")

    plot_cvrp(all_coords, best_routes, f"VRPB obj: {best_cost:.1f}")

if __name__ == "__main__":
    ROOT = Path(__file__).resolve().parents[1]
    PROBLEM_JSON = ROOT / "instances" / "problem_220_0.7.json"

    run_kjh_problem(PROBLEM_JSON)
