import json
import random
import math
import time
from pathlib import Path

from util import get_distance, plot_cvrp, check_feasible
#from KNY_constraint import is_solution_feasible
from KNY_alns import alns_vrpb
random.seed(42)



def check_feasible_internal(routes, node_types, demands, capa, depot_idx, problem_info):
    """
    내부 라우트들을 KJH-style 로 변환한 뒤 원본 check_feasible()로 검증.
    ── 원본 규칙과 100 % 동일하게:
       • 배송→회수 이후 ‘배송’ 금지
       • line↔back 전환 시 load = 0 재설정
       • load < 0 또한 즉시 불가
       • 첫 고객이 backhaul(0)인 루트 금지
    """
    for route in routes:
        if len(route) < 3:                 # depot-depot
            continue

        if node_types[route[1]] == 0:      # back-only 출발 금지
            return False

        load, flag = 0, False              # flag=False ⇒ line 구간
        for node in route[1:-1]:
            t = node_types[node]

            # ▶︎ linehaul
            if t == 1:
                if flag:                   # back → line 재진입 금지
                    return False
                load += demands[node]
            # ▶︎ backhaul
            else:
                if not flag:               # 첫 backhaul 진입 시
                    flag, load = True, 0
                load += demands[node]

            # 공통 용량 검사
            if load > capa or load < 0:
                return False

    # ── 외부 check_feasible 호출(원본 비용 계산용)
    routes_kjh     = [[0 if v == depot_idx else v + 1 for v in r] for r in routes]
    node_types_kjh = [0] + [1 if t == 1 else 2 for t in node_types]

    tmp_problem = {
        "K": max(len(routes) + 5, problem_info.get("K", 50)),
        "node_types":   node_types_kjh,
        "node_demands": [0] + demands,
        "capa":         capa,
        "dist_mat":     problem_info.get("dist_mat",
                         [[0]*len(node_types_kjh)]*len(node_types_kjh)),
    }

    try:
        return check_feasible(tmp_problem, routes_kjh, 0, 999) > 0
    except Exception:
        return False        # 변환 오류 시 infeasible

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
    capa        = problem_info["capa"]
    coords_all  = problem_info["node_coords"]      # [depot] + customers
    demands_all = [abs(d) for d in problem_info["node_demands"]]
    types_all   = problem_info["node_types"]       # 0=depot, 1=linehaul, 2=backhaul

    # ── depot은 0, 고객은 1…N  인덱스로 통일 ─────────────────────────
    delivery_idx, pickup_idx = [], []
    node_types_internal      = [0] * len(coords_all)   # 0번째는 depot
    demands_internal         = [0] * len(coords_all)   # 〃

    for j in range(1, len(coords_all)):                # 고객만 순회
        cust_type = 1 if types_all[j] == 1 else 0      # 1=linehaul, 0=backhaul
        node_types_internal[j] = cust_type
        demands_internal[j]    = demands_all[j]

        if cust_type == 1:
            delivery_idx.append(j)   # 고객 인덱스 그대로 (1…)
        else:
            pickup_idx.append(j)

    dist_matrix = problem_info["dist_mat"]             # JSON 거리행렬 그대로
    depot_idx   = 0

    return (delivery_idx, pickup_idx, demands_internal,
            capa, dist_matrix, depot_idx,
            node_types_internal, coords_all)



def load_kjh_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        info = json.load(f)
    return convert_kjh_problem(info)


def to_kjh_routes(routes, depot_idx):
    return routes[:]


def to_kjh_types(node_types_internal):
    return [0] + [1 if t == 1 else 2 for t in node_types_internal]


def redistribute_and_force_insert(pick, routes, demands, node_types, capa, depot, problem_info):
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


def force_insert_pickup(unassigned, routes, demands, node_types, capa, dist, depot, problem_info):
    """pickup 노드를 반드시 기존 배송 루트에 삽입(재분배 필요 시 수행)"""
    for n in list(unassigned):
        print(f"[DEBUG] 회수노드 {n} (demand={demands[n]}) 삽입 시도...")

        # 각 루트의 현재 상태 확인
        for i, r in enumerate(routes):
            delivery_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
            pickup_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 0)
            available_pickup = delivery_load - pickup_load
            print(f"  Route {i}: 배송={delivery_load}, 회수={pickup_load}, 회수가능={available_pickup}")

        # 1단계: 기존 루트에 직접 삽입 시도 (완화된 조건)
        best_r, best_pos, best_inc = None, None, float('inf')
        for r in routes:
            if not any(node_types[v] == 1 for v in r[1:-1] if v != depot):
                continue

            # 완화된 용량 체크: 배송량이 있으면 회수 가능
            delivery_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
            pickup_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 0)

            # 조건 완화: 배송량이 있고, 새 회수량을 더해도 용량 초과하지 않으면 OK
            if delivery_load > 0 and (pickup_load + demands[n] <= delivery_load):
                # 배송 노드들 뒤에 삽입 위치 찾기
                last_delivery_idx = max((i for i, v in enumerate(r) if v != depot and node_types[v] == 1), default=0)

                for pos in range(last_delivery_idx + 1, len(r)):
                    # ① 삽입한 경로 한 번 생성
                    # ────────────────────────────────
                    temp_route = r[:pos] + [n] + r[pos:]

                    # 간단한 feasibility 체크 (완화됨)
                    is_valid, tmp_load, delivery_phase = True, 0, True
                    for node in temp_route[1:-1]:
                        if node_types[node] == 1:  # 배송
                            if not delivery_phase:
                                is_valid = False;
                                break
                            tmp_load += demands[node]
                        else:  # 회수
                            if delivery_phase:  # 첫 back 진입
                                delivery_phase, tmp_load = False, 0
                            tmp_load += demands[node]

                        if tmp_load > capa or tmp_load < 0:  # 두 방향 모두 금지
                            is_valid = False;
                            break

                    # 기본 VRPB 제약만 체크 (더 관대하게)
                    is_valid = True
                    temp_load = 0
                    delivery_phase = True

                    for idx in range(1, len(temp_route) - 1):
                        node = temp_route[idx]
                        if node_types[node] == 1:  # 배송
                            if not delivery_phase:  # 회수 후 배송은 불가
                                is_valid = False
                                break
                            temp_load += demands[node]
                        else:  # 회수
                            delivery_phase = False
                            temp_load -= demands[node]

                        # 용량 체크 (음수는 허용, 단지 capa 초과만 체크)
                        if temp_load > capa:
                            is_valid = False
                            break

                    if is_valid:
                        inc = dist[r[pos - 1]][n] + dist[n][r[pos]] - dist[r[pos - 1]][r[pos]]
                        if inc < best_inc:
                            best_r, best_pos, best_inc = r, pos, inc

        if best_r:
            print(f"[DEBUG] Node {n} 성공적으로 삽입됨")
            best_r.insert(best_pos, n)
            unassigned.remove(n)
            continue

        # 2단계: 재분배 시도 (개선된 로직)
        print(f"[DEBUG] Node {n} 재분배 시도...")
        inserted_via_redistribution = False

        # 가장 여유있는 루트 찾기
        target_routes = []
        for r in routes:
            delivery_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
            pickup_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 0)
            if delivery_load > pickup_load:  # 회수 여유가 있는 루트
                available_space = delivery_load - pickup_load
                target_routes.append((r, available_space))

        # 여유 공간 순으로 정렬
        target_routes.sort(key=lambda x: x[1], reverse=True)

        for target_route, available_space in target_routes:
            if available_space >= demands[n]:
                # 배송 노드들 뒤에 삽입
                last_delivery_idx = max((i for i, v in enumerate(target_route) if v != depot and node_types[v] == 1),
                                        default=0)
                insert_pos = last_delivery_idx + 1

                target_route.insert(insert_pos, n)
                unassigned.remove(n)
                inserted_via_redistribution = True
                print(f"[DEBUG] Node {n} 재분배로 삽입 성공")
                break

        if inserted_via_redistribution:
            continue

        # 3단계: 새 루트 생성 (단독 회수 루트)
        print(f"[DEBUG] Node {n}을 위한 새 루트 생성 시도...")

        # 단독 회수 노드도 허용 (특별한 경우)
        #if demands[n] <= capa:
            #new_route = [depot, n, depot]
            #routes.append(new_route)
            #unassigned.remove(n)
            #print(f"[DEBUG] Node {n}을 위한 새 루트 생성 성공 (단독 회수)")
            #continue

        # 4단계: 다른 루트에서 배송 노드를 가져와서 새 루트 생성
        print(f"[DEBUG] Node {n}을 위한 배송+회수 조합 루트 생성 시도...")

        # 가장 작은 배송 노드를 찾아서 함께 새 루트 생성
        min_delivery_node = None
        min_delivery_demand = float('inf')
        source_route = None

        for r in routes:
            for i, node in enumerate(r[1:-1], 1):
                if node_types[node] == 1 and demands[node] < min_delivery_demand:
                    # 이 배송 노드와 회수 노드가 함께 갈 수 있는지 체크
                    if demands[node] + demands[n] <= capa and demands[node] >= demands[n]:
                        min_delivery_node = node
                        min_delivery_demand = demands[node]
                        source_route = r

        if min_delivery_node is not None:
            # 배송 노드를 원래 루트에서 제거
            source_route.remove(min_delivery_node)

            # 새 루트 생성 (배송 -> 회수 순서)
            new_route = [depot, min_delivery_node, n, depot]
            routes.append(new_route)
            unassigned.remove(n)
            print(f"[DEBUG] Node {n}을 배송노드 {min_delivery_node}와 함께 새 루트 생성 성공")
            continue

        # 최종 실패
        print(f"[ERROR] Node {n} 상세 정보:")
        print(f"  - Demand: {demands[n]}")
        print(f"  - Capacity: {capa}")
        print(f"  - 현재 루트 수: {len(routes)}")
        print(
            f"  - 각 루트별 배송량: {[sum(demands[node] for node in route[1:-1] if node_types[node] == 1) for route in routes]}")
        print(
            f"  - 각 루트별 회수량: {[sum(demands[node] for node in route[1:-1] if node_types[node] == 0) for route in routes]}")

        # 마지막 수단: 강제로 가장 큰 여유가 있는 루트에 삽입
        print(f"[DEBUG] 마지막 수단: 강제 삽입 시도...")

        best_route_for_force = None
        max_available = -1

        for r in routes:
            delivery_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 1)
            pickup_load = sum(demands[v] for v in r[1:-1] if node_types[v] == 0)

            if delivery_load > 0:  # 배송이 있는 루트만
                available = delivery_load - pickup_load
                if available > max_available:
                    max_available = available
                    best_route_for_force = r

        if best_route_for_force is not None and max_available > 0:
            # 강제 삽입 (제약 조건 무시)
            last_delivery_idx = max(
                (i for i, v in enumerate(best_route_for_force) if v != depot and node_types[v] == 1), default=0)
            best_route_for_force.insert(last_delivery_idx + 1, n)
            unassigned.remove(n)
            print(f"[DEBUG] Node {n} 강제 삽입 성공 (제약 완화)")
        else:
            raise ValueError(f"[❌] 회수노드 {n}은 어떤 차량에도 배정 불가")


def greedy_insertion_vrpb_binpacking(delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K,
                                     problem_info):
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
                if check_feasible_internal([tmp_r], node_types, demands, capa, depot_idx, problem_info):
                    # 증분 비용 계산
                    dist_incr = dist[r[pos - 1]][n] + dist[n][r[pos]] - dist[r[pos - 1]][r[pos]]
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
        force_insert_pickup(unassigned_pick, routes, demands, node_types, capa, dist, depot_idx, problem_info)

    print(f"[INFO] Bin-packing Greedy 초기화 완료. Route 수: {len(routes)} (제한: {K})")
    return routes


def merge_routes_if_possible(routes, demands, node_types, capa, depot_idx, dist, K, problem_info):
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


def cross_route_2opt_star(routes, dist, node_types, demands, capa, depot_idx, problem_info):
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

                        if not check_feasible_internal([new_r1, new_r2], node_types, demands, capa, depot_idx,
                                                       problem_info):
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
        delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K, problem_info
    )
    print(f"[INFO] Initial route count: {len(init_routes)} vehicles (max {K})")

    init_routes = merge_routes_if_possible(
        init_routes, demands, node_types, capa, depot_idx, dist, K, problem_info
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

    best_routes = cross_route_2opt_star(best_routes, dist, node_types, demands, capa, depot_idx, problem_info)
    best_cost = sum(route_cost(r, dist) for r in best_routes)

    routes_kjh = to_kjh_routes(best_routes, depot_idx)
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
    PROBLEM_JSON = ROOT / "instances" / "problem_100_0.7.json"

    run_kjh_problem(PROBLEM_JSON)