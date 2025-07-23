# KNY_constraint.py
from typing import List


def is_route_feasible(
    route: List[int],
    node_types: List[int],
    demands: List[int],
    capa: int,
    depot_idx: int,
) -> bool:
    # (i) Depot 출발·복귀
    if route[0] != depot_idx or route[-1] != depot_idx:
        return False

    # (iv) 배송 먼저, 그다음 회수
    seen_pick = False
    for n in route[1:-1]:  # depot 제외
        if node_types[n] == 0:
            seen_pick = True
        elif seen_pick:
            return False  # 회수 뒤에 배송 등장 → 위배

    # (v) 적재량 시뮬레이션
    load = capa  # 차량은 풀 적재로 출발
    for n in route[1:-1]:
        if node_types[n] == 1:
            load -= demands[n]   # 배송하면 감소
            if load < 0:        # 적재량 음수 불가
                return False
        else:
            load += demands[n]    # 회수하면 증가
            if load > capa:       # 용량 초과 불가
                return False

    # (vi) 적어도 1개는 고객 방문(실제로는 node_types==[]면 N=0아닌 한 여기서는 필요없음)
    # (vi) 최소 1개의 배송 노드 포함
    if not any(node_types[n] == 1 for n in route if n != depot_idx):
        return False

    return True



def is_solution_feasible(
    routes: List[List[int]],
    node_types: List[int],
    demands: List[int],
    capa: int,
    depot_idx: int,
    max_vehicles: int | None = None
) -> bool:

    if max_vehicles is not None and len(routes) > max_vehicles:
        return False

    visited = set()
    for r in routes:
        if not is_route_feasible(r, node_types, demands, capa, depot_idx):
            return False
        for n in r[1:-1]:            # depot 제외
            if n in visited:         # 중복 방문
                return False
            visited.add(n)

    # ▶︎ 빠진 노드 없는지 확인  (모든 고객을 방문했는가?)
    if len(visited) != len(node_types):          # node_types 길이 == 고객 수 N
        return False

    return True
