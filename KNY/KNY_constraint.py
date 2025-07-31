# KNY_constraint.py
from typing import List


def is_route_feasible(
    route: List[int],
    node_types: List[int],
    demands: List[int],
    capa: int,
    depot_idx: int,
) -> bool:
    if route[0] != depot_idx or route[-1] != depot_idx:
        return False

    # 배송 먼저, 회수는 뒤에
    seen_pick = False
    for n in route[1:-1]:
        if node_types[n] == 0:
            seen_pick = True
        elif seen_pick:
            return False  # 회수 뒤에 배송 → 제약 위반

    # ✅ 적재 시뮬레이션: 배송량 누적, 회수 시작 시 초기화 후 회수 누적
    load = 0
    in_pickup = False
    for n in route[1:-1]:
        if node_types[n] == 1:  # 배송
            if in_pickup:
                return False  # pickup 이후 배송 → 제약 위반
            load += demands[n]
            if load > capa:
                return False
        else:  # 회수
            if not in_pickup:
                in_pickup = True
                load = 0  # 회수 시작 시 적재 초기화
            load += demands[n]
            if load > capa:
                return False

    # 최소 1개의 배송 노드는 포함해야 함
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
