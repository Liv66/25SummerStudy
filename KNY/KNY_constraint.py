# KNY_constraint.py
from typing import List


def is_route_feasible(
    route: List[int],
    node_types: List[int],
    demands: List[int],
    capa: int,
    depot_idx: int,
) -> bool:
    """
    VRPB 6가지 제약 (i)~(vi) 검사
    route : [depot] + 노드들 + [depot]
    node_types[i] : 1 = 배송(line-haul), 0 = 회수(back-haul)
    """
    # (i) Depot 출발·복귀
    if route[0] != depot_idx or route[-1] != depot_idx:
        return False

    # (iv) 배송 먼저, 그다음 회수
    seen_pick = False
    for n in route[1:-1]:            # depot 제외
        if node_types[n] == 0:       # 회수
            seen_pick = True
        elif seen_pick:              # 회수 뒤에 배송 등장 → 위배
            return False

    # (vi) 최소 1개의 배송 노드 포함  ← depot 인덱스 제외
    if not any(node_types[n] == 1 for n in route if n != depot_idx):
        return False

    # (v) 용량 제약 (depot 제외하고 계산)
    delivery_sum = sum(demands[n] for n in route if n != depot_idx and node_types[n] == 1)
    pickup_sum   = sum(demands[n] for n in route if n != depot_idx and node_types[n] == 0)

    if delivery_sum > capa:                     # 출발 시 적재 초과
        return False
    if pickup_sum > (capa - delivery_sum):      # 회수 총량 > 잔여 공간
        return False

    return True


def is_solution_feasible(routes, node_types, demands, capa, depot_idx):
    """
    솔루션 전체 제약 검사
      - (i)~(vi) 각 경로 검사
      - (iii) 모든 고객 노드를 1회 '정확히' 방문
    """
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
