import json
import re
from collections import Counter

# 1. 문제 데이터 불러오기
with open("../../instances/problem_100_0.7.json", encoding="utf-8") as f:
    data = json.load(f)
N = data["N"]
K = data["K"]
vehicle_capacity = data["capa"]
node_demands = data["node_demands"]
node_types = data["node_types"]

# 2. output.sol에서 라우트 읽기
with open("output.sol", encoding="utf-8") as f:
    lines = f.readlines()

routes = []
for line in lines:
    if line.startswith("Route"):
        route = list(map(int, re.findall(r'\d+', line)))[1:]
        routes.append(route)

# 3. 검증 체크
all_visited = [node for route in routes for node in route[1:-1]]
node_counts = Counter(all_visited)

# 차량 대수 초과
over_vehicle = len(routes) > K

# depot 출발/도착
depot_ok = all(route[0] == 0 and route[-1] == 0 for route in routes)

# depot 처음/끝만 방문
depot_twice = all(route.count(0) == 2 for route in routes)

# 각 차량에 linehaul(1)이 반드시 있는지
linehaul_ids = [i for i, t in enumerate(node_types) if t == 1]
backhaul_ids = [i for i, t in enumerate(node_types) if t == 2]
linehaul_visited = [
    any(node in linehaul_ids for node in route[1:-1])
    for route in routes
]
has_linehaul = all(linehaul_visited)

# linehaul → backhaul → linehaul 패턴
forbidden_pattern = False
for route in routes:
    pure = route[1:-1]
    if not pure: continue
    types = [node_types[n] for n in pure]
    for i in range(1, len(types)-1):
        if types[i-1] == 1 and types[i] == 2 and 1 in types[i+1:]:
            forbidden_pattern = True
            break

# depot 제외 중복 방문 노드
duplicated_nodes = [node for node, cnt in node_counts.items() if cnt > 1]

# 미방문 노드
missed_nodes = set(range(1, N)) - set(all_visited)

# 각 차량 linehaul, backhaul 적재량 체크
all_line_ok, all_back_ok = True, True
for idx, route in enumerate(routes):
    pure = route[1:-1]
    line_load = sum(node_demands[n] for n in pure if n in linehaul_ids)
    back_load = sum(node_demands[n] for n in pure if n in backhaul_ids)
    if line_load > vehicle_capacity:
        print(f"🚨 차량 {idx+1} linehaul 적재 초과! {line_load} > {vehicle_capacity}")
        all_line_ok = False
    if back_load > vehicle_capacity:
        print(f"🚨 차량 {idx+1} backhaul 적재 초과! {back_load} > {vehicle_capacity}")
        all_back_ok = False

print("노드 번호 예시:", pure)
print("demand 예시:", [node_demands[n] for n in pure])
print(node_demands)

summary_ok = (
    not over_vehicle and depot_ok and depot_twice and has_linehaul
    and not forbidden_pattern and not duplicated_nodes
    and all_line_ok and all_back_ok and not missed_nodes
)

print("===== 솔루션 검증 결과 =====")
print(f"차량 대수 초과: {over_vehicle}")
print(f"모든 차량 depot 출발/도착: {depot_ok}")
print(f"depot 처음/끝만 방문: {depot_twice}")
print(f"모든 차량 라인홀 방문: {has_linehaul}")
print(f"Linehaul→Backhaul→Linehaul 패턴 존재: {forbidden_pattern}")
print(f"중복 방문 노드: {duplicated_nodes}")
print(f"모든 차량 linehaul 용량 만족: {all_line_ok}")
print(f"모든 차량 backhaul 용량 만족: {all_back_ok}")
print(f"방문 안 한 노드: {missed_nodes}")

if summary_ok:
    print("✅ 모든 제약조건 통과! (feasible solution)")

# (선택) 용량 초과 차량 상세
for idx, route in enumerate(routes):
    pure = route[1:-1]
    line_load = sum(node_demands[n] for n in pure if n in linehaul_ids)
    back_load = sum(node_demands[n] for n in pure if n in backhaul_ids)
    if line_load > vehicle_capacity or back_load > vehicle_capacity:
        print(f"  차량 {idx+1}: line {line_load}, back {back_load}, route: {pure}")

