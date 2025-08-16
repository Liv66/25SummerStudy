import json
import random
import math
import time
from pathlib import Path
from typing import List
import csv
import os
import numpy as np

# util.py와 KNY_alns.py는 동일한 폴더에 있다고 가정합니다.
from util import get_distance, plot_vrpb, check_feasible
from KNY_alns import alns_vrpb

# 로깅 설정은 코드 맨 아래 __main__ 블록에서 최종적으로 제어합니다.
ENABLE_LOGGING = False


# random.seed(42)

# ─────────────────────────────────────────────────────────────
# 0) 공용 도우미 함수 / 클래스
# ─────────────────────────────────────────────────────────────
def log_print(*args, **kwargs):
    if ENABLE_LOGGING:
        print(*args, **kwargs)


def route_cost(route: list[int], dist: list[list[float]]) -> float: return sum(
    dist[route[i]][route[i + 1]] for i in range(len(route) - 1))


class DistanceCache:
    def __init__(self, dist_matrix: List[List[float]]):
        self.dist, self.cache, self.insertion_cache = dist_matrix, {}, {}

    def get_route_cost(self, route):
        key = tuple(route); self.cache.setdefault(key, route_cost(route, self.dist)); return self.cache[key]

    def get_insertion_cost(self, route, node, pos):
        route_key, key = tuple(route), (tuple(route), node, pos)
        if key not in self.insertion_cache:
            if pos >= len(route): return float('inf')
            self.insertion_cache[key] = (
                        self.dist[route[pos - 1]][node] + self.dist[node][route[pos]] - self.dist[route[pos - 1]][
                    route[pos]])
        return self.insertion_cache[key]

    def clear_cache(self):
        self.cache.clear(); self.insertion_cache.clear()


cache = None


def fast_feasible_check(route: List[int], node_types: List[int], demands: List[int], capa: int, depot_idx: int) -> bool:
    if route[0] != depot_idx or route[-1] != depot_idx: return False
    if len(route) < 3: return True
    if node_types[route[1]] == 0: return False
    load, in_pick = 0, False
    for prev, curr in zip(route[:-1], route[1:]):
        if node_types[prev] == 1 and node_types[curr] == 0:
            if in_pick: return False
            in_pick, load = True, 0
        load += demands[curr]
        if load > capa: return False
    return True


def find_best_insertion_fast(pickup_node: int, routes: List[List[int]], node_types: List[int], demands: List[int],
                             capa: int, dist: List[List[float]], depot_idx: int):
    global cache
    best_route_idx, best_pos, best_inc = None, None, float("inf")
    for ridx, r in enumerate(routes):
        if sum(demands[v] for v in r[1:-1] if node_types[v] == 1) == 0: continue
        if sum(demands[v] for v in r[1:-1] if node_types[v] == 0) + demands[pickup_node] > capa: continue
        last_deliv = max((i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1), default=0)
        for pos in range(last_deliv + 1, len(r)):
            if not fast_feasible_check(r[:pos] + [pickup_node] + r[pos:], node_types, demands, capa,
                                       depot_idx): continue
            inc = cache.get_insertion_cost(r, pickup_node, pos) if cache else (
                        dist[r[pos - 1]][pickup_node] + dist[pickup_node][r[pos]] - dist[r[pos - 1]][r[pos]])
            if inc < best_inc: best_inc, best_route_idx, best_pos = inc, ridx, pos
    return best_route_idx, best_pos, best_inc


def improved_initial_solution(delivery_idx: List[int], pickup_idx: List[int], demands: List[int], capa: int,
                              dist: List[List[float]], depot_idx: int, node_types: List[int], K: int):
    routes, used_deliv, used_pick = [], set(), set()
    deliv_items = sorted(((n, demands[n]) for n in delivery_idx), key=lambda x: x[1], reverse=True)
    pickup_items = [(n, demands[n]) for n in pickup_idx]
    for d_node, d_dem in deliv_items:
        if d_node in used_deliv: continue
        cur_route, cur_load = [depot_idx, d_node], d_dem;
        used_deliv.add(d_node)
        for o_node, o_dem in deliv_items:
            if o_node in used_deliv or cur_load + o_dem > capa or dist[d_node][o_node] > dist[d_node][
                depot_idx] * 1.5: continue
            cur_route.insert(-1, o_node);
            cur_load += o_dem;
            used_deliv.add(o_node)
        last_deliv = cur_route[-1] if len(cur_route) > 2 else cur_route[-2]
        cand = sorted([(p_node, p_dem, dist[last_deliv][p_node]) for p_node, p_dem in pickup_items if
                       p_node not in used_pick and p_dem <= capa], key=lambda x: x[2])
        pick_load = 0
        for p_node, p_dem, _ in cand:
            if pick_load + p_dem <= capa:
                cur_route.append(p_node);
                pick_load += p_dem;
                used_pick.add(p_node)
        cur_route.append(depot_idx);
        routes.append(cur_route)
        if len(routes) >= K: break
    rem_deliv = [n for n in delivery_idx if n not in used_deliv]
    for d in rem_deliv:
        placed = False
        for r in routes:
            if sum(demands[v] for v in r[1:-1] if node_types[v] == 1) + demands[d] <= capa:
                r.insert(max((i for i, v in enumerate(r) if v != depot_idx and node_types[v] == 1), default=0) + 1, d);
                placed = True;
                break
        if not placed and len(routes) < K: routes.append([depot_idx, d, depot_idx])
    rem_pick = [n for n in pickup_idx if n not in used_pick]
    for p in rem_pick:
        ridx, pos, _ = find_best_insertion_fast(p, routes, node_types, demands, capa, dist, depot_idx)
        if ridx is not None: routes[ridx].insert(pos, p)
    return routes


def improved_greedy_vrpb(delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K):
    log_print("[INFO] 개선된 Greedy VRPB 초기화…")
    if 'cache' in globals() and cache: cache.clear_cache()
    routes = improved_initial_solution(delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K)
    for p in pickup_idx:
        if any(p in r for r in routes): continue
        ridx, pos, _ = find_best_insertion_fast(p, routes, node_types, demands, capa, dist, depot_idx)
        if ridx is not None:
            routes[ridx].insert(pos, p)
        elif len(routes) < K:
            routes.append([depot_idx, p, depot_idx])
        else:
            best_r = max(routes, key=lambda r: sum(demands[v] for v in r[1:-1] if node_types[v] == 1) - sum(
                demands[v] for v in r[1:-1] if node_types[v] == 0))
            best_r.insert(
                max((i for i, v in enumerate(best_r) if v != depot_idx and node_types[v] == 1), default=0) + 1, p)
    log_print(f"[INFO] 개선된 Greedy 완료 · Route 수 = {len(routes)} (K={K})")
    return routes


def convert_kjh_problem(problem_info: dict):
    capa, coords, demands_all, types_all = problem_info["capa"], problem_info["node_coords"], [abs(d) for d in
                                                                                               problem_info[
                                                                                                   "node_demands"]], \
    problem_info["node_types"]
    delivery_idx, pickup_idx, node_types_internal, demands_internal = [], [], [0] * len(coords), [0] * len(coords)
    for j in range(1, len(coords)):
        kjh_type = types_all[j]
        if kjh_type == 1:
            node_types_internal[j], delivery_idx = 1, delivery_idx + [j]
        elif kjh_type == 2:
            node_types_internal[j], pickup_idx = 0, pickup_idx + [j]
        demands_internal[j] = demands_all[j]
    return delivery_idx, pickup_idx, demands_internal, capa, problem_info["dist_mat"], 0, node_types_internal, coords


def to_kjh_types(node_types_internal): return [0 if i == 0 else 1 if t == 1 else 2 for i, t in
                                               enumerate(node_types_internal)]


def cross_route_2opt_star(routes, dist, node_types, demands, capa, depot_idx):
    changed = True
    while changed:
        changed = False
        for i in range(len(routes)):
            for j in range(i + 1, len(routes)):
                r1, r2 = routes[i], routes[j]
                for idx1 in range(1, len(r1) - 1):
                    for idx2 in range(1, len(r2) - 1):
                        if node_types[r1[idx1]] != node_types[r2[idx2]]: continue
                        new_r1, new_r2 = r1[:idx1] + [r2[idx2]] + r1[idx1 + 1:], r2[:idx2] + [r1[idx1]] + r2[idx2 + 1:]
                        if not fast_feasible_check(new_r1, node_types, demands, capa,
                                                   depot_idx) or not fast_feasible_check(new_r2, node_types, demands,
                                                                                         capa, depot_idx): continue
                        if 'cache' in globals() and cache:
                            if cache.get_route_cost(r1) + cache.get_route_cost(r2) > cache.get_route_cost(
                                    new_r1) + cache.get_route_cost(new_r2):
                                routes[i], routes[j], changed = new_r1, new_r2, True
    return routes


def get_solution_stats(routes, dist, demands, capa, node_types):
    if not routes: return {k: 0 for k in
                           ['obj', 'num_vehicles', 'mean_dist', 'std_dist', 'min_dist', 'max_dist', 'mean_load',
                            'std_load', 'min_load', 'max_load', 'mean_back_load', 'std_back_load', 'min_back_load',
                            'max_back_load']}
    route_dists = [sum(dist[r[i]][r[i + 1]] for i in range(len(r) - 1)) for r in routes]
    delivery_loads = [sum(demands[n] for n in r if node_types[n] == 1) for r in routes]
    delivery_util = [load / capa if capa > 0 else 0 for load in delivery_loads]
    backhaul_loads = []
    for r in routes:
        in_pick, load = False, 0
        for i in range(1, len(r) - 1):
            if not in_pick and node_types[r[i - 1]] == 1 and node_types[r[i]] == 0: in_pick = True
            if in_pick and node_types[r[i]] == 0: load += demands[r[i]]
        backhaul_loads.append(load)
    return {'obj': sum(route_dists), 'num_vehicles': len(routes), 'mean_dist': np.mean(route_dists),
            'std_dist': np.std(route_dists), 'min_dist': np.min(route_dists), 'max_dist': np.max(route_dists),
            'mean_load': np.mean(delivery_util) * 100, 'std_load': np.std(delivery_util) * 100,
            'min_load': np.min(delivery_util) * 100, 'max_load': np.max(delivery_util) * 100,
            'mean_back_load': np.mean(backhaul_loads), 'std_back_load': np.std(backhaul_loads),
            'min_back_load': np.min(backhaul_loads), 'max_back_load': np.max(backhaul_loads)}


### ★ 1. 원본 코드의 열 순서와 완전히 동일하게 fieldnames 수정 ###
def log_raw_result(filepath, result_data):
    file_exists = os.path.isfile(filepath)
    fieldnames = [
        'instance', 'obj', 'mean_dist', 'std_dist', 'max_dist', 'min_dist',
        'std_line_load', 'max_line_load', 'min_line_load',
        'std_back_load', 'max_back_load', 'min_back_load',
        'num_vehicle', 'alns_time', 'total_time', 'method', '수정한 부분'
    ]
    with open(filepath, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, restval='')
        if not file_exists:
            writer.writeheader()
        writer.writerow(result_data)


# ─────────────────────────────────────────────────────────────
# 7) 메인 드라이버
# ─────────────────────────────────────────────────────────────
def KJH_main(problem_info: dict):
    global cache, ENABLE_LOGGING
    K = problem_info["K"]
    (delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, coords) = convert_kjh_problem(problem_info)
    cache = DistanceCache(dist)

    init_start = time.time()
    init_routes = improved_greedy_vrpb(delivery_idx, pickup_idx, demands, capa, dist, depot_idx, node_types, K)
    init_elapsed = time.time() - init_start

    alns_start = time.time()
    best_routes, _ = alns_vrpb(init_routes, dist, node_types, demands, capa, depot_idx, max_vehicles=K, time_limit=57)
    alns_elapsed = time.time() - alns_start

    post_proc_start = time.time()
    best_routes = cross_route_2opt_star(best_routes, dist, node_types, demands, capa, depot_idx)
    post_proc_elapsed = time.time() - post_proc_start

    if ENABLE_LOGGING:
        total_elapsed = init_elapsed + alns_elapsed + post_proc_elapsed
        log_print(f"[⏱️] 총 실행 시간: {total_elapsed:.2f}초")
        final_stats = get_solution_stats(best_routes, dist, demands, capa, node_types)

        experiment_notes = {'수정한 부분': '없음'}
        log_print(f"[🔬] 이번 실행 내용: {experiment_notes['수정한 부분']}")

        ### ★ 2. 원본 코드의 열 순서와 완전히 동일하게 log_data 생성 ###
        log_data = {
            'instance': problem_info.get('instance_name', 'unknown'),
            'obj': final_stats['obj'],
            'mean_dist': round(final_stats['mean_dist'], 2),
            'std_dist': round(final_stats['std_dist'], 2),
            'max_dist': round(final_stats['max_dist'], 2),
            'min_dist': round(final_stats['min_dist'], 2),
            'std_line_load': round(final_stats['std_load'], 2),
            'max_line_load': round(final_stats['max_load'], 2),
            'min_line_load': round(final_stats['min_load'], 2),
            'std_back_load': round(final_stats['std_back_load'], 2),
            'max_back_load': round(final_stats['max_back_load'], 2),
            'min_back_load': round(final_stats['min_back_load'], 2),
            'num_vehicle': final_stats['num_vehicles'],
            'alns_time': round(alns_elapsed, 2),
            'total_time': round(total_elapsed, 2),
            'method': 1
        }
        log_data.update(experiment_notes)
        log_raw_result('raw_results.csv', log_data)
        log_print(f"[📝] 'raw_results.csv' 파일에 결과가 기록되었습니다.")

    return best_routes


# ─────────────────────────────────────────────────────────────
# 8) 프로그램 실행 부분
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ENABLE_LOGGING = False
    try:
        ROOT = Path(__file__).resolve().parents[1]
        instance_path = ROOT / "instances" / "problem_20_0.7.json"
    except IndexError:
        instance_path = Path("./instances/problem_20_0.7.json")

    try:
        with open(instance_path, "r", encoding="utf-8") as f:
            problem_data = json.load(f)
        problem_data['instance_name'] = instance_path.stem
    except FileNotFoundError:
        print(f"[오류] 문제 파일을 찾을 수 없습니다: {instance_path}")
        problem_data = None

    if problem_data:
        solution = KJH_main(problem_data)
        for route in solution:
            print(route)