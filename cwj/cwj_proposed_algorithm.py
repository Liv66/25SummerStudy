import json
from cwj_master_problem import solve_master_problem_lp, solve_master_problem_ip
from cwj_pricing_problem import solve_pricing_problem
from cwj_initial_patterns import generate_initial_patterns  # 🔸 추가

def load_instance(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)

def run_column_generation(problem_info):
    K = problem_info['K']
    capa = problem_info['capa']
    node_types = problem_info['node_types']
    node_demands = problem_info['node_demands']
    dist_mat = problem_info['dist_mat']
    N = problem_info['N']

    # 🔸 초기 패턴을 greedy 방식으로 생성
    patterns, costs = generate_initial_patterns(
    node_types, node_demands, capa, dist_mat, K)

    # 🔸 강제 사용 패턴 누적 관리
    force_use_ids = set()

    while True:
        print("========== [DEBUG] Accumulated Forced Pattern List ==========")
        if force_use_ids:
            for pid in sorted(force_use_ids):
                print(f"[FORCE] Pattern {pid}: {patterns[pid]}")
        else:
            print("[DEBUG] No patterns are forced to be used.")
        print("===============================================================")

        # 🔸 Master Problem (LP relaxation)
        model, x_vars, duals = solve_master_problem_lp(
            patterns, costs, N, force_use_ids, K
        )

        # 🔸 강제 사용된 패턴에 포함된 고객 노드를 모두 수집
        forbidden_nodes = set()
        for idx in force_use_ids:
            forbidden_nodes.update(patterns[idx][1:-1])  # depot 제외

        # 🔸 Pricing Problem (금지 노드를 제외하고 라우트 생성)
        new_route, reduced_cost = solve_pricing_problem(
            duals,
            node_types,
            node_demands,
            capa,
            dist_mat,
            forbidden_nodes=forbidden_nodes
        )

        if not new_route or reduced_cost > -1e-6:
            print("[INFO] No improving pattern found. Terminating column generation.")
            break

        # 🔸 새 패턴이 포함한 고객 노드 (depot 제외)
        new_nodes = set(new_route[1:-1])

        # 🔸 기존 패턴 중 일부 노드와 겹치는 경우 제거
        remove_indices = set()
        for idx, pat in enumerate(patterns):
            if idx in force_use_ids:
                continue
            if any(n in new_nodes for n in pat[1:-1]):
                remove_indices.add(idx)

        # 🔸 패턴 제거 (역순으로 pop) + force_use_ids 인덱스 재정렬
        for idx in sorted(remove_indices, reverse=True):
            print(f"[DEBUG] Removing pattern {idx} due to overlap with new pattern: {patterns[idx]}")
            patterns.pop(idx)
            costs.pop(idx)
            force_use_ids = {j - 1 if j > idx else j for j in force_use_ids}

        # 🔸 새로운 패턴 추가 및 강제 사용 등록
        new_cost = sum(dist_mat[new_route[i]][new_route[i+1]] for i in range(len(new_route) - 1))
        patterns.append(new_route)
        costs.append(new_cost)
        new_pattern_id = len(patterns) - 1
        force_use_ids.add(new_pattern_id)

        print(f"[DEBUG] New pattern added: {new_route}, Reduced cost: {reduced_cost:.4f}")

    # 🔸 최종 Set Partitioning 문제 (IP) 풀기
    final_model, final_x = solve_master_problem_ip(patterns, costs, N, K)

    final_routes = []
    for j, xj in enumerate(final_x):
        if xj.X > 0.5:
            final_routes.append(patterns[j])
            print(f"[DEBUG] Final route used: Pattern {j}, Route: {patterns[j]}")

    return final_routes