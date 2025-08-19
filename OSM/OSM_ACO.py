import random
import numpy
import time
from OSM_util import two_opt


class ACO_VRPB:
    def __init__(self, alpha=1, beta=1.5, sigma=3, ro=0.8, th=80, iterations=200, ants=22, q0=0.9):
        self.alpha = alpha
        self.beta = beta
        self.sigma = sigma
        self.ro = ro
        self.th = th
        self.iterations = iterations
        self.ants = ants
        self.q0 = q0

    def initialize_aco_data(self, capa, node_coords, demands, dist_mat, log=False):
        graph = {i: coord for i, coord in enumerate(node_coords)}
        demand_dict = {i: d for i, d in enumerate(demands)}
        all_nodes = list(graph.keys())

        linehaul_nodes = [node for node in all_nodes if demand_dict.get(node, 0) > 0]
        backhaul_nodes = [node for node in all_nodes if demand_dict.get(node, 0) < 0]

        if log:
            print(f"Depot: 0")
            print(f"Linehaul nodes {len(linehaul_nodes)}: {linehaul_nodes}")
            print(f"Backhaul nodes {len(backhaul_nodes)}: {backhaul_nodes}")

        edges = {}
        num_nodes = len(node_coords)
        for i in range(num_nodes):
            for j in range(i + 1, num_nodes):
                edges[(i, j)] = dist_mat[i][j]

        feromones = {k: 1.0 for k in edges.keys()}

        return linehaul_nodes, backhaul_nodes, edges, capa, demand_dict, feromones

    def _calculate_probabilities(self, current_node, candidate_nodes, feromones, edges):
        probs = []
        for node in candidate_nodes:
            edge = (min(current_node, node), max(current_node, node))
            prob = (feromones.get(edge, 1.0) ** self.alpha) * \
                   ((1 / edges.get(edge, float('inf'))) ** self.beta)
            probs.append(prob)

        sum_probs = numpy.sum(probs)
        if sum_probs == 0:
            return numpy.ones(len(candidate_nodes)) / len(candidate_nodes) if candidate_nodes else numpy.array([])

        return numpy.array(probs) / sum_probs

    def solution_one_ant_VRPB(self, K, all_linehaul, all_backhaul, edges, capacityLimit, demand, feromones,
                              node_coords):
        """
        K-vehicle 제약을 지키는 삽입 휴리스틱 기반 경로 생성
        """
        # 1. 초기화: K개의 차량, 노드, 용량
        clusters = [[] for _ in range(K)]
        linehaul_loads = [0] * K
        backhaul_loads = [0] * K
        last_nodes = [0] * K  # 각 차량의 마지막 위치, 초기값은 Depot(0)

        unvisited_customers = all_linehaul + all_backhaul
        random.shuffle(unvisited_customers)

        # 1. 초기 강제 할당 (Seeding) - 모든 차량에 고객을 1명씩 우선 할당
        if len(unvisited_customers) >= K:
            for i in range(K):
                customer = unvisited_customers[i]
                clusters[i].append(customer)  # i번 차량에 i번 고객을 할당
                last_nodes[i] = customer

                is_linehaul = demand[customer] > 0
                if is_linehaul:
                    linehaul_loads[i] += demand[customer]
                else:
                    backhaul_loads[i] += abs(demand[customer])

            # 이미 할당된 처음 K명의 고객을 제외한 나머지 고객 목록
            remaining_customers = unvisited_customers[K:]
        else:
            # 고객 수가 차량 수보다 적으면, 일부 차량만 사용
            remaining_customers = unvisited_customers

        # 2. 나머지 고객들을 대상으로 기존의 Greedy 방식 적용
        for customer in remaining_customers:
            best_vehicle_idx = -1
            min_insertion_cost = float('inf')
            is_linehaul = demand[customer] > 0

            for i in range(K):
                if is_linehaul:
                    if linehaul_loads[i] + demand[customer] > capacityLimit:
                        continue
                else:
                    if backhaul_loads[i] + abs(demand[customer]) > capacityLimit:
                        continue

                cost = edges.get((min(last_nodes[i], customer), max(last_nodes[i], customer)), float('inf'))

                if cost < min_insertion_cost:
                    min_insertion_cost = cost
                    best_vehicle_idx = i

            if best_vehicle_idx != -1:
                clusters[best_vehicle_idx].append(customer)
                last_nodes[best_vehicle_idx] = customer
                if is_linehaul:
                    linehaul_loads[best_vehicle_idx] += demand[customer]
                else:
                    backhaul_loads[best_vehicle_idx] += abs(demand[customer])
            else:
                # 할당할 수 있는 차량을 찾지 못했다면 빈 리스트를 반환하여 실패 처리
                return []

        # 3. 순서 결정 (ACO)
        solution = []
        for k in range(K):
            cluster_nodes = clusters[k]
            if not cluster_nodes:
                solution.append([])
                continue

            # 배송/반송 노드 분리
            linehaul_part = [node for node in cluster_nodes if demand[node] > 0]
            backhaul_part = [node for node in cluster_nodes if demand[node] < 0]

            # 각 파트 내에서 경로 생성 (ACO 확률 기반)
            path = []
            current_node = 0
            while linehaul_part:
                probabilities = self._calculate_probabilities(current_node, linehaul_part, feromones, edges)
                next_node = numpy.random.choice(linehaul_part, p=probabilities)
                path.append(next_node)
                linehaul_part.remove(next_node)
                current_node = next_node

            while backhaul_part:
                probabilities = self._calculate_probabilities(current_node, backhaul_part, feromones, edges)
                next_node = numpy.random.choice(backhaul_part, p=probabilities)
                path.append(next_node)
                backhaul_part.remove(next_node)
                current_node = next_node

            solution.append(path)

        return solution

    def rate_solution(self, solution, K, edges, demand_dict):
        if not solution or len(solution) != K:
            return float('inf')

        for path in solution:
            if path and not any(demand_dict.get(node, 0) > 0 for node in path):
                return float('inf')

        total_distance = 0
        depot = 0
        for path in solution:
            current_node = depot
            for node in path:
                total_distance += edges.get((min(current_node, node), max(current_node, node)), 0)
                current_node = node
            total_distance += edges.get((min(current_node, depot), max(current_node, depot)), 0)
        return total_distance

    def update_feromone(self, feromones, solutions, bestSolution):
        valid_solutions = [s for s in solutions if s[1] != float('inf')]
        if not valid_solutions:
            feromones = {k: self.ro * v for (k, v) in feromones.items()}
            return bestSolution, feromones

        Lavg = sum(s[1] for s in valid_solutions) / len(valid_solutions)
        feromones = {k: (self.ro + self.th / Lavg) * v for (k, v) in feromones.items()}

        valid_solutions.sort(key=lambda x: x[1])

        if bestSolution is None or bestSolution[1] == float('inf') or valid_solutions[0][1] < bestSolution[1]:
            bestSolution = valid_solutions[0]

        if bestSolution and bestSolution[1] != float('inf'):
            depot = 0
            for path in bestSolution[0]:
                current_node = depot
                for node in path:
                    edge = (min(current_node, node), max(current_node, node))
                    if edge in feromones: feromones[edge] += self.sigma / bestSolution[1]
                    current_node = node
                edge = (min(current_node, depot), max(current_node, depot))
                if edge in feromones: feromones[edge] += self.sigma / bestSolution[1]

        return bestSolution, feromones

    def perturb_solution(self, solution_routes, K, edges, demand_dict, capacity, dist_matrix):
        """
        파괴와 재구성(Destroy and Repair)을 이용한 강력한 흔들기 연산.
        (VRPB 제약조건 검사 로직이 추가된 완전한 버전)
        """
        new_routes = [list(path) for path in solution_routes]

        all_customers = [customer for path in new_routes for customer in path]
        if not all_customers: return new_routes

        num_to_remove = max(1, int(len(all_customers) * 0.20))
        removed_customers = random.sample(all_customers, num_to_remove)

        for i in range(len(new_routes)):
            new_routes[i] = [customer for customer in new_routes[i] if customer not in removed_customers]

        # 제거된 고객들을 다시 최적의 위치에 삽입
        for customer in removed_customers:
            best_insertion_info = {'cost': float('inf'), 'route_idx': -1, 'pos': -1}

            for i, path in enumerate(new_routes):
                # 제약조건 검사를 위해 필요한 정보 계산
                demand_to_add = demand_dict.get(customer, 0)
                is_customer_linehaul = demand_to_add > 0

                current_linehaul_load = sum(demand_dict.get(n, 0) for n in path if demand_dict.get(n, 0) > 0)
                current_backhaul_load = sum(abs(demand_dict.get(n, 0)) for n in path if demand_dict.get(n, 0) < 0)

                last_linehaul_idx = -1
                for idx, node in enumerate(path):
                    if demand_dict.get(node, 0) > 0:
                        last_linehaul_idx = idx

                for pos in range(len(path) + 1):
                    # 용량 및 방문 순서 제약조건 검사
                    # --- 용량 확인 ---
                    if is_customer_linehaul:
                        if current_linehaul_load + demand_to_add > capacity:
                            continue  # 용량 초과 시 이 경로는 더 이상 고려하지 않음
                    else:
                        if current_backhaul_load + abs(demand_to_add) > capacity:
                            continue  # 용량 초과

                    # --- 방문 순서 확인 ---
                    if is_customer_linehaul and pos > last_linehaul_idx + 1:
                        continue  # Linehaul 고객은 Linehaul 구간 뒤에 삽입될 수 없음
                    if not is_customer_linehaul and pos <= last_linehaul_idx + 1:
                        continue  # Backhaul 고객은 Linehaul 구간에 삽입될 수 없음

                    # 3. 제약조건을 통과한 경우에만 비용 계산
                    prev_node = path[pos - 1] if pos > 0 else 0
                    next_node = path[pos] if pos < len(path) else 0
                    cost_increase = (edges.get((min(prev_node, customer), max(prev_node, customer)), float('inf')) +
                                     edges.get((min(customer, next_node), max(customer, next_node)), float('inf')) -
                                     edges.get((min(prev_node, next_node), max(prev_node, next_node)), 0))

                    if cost_increase < best_insertion_info['cost']:
                        best_insertion_info = {'cost': cost_increase, 'route_idx': i, 'pos': pos}

            if best_insertion_info['route_idx'] != -1:
                idx, pos = best_insertion_info['route_idx'], best_insertion_info['pos']
                new_routes[idx].insert(pos, customer)
            else:
                return solution_routes

        return new_routes

    def solve(self, K, capa, node_coords, demands, dist_mat, start_time, time_limit, log=False):
        # 초기화
        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, node_coords,
                                                                                               demands, dist_mat)
        bestSolution = None
        dist_matrix = dist_mat

        stac_count = 0
        stac_limit = 3

        # 메인 루프
        for i in range(self.iterations):

            if time.time() - start_time > time_limit:
                if log:
                    print("\nTime limit (57s) reached. Terminating search and returning best solution found.")
                break  # for 루프를 탈출

            last_best_cost = bestSolution[1] if bestSolution else float('inf')
            solutions = []

            for _ in range(self.ants):
                ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict,
                                                          feromones, node_coords)
                ant_distance = self.rate_solution(ant_solution, K, edges, demand_dict)
                solutions.append((ant_solution, ant_distance))

            bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)

            if bestSolution and bestSolution[1] != float('inf'):
                aco_routes = bestSolution[0]
                # (2-opt 로직은 기존과 동일)
                optimized_routes = []
                for path in aco_routes:
                    if not path:
                        optimized_routes.append([])
                        continue
                    linehaul_part = [node for node in path if demand_dict.get(node, 0) > 0]
                    backhaul_part = [node for node in path if demand_dict.get(node, 0) < 0]

                    if len(linehaul_part) > 1:
                        _, opt_lh_route = two_opt([0] + linehaul_part, dist_matrix)
                        linehaul_part = opt_lh_route[1:]
                    if len(backhaul_part) > 1:
                        start_node = linehaul_part[-1] if linehaul_part else 0
                        _, opt_bh_route = two_opt([start_node] + backhaul_part, dist_matrix)
                        backhaul_part = opt_bh_route[1:]
                    optimized_routes.append(linehaul_part + backhaul_part)

                optimized_distance = self.rate_solution(optimized_routes, K, edges, demand_dict)
                if optimized_distance < bestSolution[1]:
                    bestSolution = (optimized_routes, optimized_distance)
                    if log:
                        print(f"Iteration {i + 1}: 2-opt improved distance to {bestSolution[1]:.2f}")

            if bestSolution and bestSolution[1] < last_best_cost:
                stac_count = 0  # 개선되었으므로 카운터 리셋
            else:
                stac_count += 1  # 개선되지 않았으므로 카운터 증가

            # 정체 한계에 도달하면 '크게 흔들기' 실행
            if stac_count >= stac_limit:
                if log:
                    print(f"\n--- Stagnation detected for {stac_limit} iterations! Applying perturbation. ---")
                perturbed_routes = self.perturb_solution(bestSolution[0], K, edges, demand_dict, capacity, dist_matrix)
                if any(not route for route in perturbed_routes):
                    # 빈 경로가 있다면, 이번 흔들기는 무효 처리하고 다음 반복으로 넘어감
                    if log:
                        print("Perturbation resulted in unused vehicles. Discarding this shake and continuing.")
                    stac_count = 0
                    continue

                perturbed_cost = self.rate_solution(perturbed_routes, K, edges, demand_dict)

                # 흔들린 해가 더 좋을 경우, bestSolution을 '통째로' 교체
                if perturbed_cost < bestSolution[1]:
                    bestSolution = (perturbed_routes, perturbed_cost)
                    if log:
                        print(f"Perturbation resulted in a new best solution with cost: {perturbed_cost:.2f}")

                feromones = {k: 1.0 for k in feromones.keys()}
                stac_count = 0

            # 반복마다 현재 최적 거리 출력
            if bestSolution and bestSolution[1] != float('inf'):
                if log:
                    print(f"Iteration {i + 1}: Best Distance = {bestSolution[1]:.2f}")
            else:
                if log:
                    print(f"Iteration {i + 1}: Finding valid solution...")

        # --- 루프 종료 후 최종 결과 반환 ---
        if bestSolution:
            routes, cost = bestSolution
            routes_with_depot = [[0] + r + [0] if r else [] for r in routes]
            return (routes_with_depot, cost)
        else:
            return bestSolution
