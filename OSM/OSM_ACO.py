import random
import numpy
from OSM_util import get_distance, two_opt
import time
class ACO_VRPB:
    def __init__(self, alpha=1, beta=1.5, sigma=3, ro=0.8, th=80, iterations=200, ants=22, q0=0.9, stagnation_limit=3, perturbation_strength=1):
        self.alpha = alpha
        self.beta = beta
        self.sigma = sigma 
        self.ro = ro
        self.th = th
        self.iterations = iterations
        self.ants = ants
        self.q0 = q0

        self.tau_max = None
        self.tau_min = None

        self.stagnation_limit = stagnation_limit  # 이 횟수만큼 해가 개선 안되면 흔들기 발동
        self.perturbation_strength = perturbation_strength # 흔들기 강도 (재배치할 고객 수)
    
    def initialize_aco_data(self, capa, nodes_coord, demands):
        graph = {i: coord for i, coord in enumerate(nodes_coord)}
        demand_dict = {i: d for i, d in enumerate(demands)}
        all_nodes = list(graph.keys())
        
        linehaul_nodes = [node for node in all_nodes if demand_dict.get(node, 0) > 0]
        backhaul_nodes = [node for node in all_nodes if demand_dict.get(node, 0) < 0]

        print(f"Depot: 0")
        print(f"Linehaul nodes {len(linehaul_nodes)}: {linehaul_nodes}")
        print(f"Backhaul nodes {len(backhaul_nodes)}: {backhaul_nodes}")

        edges = {(min(a, b), max(a, b)): numpy.sqrt((graph[a][0] - graph[b][0])**2 + (graph[a][1] - graph[b][1])**2)
                 for a in all_nodes for b in all_nodes if a != b}
        
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

    def solution_one_ant_VRPB(self, K, all_linehaul, all_backhaul, edges, capacityLimit, demand, feromones, nodes_coord):
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

        # 2. 비용이 적은 곳으로 고객 배치 - greedy하게
        for customer in unvisited_customers:
            best_vehicle_idx = -1
            min_insertion_cost = float('inf')
            is_linehaul = demand[customer] > 0

            for i in range(K):
                # 용량 제약 조건
                if is_linehaul:
                    if linehaul_loads[i] + demand[customer] > capacityLimit:
                        continue
                else:
                    if backhaul_loads[i] + abs(demand[customer]) > capacityLimit:
                        continue
                
                # 비용 계산 (마지막 노드에서 이 고객까지의 거리)
                cost = edges.get((min(last_nodes[i], customer), max(last_nodes[i], customer)), float('inf'))

                if cost < min_insertion_cost:
                    min_insertion_cost = cost
                    best_vehicle_idx = i
            
            # 가장 비용이 적은 차량에 고객 할당
            if best_vehicle_idx != -1:
                clusters[best_vehicle_idx].append(customer)
                last_nodes[best_vehicle_idx] = customer # 차량의 마지막 위치 업데이트
                if is_linehaul:
                    linehaul_loads[best_vehicle_idx] += demand[customer]
                else:
                    backhaul_loads[best_vehicle_idx] += abs(demand[customer])
            else:
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


    def _perturb_solution(self, solution_routes, K, edges, demand_dict, capacity, dist_matrix):
        """
        현재 해의 클러스터링을 강제로 변경하는 '재배치(Relocate)' 연산.
        (용량 및 방문 순서 제약조건을 만족하도록 수정됨)
        """
        # print(f"\n--- Perturbation Triggered! Shaking the solution (Strength: {self.perturbation_strength}) ---")
        
        new_routes = [list(path) for path in solution_routes] # 복사본으로 작업
        
        for _ in range(self.perturbation_strength):
            non_empty_routes_indices = [i for i, path in enumerate(new_routes) if path]
            if len(non_empty_routes_indices) < 2:
                return new_routes

            from_route_idx, to_route_idx = random.sample(non_empty_routes_indices, 2)
            
            if not new_routes[from_route_idx]: continue # 혹시 모를 비어있는 경로 선택 방지
            customer_to_move = random.choice(new_routes[from_route_idx])
            new_routes[from_route_idx].remove(customer_to_move)
            
            best_pos = -1
            min_cost_increase = float('inf')
            
            target_path = new_routes[to_route_idx]


            # 1. 옮길 고객과 도착 경로의 현재 적재량 계산
            demand_to_move = demand_dict.get(customer_to_move, 0)
            is_move_customer_linehaul = demand_to_move > 0
            
            current_linehaul_load = sum(demand_dict.get(n, 0) for n in target_path if demand_dict.get(n, 0) > 0)
            current_backhaul_load = sum(abs(demand_dict.get(n, 0)) for n in target_path if demand_dict.get(n, 0) < 0)

            # 2. 도착 경로(target_path)에서 Linehaul 고객이 끝나는 지점 찾기
            last_linehaul_idx = -1
            for idx, node in enumerate(target_path):
                if demand_dict.get(node, 0) > 0:
                    last_linehaul_idx = idx

            # 3. 모든 가능한 위치에 대해 제약조건을 만족하는지 확인하며 최적 위치 탐색
            for pos in range(len(target_path) + 1):
                # --- 제약조건 1: 용량 확인 ---
                if is_move_customer_linehaul:
                    if current_linehaul_load + demand_to_move > capacity:
                        continue # 용량 초과 시 이 경로는 더 이상 고려하지 않고 다음 시도로 넘어감
                else: # Backhaul 고객일 경우
                    if current_backhaul_load + abs(demand_to_move) > capacity:
                        continue # 용량 초과

                # --- 제약조건 2: 방문 순서(선행) 확인 ---
                is_order_valid = False
                if is_move_customer_linehaul:
                    # Linehaul 고객은 기존 Linehaul 고객들 사이에만 삽입 가능
                    if pos <= last_linehaul_idx + 1:
                        is_order_valid = True
                else: # Backhaul 고객
                    # Backhaul 고객은 모든 Linehaul 고객들 뒤에만 삽입 가능
                    if pos > last_linehaul_idx:
                        is_order_valid = True
                
                if not is_order_valid:
                    continue # 방문 순서 위반 시 이 위치(pos)는 건너뛰고 다음 위치 탐색

                # --- 모든 제약조건 통과 시, 비용 계산 ---
                prev_node = target_path[pos-1] if pos > 0 else 0
                next_node = target_path[pos] if pos < len(target_path) else 0
                cost_increase = (edges.get((min(prev_node, customer_to_move), max(prev_node, customer_to_move)), float('inf')) +
                                edges.get((min(customer_to_move, next_node), max(customer_to_move, next_node)), float('inf')) -
                                edges.get((min(prev_node, next_node), max(prev_node, next_node)), 0))
                
                if cost_increase < min_cost_increase:
                    min_cost_increase = cost_increase
                    best_pos = pos
            

            if best_pos != -1:
                new_routes[to_route_idx].insert(best_pos, customer_to_move)
            else:
                # 만약 옮길 고객을 삽입할 유효한 위치가 없었다면, 원래 경로로 되돌림
                new_routes[from_route_idx].append(customer_to_move)


        # 4. 각 경로에 대해 2-opt 적용
        for i, path in enumerate(new_routes):
            if len(path) > 1:
                # 2-opt는 순서만 바꾸므로, 제약조건이 깨지지 않도록 Line/Back 분리 후 적용
                linehaul_part = [node for node in path if demand_dict.get(node, 0) > 0]
                backhaul_part = [node for node in path if demand_dict.get(node, 0) < 0]

                if len(linehaul_part) > 1:
                    _, opt_lh_route = two_opt([0] + linehaul_part, dist_matrix)
                    linehaul_part = opt_lh_route[1:]
                
                if len(backhaul_part) > 1:
                    start_node = linehaul_part[-1] if linehaul_part else 0
                    _, opt_bh_route = two_opt([start_node] + backhaul_part, dist_matrix)
                    backhaul_part = opt_bh_route[1:]

                new_routes[i] = linehaul_part + backhaul_part

        return new_routes


    def update_feromone_mmas(self, feromones, best_solution):

        for edge in feromones:
            feromones[edge] *= self.ro

        # 2. 최고의 해(경로)에만 페로몬을 추가
        if best_solution and best_solution[1] != float('inf'):
            depot = 0
            pheromone_to_add = 1.0 / best_solution[1] # 추가할 페로몬 양
            for path in best_solution[0]:
                current_node = depot
                for node in path:
                    edge = (min(current_node, node), max(current_node, node))
                    if edge in feromones:
                        feromones[edge] += pheromone_to_add
                    current_node = node
                edge = (min(current_node, depot), max(current_node, depot))
                if edge in feromones:
                    feromones[edge] += pheromone_to_add

        # 3. 모든 페로몬이 상한/하한을 벗어나지 않도록 강제 조정
        for edge in feromones:
            if feromones[edge] > self.tau_max:
                feromones[edge] = self.tau_max
            elif feromones[edge] < self.tau_min:
                feromones[edge] = self.tau_min
        
        return feromones

   
    def solve_mmas(self, K, capa, nodes_coord, demands):
        # 1. 데이터 초기화 (기존과 동일)
        start_time = time.time()
        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
        global_best_solution = None
        dist_matrix = get_distance(nodes_coord)

        stagnation_counter = 0
        
        # 2. MMAS를 위한 τ_max, τ_min 초기값 계산 (기존과 동일)
        print("Calculating initial greedy solution for MMAS...")
        # ... (이 부분은 기존 코드와 완전히 동일하므로 생략) ...
        initial_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, {k:1.0 for k in feromones.keys()}, nodes_coord)
        initial_cost = self.rate_solution(initial_solution, K, edges, demand_dict)
        if initial_cost == float('inf'):
            initial_cost = 10000 
            print("Warning: Initial greedy solution is invalid. Using default cost.")
        self.tau_max = 1.0 / ((1 - self.ro) * initial_cost)
        self.tau_min = self.tau_max / (2 * len(nodes_coord))
        print(f"Initial Cost: {initial_cost:.2f}, tau_max: {self.tau_max:.4f}, tau_min: {self.tau_min:.4f}")
        feromones = {k: self.tau_max for k in feromones.keys()}
        print("Pheromones initialized to tau_max.")

        # --- 메인 루프 ---
        for i in range(self.iterations):
            if time.time() - start_time > 57:  # 1시간 제한
                print("Time limit exceeded. Stopping iterations.")
                break

            # 이번 세대 시작 전의 최고 비용을 기록
            last_best_cost = global_best_solution[1] if global_best_solution else float('inf')

            solutions = []
            for _ in range(self.ants):
                ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, nodes_coord)
                optimized_ant_routes = []
                if ant_solution:
                    for path in ant_solution:
                        # 배송/반송 분리 및 2-opt 적용
                        if not path:
                            optimized_ant_routes.append([])
                            continue
                        #1. 배송/반송 분리
                        linehaul_part = [node for node in path if demand_dict.get(node, 0) > 0]
                        backhaul_part = [node for node in path if demand_dict.get(node, 0) < 0]
                        
                        # 2. 배송/반송 각각 2-opt 적용
                        if len(linehaul_part) > 1:
                            _, opt_lh_route = two_opt([0] + linehaul_part, dist_matrix)
                            linehaul_part = opt_lh_route[1:]
                        
                        if len(backhaul_part) > 1:
                            start_node = linehaul_part[-1] if linehaul_part else 0
                            _, opt_bh_route = two_opt([start_node] + backhaul_part, dist_matrix)
                            backhaul_part = opt_bh_route[1:]
                        
                        # 3. 배송/ 반송 나누었던 거 다시 결합
                        optimized_ant_routes.append(linehaul_part + backhaul_part)
                
                ant_distance = self.rate_solution(optimized_ant_routes, K, edges, demand_dict)
                solutions.append((optimized_ant_routes, ant_distance))

            solutions.sort(key=lambda x: x[1])
            iteration_best_solution = solutions[0]

            if global_best_solution is None or iteration_best_solution[1] < global_best_solution[1]:
                global_best_solution = iteration_best_solution
                # (τ_max, τ_min 업데이트 로직은 아래에서 처리하므로 여기서는 생략 가능)
                
            feromones = self.update_feromone_mmas(feromones, global_best_solution)

            # if global_best_solution and global_best_solution[1] != float('inf'):
            #     print(f"Iteration {i+1}: Best Distance = {global_best_solution[1]:.2f}")
            # else:
            #     print(f"Iteration {i+1}: Finding valid solution...")

            # 정체 상태를 확인하고, 필요 시 '흔들기'를 발동
            if global_best_solution and global_best_solution[1] >= last_best_cost:
                stagnation_counter += 1 # 해가 개선되지 않았으면 카운터 증가
            else:
                stagnation_counter = 0 # 해가 개선되었으면 카운터 리셋

            if stagnation_counter >= self.stagnation_limit:
                # 1. 현재 최고 해를 '흔들어서' 새로운 클러스터링을 만듦
                perturbed_routes = self._perturb_solution(global_best_solution[0], K, edges, demand_dict, capacity, dist_matrix)
                perturbed_cost = self.rate_solution(perturbed_routes, K, edges, demand_dict)

                # 2. '흔들린 해'를 기반으로 페로몬 판을 리셋함 (탐색 재시작)
                self.tau_max = 1.0 / ((1-self.ro) * perturbed_cost) if perturbed_cost != float('inf') else self.tau_max
                self.tau_min = self.tau_max / (2 * len(nodes_coord))
                feromones = {k: self.tau_max for k in feromones.keys()}
                
                # print(f"Pheromones reset based on perturbed solution (Cost: {perturbed_cost:.2f})")

                # 3. 흔들린 해가 기존 최고 해보다 좋으면 업데이트
                if perturbed_cost < global_best_solution[1]:
                    global_best_solution = (perturbed_routes, perturbed_cost)
                    # print("Perturbed solution became the new best!")

                # 4. 정체 카운터 초기화
                stagnation_counter = 0
                
        # 최종 결과 반환
        if global_best_solution:
            print("\n--- Final Solution ---")
            print(f"Final distance: {global_best_solution[1]:.2f}")
        
        return global_best_solution

    # def solve(self, K, capa, nodes_coord, demands):
    #     linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
    #     bestSolution = None
    #     dist_matrix = get_distance(nodes_coord)

    #     # 메인 루프
    #     for i in range(self.iterations):
    #         solutions = []
    #         for _ in range(self.ants):
    #             ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, nodes_coord)
    #             ant_distance = self.rate_solution(ant_solution, K, edges, demand_dict)
    #             solutions.append((ant_solution, ant_distance))

    #         bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)

    #         if bestSolution and bestSolution[1] != float('inf'):
    #             print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}, Vehicles = {len(bestSolution[0])}")
    #         else:
    #             print(f"Iteration {i+1}: Finding valid solution...")

    #     # 배송/반봉 분할해서 2-opt 적용
    #     if bestSolution and bestSolution[1] != float('inf'):
    #         print("\n--- Applying split 2-opt to the final solution... ---")

    #         aco_routes = bestSolution[0]

    #         optimized_routes = []
    #         for path in aco_routes:
    #             if not path:
    #                 optimized_routes.append([])
    #                 continue

    #             # 1. 배송 / 반송 분리
    #             linehaul_part = [node for node in path if demand_dict.get(node, 0) > 0]
    #             backhaul_part = [node for node in path if demand_dict.get(node, 0) < 0]

    #             # 2. 배송 / 반송 나누어서 2-opt 실행
    #             if len(linehaul_part) > 1:
    #                 _, opt_lh_route = two_opt([0] + linehaul_part, dist_matrix)
    #                 linehaul_part = opt_lh_route[1:] 

    #             if len(backhaul_part) > 1:
    #                 start_node = linehaul_part[-1] if linehaul_part else 0
    #                 _, opt_bh_route = two_opt([start_node] + backhaul_part, dist_matrix)
    #                 backhaul_part = opt_bh_route[1:] 

    #             # 3. 배송 / 반송 나누었던 거 다시 결합
    #             optimized_routes.append(linehaul_part + backhaul_part)

    #         # 개선된 경로로 최종 거리 재계산
    #         final_distance = self.rate_solution(optimized_routes, K, edges, demand_dict)
    #         final_solution = (optimized_routes, final_distance)

    #         return final_solution
    #     else:
    #         return bestSolution


#    def solve(self, K, capa, nodes_coord, demands):
#            """
#            [1. 클러스터링 -> 2. ACO 경로탐색 -> 3. 2-opt 후처리] 하이브리드 전략
#            """
#            # --- 데이터 준비 ---
#            linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
#            dist_matrix = get_distance(nodes_coord)
#            bestSolution = None
#
#            # --- 메인 루프 ---
#            for i in range(self.iterations):
#                solutions = []
#                for _ in range(self.ants):
#
#                    # --- 1단계 & 2단계: 선 클러스터링(CP-SAT) + 경로 순서 탐색(ACO) ---
#                    # solution_one_ant_VRPB가 클러스터링 후 ACO로 경로 순서를 찾아옴
#                    ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, nodes_coord)
#
#                    # --- 3단계: 경로 후처리 (2-opt) ---
#                    optimized_paths = []
#                    if ant_solution:
#                        for path in ant_solution:
#                            if len(path) > 1: # 노드가 2개 이상인 경로만 2-opt 수행
#                                # VRPB 규칙을 아는 2-opt 함수로 경로 다듬기
#                                _, optimized_path = two_opt([0] + path, dist_matrix, demand_dict)
#                                optimized_paths.append(optimized_path[1:]) # depot 제외하고 저장
#                            else:
#                                optimized_paths.append(path)
#
#                    # '개선된 해답'으로 거리 평가
#                    ant_distance = self.rate_solution(optimized_paths, K, edges, demand_dict)
#                    solutions.append((optimized_paths, ant_distance))
#
#                # --- 학습 단계 ---
#                bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)
#
#                if bestSolution and bestSolution[1] != float('inf'):
#                    print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}, Vehicles = {len(bestSolution[0])}")
#                else:
#                    print(f"Iteration {i+1}: Finding valid solution...")
#
#            # --- 최종 결과 출력 ---
#            print("\n--- Final Solution ---")
#            if bestSolution and bestSolution[1] != float('inf'):
#                print(f"Path: {bestSolution[0]}")
#                print(f"Total Distance: {bestSolution[1]:.2f}")
#            else:
#                print("Failed to find a valid solution with the given constraints.")
#
#            return bestSolution


#    def solve(self, K, capa, nodes_coord, demands):
#        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
#        bestSolution = None
#        dist_matrix = get_distance(nodes_coord)
#
#        # ACO 메인 루프
#        for i in range(self.iterations):
#            solutions = []
#            for _ in range(self.ants):
#                ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, nodes_coord)
#                ant_distance = self.rate_solution(ant_solution, K, edges, demand_dict)
#                solutions.append((ant_solution, ant_distance))
#            
#            bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)
#            
#            if bestSolution and bestSolution[1] != float('inf'):
#                print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}, Vehicles = {len(bestSolution[0])}")
#            else:
#                print(f"Iteration {i+1}: Finding valid solution...")
#
#        # 최종 해답에 2-opt를 5회 반복
#        if bestSolution and bestSolution[1] != float('inf'):
#            current_best_routes = bestSolution[0]
#            for i in range(5): # 5회 반복
#                optimized_routes = []
#                for path in current_best_routes:
#                    if len(path) > 1:
#                        _, optimized_path = two_opt([0] + path, dist_matrix)
#                        optimized_routes.append(optimized_path[1:])
#                    else:
#                        optimized_routes.append(path)
#                # 경로 업데이트
#                current_best_routes = optimized_routes
#
#            # 반복 끝나고 최종 거리 재계산
#            final_distance = self.rate_solution(current_best_routes, K, edges, demand_dict)
#            final_solution = (current_best_routes, final_distance)
#            
#            print(f"Final distance after 5 iterations of 2-opt: {final_distance:.2f}")
#            return final_solution
#        else:
#            return bestSolution

# def update_feromone(self, feromones, solutions, bestSolution):
#     valid_solutions = [s for s in solutions if s[1] != float('inf')]
#     if not valid_solutions:
#         feromones = {k: self.ro * v for (k, v) in feromones.items()}
#         return bestSolution, feromones

#     Lavg = sum(s[1] for s in valid_solutions) / len(valid_solutions)
#     feromones = {k: (self.ro + self.th / Lavg) * v for (k, v) in feromones.items()}
    
#     valid_solutions.sort(key=lambda x: x[1])
    
#     if bestSolution is None or bestSolution[1] == float('inf') or valid_solutions[0][1] < bestSolution[1]:
#         bestSolution = valid_solutions[0]

#     if bestSolution and bestSolution[1] != float('inf'):
#         depot = 0
#         for path in bestSolution[0]:
#             current_node = depot
#             for node in path:
#                 edge = (min(current_node, node), max(current_node, node))
#                 if edge in feromones: feromones[edge] += self.sigma / bestSolution[1]
#                 current_node = node
#             edge = (min(current_node, depot), max(current_node, depot))
#             if edge in feromones: feromones[edge] += self.sigma / bestSolution[1]
    
#     return bestSolution, feromones