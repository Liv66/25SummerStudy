import random
import numpy
from OSM_util import get_distance, two_opt

class ACO_VRPB:
    def __init__(self, alpha=1, beta=5, sigma=3, ro=0.8, th=80, iterations=200, ants=22, q0=0.9):
        self.alpha = alpha
        self.beta = beta
        self.sigma = sigma
        self.ro = ro
        self.th = th
        self.iterations = iterations
        self.ants = ants
        self.q0 = q0

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
        # 1. 초기화: K개의 차량 상태와 방문할 고객 목록 준비
        clusters = [[] for _ in range(K)]
        linehaul_loads = [0] * K
        backhaul_loads = [0] * K
        last_nodes = [0] * K  # 각 차량의 마지막 위치, 초기값은 Depot(0)

        unvisited_customers = all_linehaul + all_backhaul
        random.shuffle(unvisited_customers) # 개미마다 다른 해를 찾기 위해 순서를 섞음

        # 2. 모든 고객을 K대의 차량 중 가장 비용이 적은 곳에 삽입
        for customer in unvisited_customers:
            best_vehicle_idx = -1
            min_insertion_cost = float('inf')
            is_linehaul = demand[customer] > 0

            for i in range(K):
                # 용량 제약 조건 확인
                if is_linehaul:
                    if linehaul_loads[i] + demand[customer] > capacityLimit:
                        continue # 용량 초과
                else: # 반송
                    if backhaul_loads[i] + abs(demand[customer]) > capacityLimit:
                        continue # 용량 초과
                
                # 삽입 비용 계산 (현재 차량의 마지막 노드에서 이 고객까지의 거리)
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
                # 모든 차량이 이 고객을 받을 수 없는 경우 (용량 등), 이 개미는 실패
                return []

        # 3. K개의 그룹(클러스터)이 완성됨. 이제 각 그룹 내에서 경로 순서 결정
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
            current_node = 0 # Depot에서 시작
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

    def solve(self, K, capa, nodes_coord, demands):
            """
            [1. 클러스터링 -> 2. ACO 경로탐색 -> 3. 2-opt 후처리] 하이브리드 전략
            """
            # --- 데이터 준비 ---
            linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
            dist_matrix = get_distance(nodes_coord)
            bestSolution = None

            # --- 메인 루프 ---
            for i in range(self.iterations):
                solutions = []
                for _ in range(self.ants):

                    # --- 1단계 & 2단계: 선 클러스터링(CP-SAT) + 경로 순서 탐색(ACO) ---
                    # solution_one_ant_VRPB가 클러스터링 후 ACO로 경로 순서를 찾아옴
                    ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, nodes_coord)

                    # --- 3단계: 경로 후처리 (2-opt) ---
                    optimized_paths = []
                    if ant_solution:
                        for path in ant_solution:
                            if len(path) > 1: # 노드가 2개 이상인 경로만 2-opt 수행
                                # VRPB 규칙을 아는 2-opt 함수로 경로 다듬기
                                _, optimized_path = two_opt([0] + path, dist_matrix, demand_dict)
                                optimized_paths.append(optimized_path[1:]) # depot 제외하고 저장
                            else:
                                optimized_paths.append(path)

                    # '개선된 해답'으로 거리 평가
                    ant_distance = self.rate_solution(optimized_paths, K, edges, demand_dict)
                    solutions.append((optimized_paths, ant_distance))

                # --- 학습 단계 ---
                bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)

                if bestSolution and bestSolution[1] != float('inf'):
                    print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}, Vehicles = {len(bestSolution[0])}")
                else:
                    print(f"Iteration {i+1}: Finding valid solution...")

            # --- 최종 결과 출력 ---
            print("\n--- Final Solution ---")
            if bestSolution and bestSolution[1] != float('inf'):
                print(f"Path: {bestSolution[0]}")
                print(f"Total Distance: {bestSolution[1]:.2f}")
            else:
                print("Failed to find a valid solution with the given constraints.")

            return bestSolution