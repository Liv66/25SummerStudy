import random
import numpy
from OSM_util import get_distance, two_opt

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

    def initialize_aco_data(self, capa, node_coords, demands, dist_mat):
        graph = {i: coord for i, coord in enumerate(node_coords)}
        demand_dict = {i: d for i, d in enumerate(demands)}
        all_nodes = list(graph.keys())
        
        linehaul_nodes = [node for node in all_nodes if demand_dict.get(node, 0) > 0]
        backhaul_nodes = [node for node in all_nodes if demand_dict.get(node, 0) < 0]

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

    def solution_one_ant_VRPB(self, K, all_linehaul, all_backhaul, edges, capacityLimit, demand, feromones, node_coords):
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


    def solve(self, K, capa, node_coords, demands, dist_mat):
        # 초기화 (기존과 동일)
        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, node_coords, demands, dist_mat)
        bestSolution = None
        dist_matrix = dist_mat

        # 메인 루프
        for i in range(self.iterations):
            solutions = []
            for _ in range(self.ants):
                ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, node_coords)
                ant_distance = self.rate_solution(ant_solution, K, edges, demand_dict)
                solutions.append((ant_solution, ant_distance))

            # 페로몬 업데이트 및 현재까지의 최적해 선정
            bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)

            # 2-opt 최적화를 메인 루프 안으로 이동
            if bestSolution and bestSolution[1] != float('inf'):
                aco_routes = bestSolution[0]
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

                # 2-opt 적용 후 성능이 개선되었는지 확인하고 최적해 업데이트
                optimized_distance = self.rate_solution(optimized_routes, K, edges, demand_dict)
                if optimized_distance < bestSolution[1]:
                    bestSolution = (optimized_routes, optimized_distance)
                    print(f"Iteration {i+1}: 2-opt improved distance to {bestSolution[1]:.2f}")

            # 반복마다 현재 최적 거리 출력
            if bestSolution and bestSolution[1] != float('inf'):
                print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}, Vehicles = {len(bestSolution[0])}")
            else:
                print(f"Iteration {i+1}: Finding valid solution...")

        # ‼️ 3. 루프 종료 후, 최종적으로 찾은 bestSolution을 형식에 맞게 반환
        if bestSolution:
            routes, cost = bestSolution
            routes_with_depot = [[0] + r + [0] if r else [] for r in routes]
            return (routes_with_depot, cost)
        else:
            return bestSolution


"""
    def solve(self, K, capa, node_coords, demands, dist_mat):
        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, node_coords, demands, dist_mat)
        bestSolution = None
        dist_matrix = dist_mat

        # 메인 루프
        for i in range(self.iterations):
            solutions = []
            for _ in range(self.ants):
                ant_solution = self.solution_one_ant_VRPB(K, linehaul, backhaul, edges, capacity, demand_dict, feromones, node_coords)
                ant_distance = self.rate_solution(ant_solution, K, edges, demand_dict)
                solutions.append((ant_solution, ant_distance))

            bestSolution, feromones = self.update_feromone(feromones, solutions, bestSolution)

            if bestSolution and bestSolution[1] != float('inf'):
                print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}, Vehicles = {len(bestSolution[0])}")
            else:
                print(f"Iteration {i+1}: Finding valid solution...")

        # 배송/반봉 분할해서 2-opt 적용
        if bestSolution and bestSolution[1] != float('inf'):
            print("\n--- Applying split 2-opt to the final solution... ---")

            aco_routes = bestSolution[0]

            optimized_routes = []
            for path in aco_routes:
                if not path:
                    optimized_routes.append([])
                    continue

                # 1. 배송 / 반송 분리
                linehaul_part = [node for node in path if demand_dict.get(node, 0) > 0]
                backhaul_part = [node for node in path if demand_dict.get(node, 0) < 0]

                # 2. 배송 / 반송 나누어서 2-opt 실행
                if len(linehaul_part) > 1:
                    _, opt_lh_route = two_opt([0] + linehaul_part, dist_matrix)
                    linehaul_part = opt_lh_route[1:] 

                if len(backhaul_part) > 1:
                    start_node = linehaul_part[-1] if linehaul_part else 0
                    _, opt_bh_route = two_opt([start_node] + backhaul_part, dist_matrix)
                    backhaul_part = opt_bh_route[1:] 

                # 3. 배송 / 반송 나누었던 거 다시 결합
                optimized_routes.append(linehaul_part + backhaul_part)

            # 개선된 경로로 최종 거리 재계산
            final_distance = self.rate_solution(optimized_routes, K, edges, demand_dict)
            routes_with_depot = [[0] + route + [0] if route else [] for route in optimized_routes]
            final_solution = (routes_with_depot, final_distance)

            return final_solution
        else:
            if bestSolution:
                routes, cost = bestSolution
                routes_with_depot = [[0] + r + [0] if r else [] for r in routes]
                return (routes_with_depot, cost)
            else:
                return bestSolution
"""