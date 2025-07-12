import random
import numpy
from functools import reduce

class ACO_VRPB:

    def __init__(self, alpha=2, beta=5, sigma=3, ro=0.8, th=80, iterations=1000, ants=22):
        self.alpha = alpha
        self.beta = beta
        self.sigma = sigma
        self.ro = ro
        self.th = th
        self.iterations = iterations
        self.ants = ants
        """
        alpha 페로몬 영향력: 클수록 많이 다니는 길을 선택할 확률이 높아짐
        beta 거리 영향력: 클수록 가까운 길을 선택할 확률이 높아짐 
        sigma 경로 강화할 상위 개미의 수
        ro = 페로몬 증발 계수
        th = 페로몬 업데이트 계산에 사용되는 추가 변수
        iterations 세대 수: 알고리즘 반복 횟수
        ants 개미 수    
        """

    def initialize_aco_data(self, capa, nodes_coord, demands): # N, capa, nodes_coord, demands를 직접 파라미터로 받음
        # 수요: 양수(+) linehaul, 음수(-) backhaul
        
        # 노드 ID를 키로 하는 graph와 demand 딕셔너리 생성 (0번 노드는 차고지)
        graph = {i: coord for i, coord in enumerate(nodes_coord)}   # nodes_coord 리스트를 그래프로 만들어줌
        demand_dict = {i: d for i, d in enumerate(demands)} # 수요량을 딕셔너리로 저장

        # 수요를 기반으로 노드 분류
        all_nodes = list(graph.keys())
        depot = 0   # 0번 노드를 차고지로 가정

        linehaul_nodes = [node for node in all_nodes if demand_dict[node] > 0]
        backhaul_nodes = [node for node in all_nodes if demand_dict[node] < 0]

        print(f"Depot: {depot}")
        print(f"Linehaul nodes {len(linehaul_nodes)}: {linehaul_nodes}")
        print(f"Backhaul nodes {len(backhaul_nodes)}: {backhaul_nodes}")

        # 모든 노드 간의 거리(edges) 계산
        edges = {(min(a,b), max(a,b)): numpy.sqrt((graph[a][0]-graph[b][0])**2 + (graph[a][1]-graph[b][1])**2) 
                 for a in all_nodes for b in all_nodes if a != b}

        # 페로몬 초기화
        feromones = {k: 1.0 for k in edges.keys()}

        # 분류된 노드 리스트를 반환
        return linehaul_nodes, backhaul_nodes, edges, capa, demand_dict, feromones
    
    def _calculate_probabilities(self, current_node, candidate_nodes, feromones, edges):
        """ 현재 노드에서 후보 노드로 이동할 확률 계산 """
        probs = []
        for node in candidate_nodes:
            # 에러 방지를 위해 .get() 사용
            edge = (min(current_node, node), max(current_node, node))
            prob = (feromones.get(edge, 1.0) ** self.alpha) * \
                   ((1 / edges.get(edge, float('inf'))) ** self.beta)
            probs.append(prob)

        sum_probs = numpy.sum(probs)
        if sum_probs == 0:
            # 모든 확률이 0이면, 균등 확률을 반환하여 멈추는 것을 방지
            return numpy.ones(len(candidate_nodes)) / len(candidate_nodes) if candidate_nodes else numpy.array([])

        return numpy.array(probs) / sum_probs

    def solutionOfOneAnt_VRPB(self, all_linehaul, all_backhaul, edges, capacityLimit, demand, feromones):
        solution = list()

        # 방문해야 할 노드 목록 복사
        unvisited_linehaul = all_linehaul.copy()
        unvisited_backhaul = all_backhaul.copy()

        while unvisited_linehaul or unvisited_backhaul: # 방문할 노드가 남아있는 동안 반복

            # 배송(Linehaul) 경로
            path = []
            current_linehaul_demand = 0

            # 제약조건6: 각 경로에는 적어도 1개 이상의 배송 노드가 있어야 함
            if not unvisited_linehaul: break # 더 이상 배송할 노드가 없으면 종료

            # 첫 배송 노드 선택 (랜덤)
            current_node = numpy.random.choice(unvisited_linehaul)

            # 경로 시작
            path.append(current_node)
            current_linehaul_demand += demand[current_node]
            unvisited_linehaul.remove(current_node)

            # 추가 배송 노드 선택
            while unvisited_linehaul:
                # 확률 계산 (오직 미방문 배송 노드만을 대상으로)
                probabilities = self._calculate_probabilities(current_node, unvisited_linehaul, feromones, edges)
                next_node = numpy.random.choice(unvisited_linehaul, p=probabilities)

                # 용량 제약 조건 (v) 확인
                if current_linehaul_demand + demand[next_node] <= capacityLimit:
                    path.append(next_node)
                    current_linehaul_demand += demand[next_node]
                    unvisited_linehaul.remove(next_node)
                    current_node = next_node
                else:
                    break # 용량 초과 시 배송 단계 종료

            # === 2단계: 반송(Backhaul) 경로 생성 ===
            current_backhaul_demand = 0

            while unvisited_backhaul:
                # 확률 계산 (오직 미방문 반송 노드만을 대상으로, 현재 위치는 마지막 배송 노드)
                probabilities = self._calculate_probabilities(current_node, unvisited_backhaul, feromones, edges)
                next_node = numpy.random.choice(unvisited_backhaul, p=probabilities)

                # 용량 제약 조건 (v) 확인 (반송 수요는 보통 절대값으로 계산)
                if current_backhaul_demand + abs(demand[next_node]) <= capacityLimit:
                    path.append(next_node)
                    current_backhaul_demand += abs(demand[next_node])
                    unvisited_backhaul.remove(next_node)
                    current_node = next_node
                else:
                    break # 용량 초과 시 반송 단계 종료
                
            solution.append(path) # 완성된 한 차량의 경로를 전체 해답에 추가

        return solution
    
    def rateSolution(self, solution, edges):
        total_distance = 0
        depot = 0
        for path in solution:
            current_node = depot
            for node in path:
                total_distance += edges.get((min(current_node, node), max(current_node, node)), 0)
                current_node = node
            total_distance += edges.get((min(current_node, depot), max(current_node, depot)), 0)
        return total_distance

    def updateFeromone(self, feromones, solutions, bestSolution):
        Lavg = reduce(lambda x, y: x + y, (i[1] for i in solutions)) / len(solutions)
        feromones = {k: (self.ro + self.th / Lavg) * v for (k, v) in feromones.items()}
        solutions.sort(key=lambda x: x[1])
        
        if bestSolution is None or solutions[0][1] < bestSolution[1]:
            bestSolution = solutions[0]

        # 최고 해답 경로에 페로몬 추가
        for path in bestSolution[0]:
            current_node = 0
            for node in path:
                edge = (min(current_node, node), max(current_node, node))
                if edge in feromones:
                    feromones[edge] += self.sigma / bestSolution[1]
                current_node = node
            edge = (min(current_node, 0), max(current_node, 0))
            if edge in feromones:
                feromones[edge] += self.sigma / bestSolution[1]

        return bestSolution, feromones

    def solve(self, N, capa, nodes_coord, demands):
        """ VRPB 문제를 해결하는 메인 컨트롤러 """
        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
        
        bestSolution = None

        for i in range(self.iterations):
            solutions = []
            for _ in range(self.ants):
                # 개미가 경로를 생성할 때마다 원본 노드 리스트를 전달해야 함
                ant_solution = self.solutionOfOneAnt_VRPB(linehaul, backhaul, edges, capacity, demand_dict, feromones)
                ant_distance = self.rateSolution(ant_solution, edges)
                solutions.append((ant_solution, ant_distance))
            
            bestSolution, feromones = self.updateFeromone(feromones, solutions, bestSolution)
            
            if bestSolution:
                print(f"Iteration {i+1}: Best Distance = {bestSolution[1]:.2f}")

        print("\n--- Final Solution ---")
        print(f"Path: {bestSolution[0]}")
        print(f"Total Distance: {bestSolution[1]:.2f}")
        return bestSolution
    
if __name__ == "__main__":
    # 1. 문제 데이터 생성
    N = 20
    capa = 1000
    nodes_coord = [(random.uniform(0, 100), random.uniform(0, 100)) for _ in range(N + 1)]
    demands = [0] + [random.randint(50, 100) for _ in range(N // 2)] + [random.randint(-100, -50) for _ in range(N - N // 2)]
    random.shuffle(demands[1:])

    # 2. ACO_VRPB 객체 생성 및 문제 해결
    aco_solver = ACO_VRPB(iterations=100, ants=20)
    final_solution = aco_solver.solve(N + 1, capa, nodes_coord, demands)