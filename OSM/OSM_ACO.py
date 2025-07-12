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
        alpha ��θ� �����: Ŭ���� ���� �ٴϴ� ���� ������ Ȯ���� ������
        beta �Ÿ� �����: Ŭ���� ����� ���� ������ Ȯ���� ������ 
        sigma ��� ��ȭ�� ���� ������ ��
        ro = ��θ� ���� ���
        th = ��θ� ������Ʈ ��꿡 ���Ǵ� �߰� ����
        iterations ���� ��: �˰��� �ݺ� Ƚ��
        ants ���� ��    
        """

    def initialize_aco_data(self, capa, nodes_coord, demands): # N, capa, nodes_coord, demands�� ���� �Ķ���ͷ� ����
        # ����: ���(+) linehaul, ����(-) backhaul
        
        # ��� ID�� Ű�� �ϴ� graph�� demand ��ųʸ� ���� (0�� ���� ������)
        graph = {i: coord for i, coord in enumerate(nodes_coord)}   # nodes_coord ����Ʈ�� �׷����� �������
        demand_dict = {i: d for i, d in enumerate(demands)} # ���䷮�� ��ųʸ��� ����

        # ���並 ������� ��� �з�
        all_nodes = list(graph.keys())
        depot = 0   # 0�� ��带 �������� ����

        linehaul_nodes = [node for node in all_nodes if demand_dict[node] > 0]
        backhaul_nodes = [node for node in all_nodes if demand_dict[node] < 0]

        print(f"Depot: {depot}")
        print(f"Linehaul nodes {len(linehaul_nodes)}: {linehaul_nodes}")
        print(f"Backhaul nodes {len(backhaul_nodes)}: {backhaul_nodes}")

        # ��� ��� ���� �Ÿ�(edges) ���
        edges = {(min(a,b), max(a,b)): numpy.sqrt((graph[a][0]-graph[b][0])**2 + (graph[a][1]-graph[b][1])**2) 
                 for a in all_nodes for b in all_nodes if a != b}

        # ��θ� �ʱ�ȭ
        feromones = {k: 1.0 for k in edges.keys()}

        # �з��� ��� ����Ʈ�� ��ȯ
        return linehaul_nodes, backhaul_nodes, edges, capa, demand_dict, feromones
    
    def _calculate_probabilities(self, current_node, candidate_nodes, feromones, edges):
        """ ���� ��忡�� �ĺ� ���� �̵��� Ȯ�� ��� """
        probs = []
        for node in candidate_nodes:
            # ���� ������ ���� .get() ���
            edge = (min(current_node, node), max(current_node, node))
            prob = (feromones.get(edge, 1.0) ** self.alpha) * \
                   ((1 / edges.get(edge, float('inf'))) ** self.beta)
            probs.append(prob)

        sum_probs = numpy.sum(probs)
        if sum_probs == 0:
            # ��� Ȯ���� 0�̸�, �յ� Ȯ���� ��ȯ�Ͽ� ���ߴ� ���� ����
            return numpy.ones(len(candidate_nodes)) / len(candidate_nodes) if candidate_nodes else numpy.array([])

        return numpy.array(probs) / sum_probs

    def solutionOfOneAnt_VRPB(self, all_linehaul, all_backhaul, edges, capacityLimit, demand, feromones):
        solution = list()

        # �湮�ؾ� �� ��� ��� ����
        unvisited_linehaul = all_linehaul.copy()
        unvisited_backhaul = all_backhaul.copy()

        while unvisited_linehaul or unvisited_backhaul: # �湮�� ��尡 �����ִ� ���� �ݺ�

            # ���(Linehaul) ���
            path = []
            current_linehaul_demand = 0

            # ��������6: �� ��ο��� ��� 1�� �̻��� ��� ��尡 �־�� ��
            if not unvisited_linehaul: break # �� �̻� ����� ��尡 ������ ����

            # ù ��� ��� ���� (����)
            current_node = numpy.random.choice(unvisited_linehaul)

            # ��� ����
            path.append(current_node)
            current_linehaul_demand += demand[current_node]
            unvisited_linehaul.remove(current_node)

            # �߰� ��� ��� ����
            while unvisited_linehaul:
                # Ȯ�� ��� (���� �̹湮 ��� ��常�� �������)
                probabilities = self._calculate_probabilities(current_node, unvisited_linehaul, feromones, edges)
                next_node = numpy.random.choice(unvisited_linehaul, p=probabilities)

                # �뷮 ���� ���� (v) Ȯ��
                if current_linehaul_demand + demand[next_node] <= capacityLimit:
                    path.append(next_node)
                    current_linehaul_demand += demand[next_node]
                    unvisited_linehaul.remove(next_node)
                    current_node = next_node
                else:
                    break # �뷮 �ʰ� �� ��� �ܰ� ����

            # �ݼ�(Backhaul) ���
            current_backhaul_demand = 0

            while unvisited_backhaul:
                # Ȯ�� ��� (�̹湮 �ݼ� ��� �������, ���� ��ġ�� ������ ��� ���)
                probabilities = self._calculate_probabilities(current_node, unvisited_backhaul, feromones, edges)
                next_node = numpy.random.choice(unvisited_backhaul, p=probabilities)

                # 5. �뷮 ���� ���� Ȯ�� (�ݼ� ����� ���� ���밪���� ���)
                if current_backhaul_demand + abs(demand[next_node]) <= capacityLimit:
                    path.append(next_node)
                    current_backhaul_demand += abs(demand[next_node])
                    unvisited_backhaul.remove(next_node)
                    current_node = next_node
                else:
                    break # �뷮 �ʰ� �� �ݼ� �ܰ� ����
                
            solution.append(path) # �ϼ��� �� ������ ��θ� ��ü �ش信 �߰�

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

        # �ְ� �ش� ��ο� ��θ� �߰�
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
        """ VRPB ������ �ذ��ϴ� ���� ��Ʈ�ѷ� """
        linehaul, backhaul, edges, capacity, demand_dict, feromones = self.initialize_aco_data(capa, nodes_coord, demands)
        
        bestSolution = None

        for i in range(self.iterations):
            solutions = []
            for _ in range(self.ants):
                # ���̰� ��θ� ������ ������ ���� ��� ����Ʈ�� �����ؾ� ��
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
    # 1. ���� ������ ����
    N = 20
    capa = 1000
    nodes_coord = [(random.uniform(0, 100), random.uniform(0, 100)) for _ in range(N + 1)]
    demands = [0] + [random.randint(50, 100) for _ in range(N // 2)] + [random.randint(-100, -50) for _ in range(N - N // 2)]
    random.shuffle(demands[1:])

    # 2. ACO_VRPB ��ü ���� �� ���� �ذ�
    aco_solver = ACO_VRPB(iterations=100, ants=20)
    final_solution = aco_solver.solve(N + 1, capa, nodes_coord, demands)