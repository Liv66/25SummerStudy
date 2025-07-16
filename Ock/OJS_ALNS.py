import random
import math
from OJS_heuristic import destroy_solution, repair_solution # heuristic.py 파일에서 클래스 가져오기
from OJS_heuristic import calculate_total_cost # heuristic.py 파일에서 함수 가져오기

class ALNS:
    def __init__(self, initial_solution, nodes, destroyer, repairer, **params):
        # 기본 정보
        self.nodes = nodes
        self.params = params

        # 해 정보
        self.current_solution = initial_solution
        self.best_solution = initial_solution
        self.best_cost = calculate_total_cost(nodes, initial_solution)
        
        # 파괴 및 재구성 휴리스틱 목록
        self.destroy_methods = [
            destroyer.random_removal,
            destroyer.worst_removal,
            destroyer.route_removal
        ]
        self.repair_methods = [
            # 참고: 구현하신 greedy/regret 휴리스틱은 현재 Linehaul만 처리하므로,
            # Backhaul까지 완성하거나, random_insertion만 우선 사용할 수 있습니다.
            # 여기서는 3개 모두 포함하여 구조를 보여드립니다.
            repairer.greedy_insertion,
            repairer.regret_insertion,
            repairer.random_insertion

        ]
        
        # 휴리스틱 가중치 및 점수 초기화
        self.destroy_weights = [1.0] * len(self.destroy_methods)
        self.repair_weights = [1.0] * len(self.repair_methods)
        self.destroy_scores = [0] * len(self.destroy_methods)
        self.repair_scores = [0] * len(self.repair_methods)

    def select_heuristic(self, methods, weights):
        """룰렛 휠 방식으로 가중치에 따라 휴리스틱 선택"""
        total_weight = sum(weights)
        pick = random.uniform(0, total_weight)
        current = 0
        for i, weight in enumerate(weights):
            current += weight
            if current > pick:
                return i, methods[i]
        return len(methods) - 1, methods[-1]

    def update_weights(self, reaction_factor=0.1):
        """성과에 따라 가중치 업데이트"""
        for i in range(len(self.destroy_weights)):
            self.destroy_weights[i] = (1 - reaction_factor) * self.destroy_weights[i] + reaction_factor * self.destroy_scores[i]
        for i in range(len(self.repair_weights)):
            self.repair_weights[i] = (1 - reaction_factor) * self.repair_weights[i] + reaction_factor * self.repair_scores[i]
        
        # 점수는 다음 세그먼트를 위해 초기화
        self.destroy_scores = [0] * len(self.destroy_methods)
        self.repair_scores = [0] * len(self.repair_methods)

    def run(self, iterations, temperature, cooling_rate):
        print(f"ALNS 시작. 초기 비용: {self.best_cost:.2f}")
        
        for i in range(iterations):
            # 1. 파괴 및 재구성 휴리스틱 선택
            destroy_idx, destroy_method = self.select_heuristic(self.destroy_methods, self.destroy_weights)
            repair_idx, repair_method = self.select_heuristic(self.repair_methods, self.repair_weights)
            
            # 2. 새로운 해 생성
            # num_to_remove 등 파라미터는 필요에 따라 조절 가능
            partial_solution, removed = destroy_method(self.current_solution, num_to_remove=5) 
            new_solution = repair_method(partial_solution, removed)
            new_cost = calculate_total_cost(self.nodes, new_solution)

            # 3. 해 채택 결정 (Simulated Annealing 방식)
            current_cost = calculate_total_cost(self.nodes, self.current_solution)
            score = 0
            if new_cost < self.best_cost:
                # 전역 최적해 갱신: 가장 높은 점수
                self.best_solution = new_solution
                self.best_cost = new_cost
                self.current_solution = new_solution
                score = 3
                # print(f"Iteration {i+1}: 🏆 새로운 최적해 발견! 비용: {self.best_cost:.2f}")
            elif new_cost < current_cost:
                # 현재 해보다 좋은 해: 중간 점수
                self.current_solution = new_solution
                score = 2
            elif temperature > 0 and math.exp((current_cost - new_cost) / temperature) > random.random():
                # 더 나쁜 해지만 확률적으로 채택: 낮은 점수
                self.current_solution = new_solution
                score = 1

            # 4. 휴리스틱 성과 기록
            if score > 0:
                self.destroy_scores[destroy_idx] += score
                self.repair_scores[repair_idx] += score

            # 5. 가중치 업데이트 (매 100번 반복마다)
            if (i + 1) % 100 == 0:
                self.update_weights()
                # print(f"Iteration {i+1}: 가중치 업데이트")

            # 6. 온도 감소
            temperature *= cooling_rate
            
        print("\n--- ALNS 종료 ---")
        return self.best_solution, self.best_cost