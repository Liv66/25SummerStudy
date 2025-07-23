import random
import math
from Ock_heuristic import destroy_solution, repair_solution # heuristic.py 파일에서 클래스 가져오기
from Ock_heuristic import calculate_total_cost # heuristic.py 파일에서 함수 가져오기
import sys

class ALNS:
    def __init__(self, initial_solution, nodes, destroyer, repairer, Cost_matrix , **params):
        # 기본 정보
        self.nodes = nodes
        self.params = params
        self.Cost_matrix = Cost_matrix
        # 해 정보
        self.current_solution = initial_solution
        self.best_solution = initial_solution
        self.best_cost = calculate_total_cost(initial_solution, Cost_matrix)
        
        # 파괴 및 재구성 휴리스틱 목록
        self.destroy_methods = [
            destroyer.random_removal,
            destroyer.worst_removal,
            destroyer.route_removal
        ]
        self.repair_methods = [
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

    def run(self, iterations, temperature, cooling_rate, max_no_improvement=500):
        # print(f"ALNS 시작. 초기 비용: {self.best_cost:.2f}")
        no_improve = 0
        fail_repair = 0
        for i in range(iterations):
            # 1. 파괴 및 재구성 휴리스틱 선택
            destroy_idx, destroy_method = self.select_heuristic(self.destroy_methods, self.destroy_weights)
            repair_idx, repair_method = self.select_heuristic(self.repair_methods, self.repair_weights)
            
            # 2. 새로운 해 생성
            # num_to_remove 등 파라미터는 필요에 따라 조절 가능
            num_to_remove = random.randint(1, len(self.nodes)/10) 
            partial_solution, removed = destroy_method(self.current_solution, num_to_remove=num_to_remove) 
            new_solution = repair_method(partial_solution, removed)
            # print('경로 포함된 노드 수',sum(1 for route in new_solution for customer in route if customer != 0))
            # print('실제 노드 수', len(self.nodes) - 1)  
            if sum(1 for route in new_solution for customer in route if customer != 0) != len(self.nodes) - 1:
                fail_repair += 1
                # print(f"사용한 repareer: {repair_method.__name__}")
                # print(f"사용한 destroyer: {destroy_method.__name__}")
                # print("partial",partial_solution)
                # print("removed",removed)
                # print("new",new_solution)
                continue

            new_cost = calculate_total_cost(new_solution, self.Cost_matrix)

            # 3. 해 채택 결정 (Simulated Annealing 방식)
            current_cost = calculate_total_cost(self.current_solution, self.Cost_matrix)
            score = 0

            new_best_found = False
            if new_cost < self.best_cost:
                # 전역 최적해 갱신: 가장 높은 점수
                self.best_solution = new_solution
                self.best_cost = new_cost
                self.current_solution = new_solution
                score = 3
                new_best_found = True
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


            ### --- 조기 종료 로직 --- ###
            if new_best_found:
                no_improve = 0 # 최적해가 개선되면 카운터 초기화
            else:
                no_improve += 1 # 개선되지 않으면 카운터 증가

            # 카운터가 지정된 한계를 넘으면 루프 중단
            if no_improve >= max_no_improvement:
                print(f"\nIteration {i+1}: {max_no_improvement}번 반복 동안 해가 개선되지 않아 조기 종료합니다.")
                break

            # 5. 가중치 업데이트 (매 100번 반복마다)
            if (i + 1) % 100 == 0:
                self.update_weights()
                # print(f"Iteration {i+1}: 가중치 업데이트")

            # 6. 온도 감소
            temperature *= cooling_rate
            
        print("\n--- ALNS 종료 ---")
        print(f"총 실패 횟수 {fail_repair}, 총 실패 비율 {fail_repair / iterations * 100:.2f}%")
        return self.best_solution, self.best_cost