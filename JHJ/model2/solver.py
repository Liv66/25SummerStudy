import time
from construction import BaseConstructionStrategy, GreedyConstructionStrategy
from method import FirstImprovementStrategy
from Solution import CVRPBSolution
import random
import copy

class CVRPBLocalSearchSolver:
    """
    초기 해 생성(construction_strategy)과 로컬서치(local_search_strategy)를 조합해
    VRPB 문제를 해결하는 Solver 클래스
    """
    def __init__(self, construction_strategy: BaseConstructionStrategy, local_search_strategy: FirstImprovementStrategy):
        if construction_strategy is None:
            raise ValueError("construction_strategy cannot be None")
        if local_search_strategy is None:
            raise ValueError("local_search_strategy cannot be None")
        self.construction_strategy = construction_strategy
        self.local_search_strategy = local_search_strategy

    def solve(self, instance, start_time: float, time_limit: float) -> CVRPBSolution:
        """
        instance: CVRPBProblem dict 또는 객체
        반환: CVRPBSolution 객체
        """
        # 1) 초기 해 생성
        initial_routes = self.construction_strategy.build_solution(instance)
        solution = CVRPBSolution(initial_routes)
        
        # 2) 로컬서치 적용
        self.local_search_strategy.minimize(solution, start_time, time_limit)
        
        return solution

class IteratedLocalSearchSolver:
    """
    ILS 알고리즘을 사용하여 VRPB 문제를 해결합니다.
    - 초기해 생성 -> LS -> (교란 -> LS -> 수용) 반복
    """
    def __init__(self, construction_strategy: BaseConstructionStrategy, local_search_strategy: FirstImprovementStrategy):
        self.construction_strategy = construction_strategy
        self.local_search_strategy = local_search_strategy
        self.repair_strategy = GreedyConstructionStrategy() # 재창조 시 사용할 전략

    def _get_insertion_cost(self, customer: int, route, problem_info: dict) -> tuple:
        """고객을 삽입할 최적의 위치와 그 비용을 반환"""
        min_cost = float('inf')
        best_pos = None
        
        is_linehaul = problem_info['node_types'][customer] == 1
        
        if is_linehaul:
            start_idx, end_idx = 1, route.get_linehaul_count() + 1
        else:
            if route.get_linehaul_count() == 0:
                return None, float('inf')
            start_idx = route.get_linehaul_count() + 1
            end_idx = route.size() - 1

        for i in range(start_idx, end_idx + 1):
            cost = self.repair_strategy._get_insertion_cost(customer, route, i, problem_info)
            if cost < min_cost:
                min_cost = cost
                best_pos = i
        
        return best_pos, min_cost

    def _repair(self, solution: CVRPBSolution, removed_customers: list, problem_info: dict):
        """제거된 고객들을 탐욕적으로 재삽입"""
        while removed_customers:
            best_insertion = None
            min_cost = float('inf')
            customer_to_insert = -1

            for customer in removed_customers:
                for route in solution.get_routes():
                    pos, cost = self._get_insertion_cost(customer, route, problem_info)
                    if cost < min_cost:
                        min_cost = cost
                        best_insertion = (route, pos)
                        customer_to_insert = customer
            
            if best_insertion:
                route, pos = best_insertion
                route.add_customer(customer_to_insert, pos)
                removed_customers.remove(customer_to_insert)
            else:
                # 안전장치
                customer = removed_customers.pop(0)
                target_route = min(solution.get_routes(), key=lambda r: r.get_delivery_load() + r.get_pickup_load())
                target_route.force_add_customer(customer)
                print(f"Warning: Could not repair customer {customer}, forcing assignment.")
        solution.invalidate()

    def _perturb(self, solution: CVRPBSolution, problem_info: dict, strength: float) -> CVRPBSolution:
        """
        Destroy and Repair 방식의 교란 함수
        - strength: 파괴할 고객의 비율 (0.0 ~ 1.0)
        """
        new_solution = copy.deepcopy(solution)
        
        # 1. Destroy: 고객 제거
        all_customers = new_solution.get_customers()
        num_to_remove = int(len(all_customers) * strength)
        removed_customers = random.sample(all_customers, num_to_remove)

        # 경로에서 고객 제거 (뒤에서부터 제거해야 인덱스 불일치 방지)
        for r in new_solution.get_routes():
            customers_in_route = r.get_customers()
            indices_to_remove = []
            for i, c in enumerate(customers_in_route):
                if c in removed_customers:
                    # 실제 인덱스는 depot(0) 때문에 +1
                    indices_to_remove.append(i + 1)
            
            for index in sorted(indices_to_remove, reverse=True):
                r.remove_customer(index)
        
        new_solution.invalidate()

        # 2. Repair: 똑똑하게 재삽입
        self._repair(new_solution, removed_customers, problem_info)
        
        return new_solution

    def solve(self, instance, start_time: float, time_limit: float) -> CVRPBSolution:
        # 1. 초기해 생성 및 Local Search
        initial_routes = self.construction_strategy.build_solution(instance)
        current_solution = CVRPBSolution(initial_routes)
        self.local_search_strategy.minimize(current_solution, start_time, time_limit)
        
        best_solution = copy.deepcopy(current_solution)
        
        iteration = 0
        iterations_without_improvement = 0
        max_iters_without_improvement = 20 # 정체 감지 임계값

        while time.time() - start_time < time_limit:
            if iterations_without_improvement >= max_iters_without_improvement:
                print(f"Stopping ILS after {max_iters_without_improvement} iterations without improvement.")
                break
            
            iteration += 1
            best_cost = best_solution.get_total_cost()
            print(f"--- ILS Iteration {iteration}, Best Cost: {best_cost:.2f} ---")
            
            # 2. 교란 (Perturbation)
            strength = 0.2 + (iteration * 0.01) if iteration < 20 else 0.4
            perturbed_solution = self._perturb(best_solution, instance, strength)
            
            # 3. 지역 탐색 (Local Search)
            self.local_search_strategy.minimize(perturbed_solution, start_time, time_limit)
            
            # 4. 지능적인 수용 기준 (Smart Acceptance Criterion)
            # Feasibility는 페널티 값보다 비용이 낮은지로 간단히 판별
            new_cost = perturbed_solution.get_total_cost()
            is_new_feasible = new_cost < perturbed_solution.penalty_rate
            is_best_feasible = best_cost < best_solution.penalty_rate

            accepted = False
            if is_new_feasible and not is_best_feasible:
                accepted = True # 1순위: Feasible은 Infeasible을 항상 이긴다
            elif is_new_feasible and is_best_feasible:
                if new_cost < best_cost:
                    accepted = True # 2순위: 둘 다 Feasible하면 더 좋은 비용을 선택
            elif not is_new_feasible and not is_best_feasible:
                if new_cost < best_cost:
                    accepted = True # 둘 다 Infeasible하면, 그나마 나은 것을 선택
            
            if accepted:
                best_solution = copy.deepcopy(perturbed_solution)
                iterations_without_improvement = 0
                print(f"*** New best solution found! Cost: {best_solution.get_total_cost():.2f} (Feasible: {is_new_feasible}) ***")
            else:
                iterations_without_improvement += 1

        return best_solution
