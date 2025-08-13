import time
from construction import GreedyConstructionStrategy
from method import FirstImprovementStrategy
from Solution import CVRPBSolution
import random
import copy

class IteratedLocalSearchSolver:

    def __init__(self, construction_strategy: GreedyConstructionStrategy, local_search_strategy: FirstImprovementStrategy):
        self.construction_strategy = construction_strategy
        self.local_search_strategy = local_search_strategy

    def solve(self, instance, start_time, time_limit) -> CVRPBSolution:
        # 1. 초기해 생성
        current_solution = CVRPBSolution(self.construction_strategy.build_solution(instance))

        # 2. 초기 Local Search
        initial_ls_improvement_count = 0
        initial_ls_improvement_limit = 200

        while time.time() - start_time < time_limit:
            if initial_ls_improvement_count >= initial_ls_improvement_limit:
                print(f"Initial LS reached improvement limit of {initial_ls_improvement_limit}.")
                break

            was_improved = self.local_search_strategy.minimize(current_solution, start_time, time_limit)
            if not was_improved:
                break
            initial_ls_improvement_count += 1

        best_solution = copy.deepcopy(current_solution)


        iteration = 0
        iterations_without_improvement = 0
        max_iters_without_improvement = 10000

        # 3. ILS 메인
        while time.time() - start_time < time_limit:
            if iterations_without_improvement >= max_iters_without_improvement:
                print(f"Stopping ILS after {max_iters_without_improvement} iterations without improvement.")
                break

            iteration += 1
            elapsed_time = time.time() - start_time
            remaining_time = time_limit - elapsed_time
            best_cost = best_solution.get_total_cost()
            is_best_feasible = best_solution.get_penalty_cost(best_solution.penalty_rate) < 1e-9
            print(f"--- ILS Iteration {iteration}, Best Cost: {best_cost:.2f} (Feasible: {is_best_feasible}), Time: {elapsed_time:.1f}s/{time_limit}s ---")

            # 4. 교란 (Perturbation)
            strength = 0.3 + (iteration * 0.01) if iteration <= 50 else 0.7
            perturbed_solution = self._perturb(best_solution, instance, strength)

            # 5. 교란된 해에 대한 Local Search
            ls_improvements = 0
            max_ls_improvements = 250  # ILS 내 Local Search 개선 횟수 제한
            while time.time() - start_time < time_limit and ls_improvements < max_ls_improvements:
                was_improved = self.local_search_strategy.minimize(perturbed_solution, start_time, time_limit)
                if not was_improved:
                    break
                ls_improvements += 1
                #
            if ls_improvements >= max_ls_improvements:
                print(f"    LS reached improvement limit: {ls_improvements}")
            else:
                print(f"    LS improvements: {ls_improvements}")

            # 6. 수용
            new_cost = perturbed_solution.get_total_cost()
            is_new_feasible = perturbed_solution.get_penalty_cost(perturbed_solution.penalty_rate) < 1e-9

            accepted = False
            if is_new_feasible and not is_best_feasible:
                accepted = True
            elif is_new_feasible and is_best_feasible:
                if new_cost < best_cost:
                    accepted = True
            elif not is_new_feasible and not is_best_feasible:
                if new_cost < best_cost:
                    accepted = True

            if accepted:
                best_solution = copy.deepcopy(perturbed_solution)
                iterations_without_improvement = 0
                print(f"*** New best solution found! Cost: {best_solution.get_total_cost():.2f} (Feasible: {is_new_feasible}) ***")
            else:
                iterations_without_improvement += 1
                print(f"    Solution not accepted. No improvement count: {iterations_without_improvement}")

        # ILS 종료 원인 출력
        final_elapsed_time = time.time() - start_time
        if final_elapsed_time >= time_limit:
            print(f"ILS terminated due to time limit. Total time: {final_elapsed_time:.1f}s")
        elif iterations_without_improvement >= max_iters_without_improvement:
            print(f"ILS terminated due to no improvement for {max_iters_without_improvement} iterations.")
        else:
            print(f"ILS terminated for unknown reason. Time: {final_elapsed_time:.1f}s, No improvements: {iterations_without_improvement}")

        return best_solution

    def _perturb(self, solution: CVRPBSolution, problem_info: dict, strength: float) -> CVRPBSolution:
        new_solution = copy.deepcopy(solution)
        all_customers = new_solution.get_customers()
        num_to_remove = int(len(all_customers) * strength)
        if not all_customers or num_to_remove == 0:
            return new_solution

        customers_to_remove = random.sample(all_customers, num_to_remove)

        for route in new_solution.get_routes():
            for customer in list(route.get_customers()):
                if customer in customers_to_remove:
                    route.remove_customer_by_id(customer)
                    customers_to_remove.remove(customer)

        new_solution.invalidate()

        removed_customers = list(set(all_customers) - set(new_solution.get_customers()))
        if removed_customers:
            self._repair(new_solution, removed_customers, problem_info)
        return new_solution

    def _repair(self, solution: CVRPBSolution, customers_to_add: list, problem_info: dict):
        random.shuffle(customers_to_add)
        # customer 하나하나 복구
        for customer in customers_to_add:
            min_cost_increase = float('inf')
            best_route = None
            best_index = -1

            for route in solution.get_routes():
                is_linehaul = problem_info['node_types'][customer] == 1
                start_idx, end_idx = 0, 0
                if is_linehaul:
                    start_idx, end_idx = 1, route.get_linehaul_count() + 1
                else:
                    if route.get_linehaul_count() == 0: continue
                    start_idx = route.get_linehaul_count() + 1
                    end_idx = route.size()
                # 비용이 적게 증가하는 위치에 다시 삽입
                for i in range(start_idx, end_idx + 1):
                    cost_increase = route.get_cost_increase_for_insertion(customer, i, solution.penalty_rate)
                    if cost_increase < min_cost_increase:
                        min_cost_increase = cost_increase
                        best_route = route
                        best_index = i

            if best_route:
                best_route.add_customer(customer, best_index)
            else:
                target_route = min(solution.get_routes(), key=lambda r: r.get_delivery_load() + r.get_pickup_load())
                target_route.force_add_customer(customer)
        solution.invalidate()