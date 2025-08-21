import time
from JHJ_construction import GreedyConstructionStrategy
from JHJ_method import FirstImprovementStrategy
from JHJ_Solution import CVRPBSolution
import random
import copy

class IteratedLocalSearchSolver:

    def __init__(self, construction_strategy, local_search_strategy):
        self.construction_strategy = construction_strategy
        self.local_search_strategy = local_search_strategy

    def solve(self, instance, start_time, time_limit, log=True) :
        # 1. 초기해 생성
        current_solution = CVRPBSolution(self.construction_strategy.build_solution(instance))

        # 2. 초기 Local Search
        cnt = 0
        initial_ls_improvement_limit = 200

        while time.time() - start_time < time_limit:
            if cnt >= initial_ls_improvement_limit:
                if log:
                    print(f"Initial LS reached improvement limit of {initial_ls_improvement_limit}.")
                break

            was_improved = self.local_search_strategy.minimize(current_solution, start_time, time_limit)
            if not was_improved:
                break

            cnt += 1

        best_solution = copy.deepcopy(current_solution)


        iteration = 0
        iterations_no_improvement = 0
        max_iters_no_improvement = 200 # 개선 안되는 횟수

        # perturb에서 아예 새로운 greedy_construction 초기해에서 시작 ?

        # 3. ILS 메인
        while time.time() - start_time < time_limit:
            if iterations_no_improvement >= max_iters_no_improvement:
                print(f"Stopping ILS after {max_iters_no_improvement} iterations without improvement.")
                break

            iteration += 1
            elapsed_time = time.time() - start_time
            best_cost = best_solution.get_total_cost()
            is_best_feasible = best_solution.get_penalty_cost(best_solution.penalty_rate) < 1e-9

            if log:
                print(f"반복 : {iteration}, Best Cost: {best_cost:.2f} (Feasible: {is_best_feasible}), Time: {elapsed_time:.1f}s/{time_limit}s ---")

            new_construction_strategy = self.construction_strategy.build_solution(instance)

            # 4. 교란 (Perturbation)
            #
            strength = 0.5 + (iterations_no_improvement * 0.01) if iterations_no_improvement <= 30 else 0.5
            perturbed_solution = self._perturb(best_solution, instance, strength)


            # if iterations_no_improvement > 30 == 0:
            #     perturbed_solution = CVRPBSolution(self.construction_strategy.build_solution(instance))

            # 5. 교란된 해에 대한 Local Search
            ls_improvements = 0
            max_ls_improvements = 250  # ILS 내 Local Search 개선 횟수 제한
            while time.time() - start_time < time_limit and ls_improvements < max_ls_improvements:

                was_improved = self.local_search_strategy.minimize(perturbed_solution, start_time, time_limit, iterations_no_improvement)
                if not was_improved:
                    break
                ls_improvements += 1

            if log:
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
                iterations_no_improvement = 0
                if log:
                    print(f"*** New best solution found! Cost: {best_solution.get_total_cost():.2f} (Feasible: {is_new_feasible}) ***")
            else:
                iterations_no_improvement += 1

                if log:
                    print(f"    Solution not accepted. No improvement count: {iterations_no_improvement}")

        # ILS 종료 원인 출력
        final_elapsed_time = time.time() - start_time

        if log:
            if final_elapsed_time >= time_limit:
                print(f"ILS terminated due to time limit. Total time: {final_elapsed_time:.1f}s")
            elif iterations_no_improvement >= max_iters_no_improvement:
                print(f"ILS terminated due to no improvement for {max_iters_no_improvement} iterations.")
            else:
                print(f"ILS terminated for unknown reason. Time: {final_elapsed_time:.1f}s, No improvements: {iterations_no_improvement}")

        return best_solution

    def _perturb(self, solution, problem_info, strength):
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

    def _repair(self, solution, customers_to_add, problem_info):
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
                    start_idx, end_idx = 1, len(route.linehaul_customers) + 1
                else:
                    if len(route.linehaul_customers) == 0: continue
                    start_idx = len(route.linehaul_customers) + 1
                    end_idx = route.size() - 1
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
                target_route = min(solution.get_routes(), key=lambda r: r.delivery_load + r.pickup_load)
                target_route.force_add_customer(customer)
        solution.invalidate()