import random
from Route import *

class GreedyConstructionStrategy:
    # 총 비용이 가장 적게 드는 위치로 노드 삽입
    def _get_insertion_cost(self, customer, route, index, problem_info):
        dist_mat = problem_info['dist_mat']
        prev_node = route.get(index - 1)
        next_node = route.get(index)

        # 1. 거리 비용 변화량 계산
        # => 변화한 노드에서 발생한 거리 비용 ( prev, next는 고정 )
        dist_cost_change = (dist_mat[prev_node][customer] + 
                            dist_mat[customer][next_node] - 
                            dist_mat[prev_node][next_node])

        # 2. 페널티 비용 변화량 계산
        demands = problem_info['node_demands']
        types = problem_info['node_types']
        capacity = route.get_capacity()
        customer_demand = demands[customer]
        
        # 생성 단계에서는 Local Search와 다른 페널티율을 사용할 수 있음
        construction_penalty_rate = 100

        # 이전 패널티
        penalty_before = route.get_penalty_cost(construction_penalty_rate)

        # 고객을 삽입했을 경우의 새로운 부하량 (가상)
        #
        if types[customer] == 1: # linehaul
            delivery_load_after = route.get_delivery_load() + customer_demand
            pickup_load_after = route.get_pickup_load()
        else: # backhaul
            delivery_load_after = route.get_delivery_load()
            pickup_load_after = route.get_pickup_load() + customer_demand
        
        violation_after = max(0, delivery_load_after - capacity) + max(0, pickup_load_after - capacity)
        penalty_after = violation_after * construction_penalty_rate
        # 용량 초과한만큼 패널티 부여
        penalty_cost_change = penalty_after - penalty_before
        
        return dist_cost_change + penalty_cost_change

    def build_solution(self, problem_info):
        K = problem_info['K']
        routes = [Route(problem_info) for _ in range(K)]
        
        unassigned_customers = [i for i, t in enumerate(problem_info['node_types']) if t > 0]
        random.shuffle(unassigned_customers) # 고객 리스트를 무작위로 섞음
        
        while unassigned_customers:
            best_insertion = None
            min_cost = float('inf')

            # 이번에는 '다음으로 삽입할 고객'을 미리 정하지 않고,
            # 섞인 리스트의 첫 번째 고객을 대상으로 최적의 위치를 찾습니다.
            customer_to_insert = unassigned_customers[0]
            is_linehaul = problem_info['node_types'][customer_to_insert] == 1
            
            best_route_for_customer = None
            best_index_for_customer = -1

            for route in routes:
                if is_linehaul:
                    start_idx, end_idx = 1, route.get_linehaul_count() + 1
                else:
                    if route.get_linehaul_count() == 0:
                        continue
                    start_idx = route.get_linehaul_count() + 1
                    end_idx = route.size() - 1
                # 노드를 각 경로 내에서도 어디에 넣어야할지 N번 확인
                for i in range(start_idx, end_idx + 1):
                    # 거리가 변하면서 발생한 비용
                    cost = self._get_insertion_cost(customer_to_insert, route, i, problem_info)
                    if cost < min_cost:
                        min_cost = cost
                        best_route_for_customer = route
                        best_index_for_customer = i
            # 노드의 최적 위치에 삽입
            if best_route_for_customer:
                best_route_for_customer.add_customer(customer_to_insert, best_index_for_customer)
                unassigned_customers.pop(0)
            else:
                # 삽입 위치를 찾지 못하면 강제 할당
                customer_to_assign = unassigned_customers.pop(0)
                target_route = min(routes, key=lambda r: r.get_delivery_load() + r.get_pickup_load())
                target_route.force_add_customer(customer_to_assign)
                print(f"Warning: Could not find best insertion for {customer_to_assign}, forcing assignment.")
        
        return routes
