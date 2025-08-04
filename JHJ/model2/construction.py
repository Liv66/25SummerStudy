
import heapq
import random
from Route import *

class BaseConstructionStrategy:
    """
    최대 수요 고객을 최소 부하 경로에 차례로 배치하는 탐욕적 초기해 생성.
    라인홀(1) 고객 먼저, 이후 백홀(2) 고객을 삽입하여 precedence 제약 만족.
    """
    def build_solution(self, problem_info):
        K = problem_info['K']
        # 1) 빈 Route K개 생성
        routes = [Route(problem_info) for _ in range(K)]

        # 2) 라인홀 고객 목록 및 내림차순 정렬
        line_customers = [i for i, t in enumerate(problem_info['node_types']) if t == 1]
        line_customers.sort(key=lambda i: problem_info['node_demands'][i], reverse=True)

        # 3) delivery_load 기준 최소 힙 초기화
        heap = [(r.delivery_load, idx, r) for idx, r in enumerate(routes)]
        heapq.heapify(heap)

        # 4) 라인홀 고객을 최소 부하 경로에 삽입 (개선된 로직)
        for node in line_customers:
            assigned = False
            popped_routes = []
            node_demand = problem_info['node_demands'][node]
            while heap and not assigned:
                load, idx, r = heapq.heappop(heap)
                if r.get_delivery_load() + node_demand <= r.get_capacity():
                    r.add_customer(node)
                    heapq.heappush(heap, (r.delivery_load, idx, r))
                    assigned = True
                else:
                    popped_routes.append((load, idx, r))
            
            for item in popped_routes:
                heapq.heappush(heap, item)

            if not assigned:
                # 제약 완화: 용량을 초과하더라도 가장 여유 있는 차량에 할당
                print(f"Warning: Linehaul customer {node} assigned with capacity violation")
                # 가장 적게 로드된 차량을 찾아서 할당
                min_load_route = min(routes, key=lambda r: r.get_delivery_load())
                min_load_route.force_add_customer(node)
                # 힙 재구성
                heap = [(r.delivery_load, idx, r) for idx, r in enumerate(routes)]
                heapq.heapify(heap)

        # 5) 백홀 고객 목록 및 내림차순 정렬
        back_customers = [i for i, t in enumerate(problem_info['node_types']) if t == 2]
        back_customers.sort(key=lambda i: problem_info['node_demands'][i], reverse=True)

        # 6) pickup_load 기준 최소 힙 생성
        heap = [(r.pickup_load, idx, r) for idx, r in enumerate(routes)]
        heapq.heapify(heap)

        # 7) 백홀 고객을 최소 픽업 부하 경로에 삽입 (개선된 로직)
        for node in back_customers:
            assigned = False
            popped_routes = []
            node_demand = problem_info['node_demands'][node]
            while heap and not assigned:
                load, idx, r = heapq.heappop(heap)
                if r.get_pickup_load() + node_demand <= r.get_capacity():
                    r.add_customer(node)
                    heapq.heappush(heap, (r.pickup_load, idx, r))
                    assigned = True
                else:
                    popped_routes.append((load, idx, r))

            for item in popped_routes:
                heapq.heappush(heap, item)
            
            if not assigned:
                # 제약 완화: 용량을 초과하더라도 가장 여유 있는 차량에 할당
                print(f"Warning: Backhaul customer {node} assigned with capacity violation")
                # 가장 적게 로드된 차량을 찾아서 할당
                min_load_route = min(routes, key=lambda r: r.get_pickup_load())
                min_load_route.force_add_customer(node)
                # 힙 재구성
                heap = [(r.pickup_load, idx, r) for idx, r in enumerate(routes)]
                heapq.heapify(heap)

        # 8) 할당되지 않은 고객이 있는지 확인하고 강제로 할당 (안전장치)
        all_assigned_customers = set()
        for r in routes:
            all_assigned_customers.update(r.get_customers())

        all_customers = set(i for i, t in enumerate(problem_info['node_types']) if t > 0)
        unassigned_customers = list(all_customers - all_assigned_customers)

        if unassigned_customers:
            print(f"Warning: Found unassigned customers: {unassigned_customers}. Forcing assignment.")
            for customer in unassigned_customers:
                # 고객 타입에 따라 가장 적절한 경로에 강제 할당
                is_linehaul = problem_info['node_types'][customer] == 1
                if is_linehaul:
                    # 라인홀 고객: delivery_load가 가장 적은 경로에 할당
                    target_route = min(routes, key=lambda r: r.get_delivery_load())
                else:
                    # 백홀 고객: 라인홀 고객이 있으면서 pickup_load가 가장 적은 경로를 우선 탐색
                    eligible_routes = [r for r in routes if r.get_linehaul_count() > 0]
                    if eligible_routes:
                        target_route = min(eligible_routes, key=lambda r: r.get_pickup_load())
                    else:
                        # 라인홀 경로가 없으면, 임의의 경로에 추가 (이후 repair/local search에서 개선 기대)
                        target_route = routes[0]
                
                target_route.force_add_customer(customer)
                print(f"Force-assigned customer {customer} to a route.")

        return routes


class ShuffleConstructionStrategy(BaseConstructionStrategy):
    """
    BaseConstructionStrategy를 상속받되, 고객 리스트를 무작위로 섞어서 처리.
    """
    def build_solution(self, problem_info):
        K = problem_info['K']
        routes = [Route(problem_info) for _ in range(K)]

        # 라인홀 고객 목록을 섞음
        line_customers = [i for i, t in enumerate(problem_info['node_types']) if t == 1]
        random.shuffle(line_customers)

        heap = [(r.delivery_load, idx, r) for idx, r in enumerate(routes)]
        heapq.heapify(heap)

        # 라인홀 고객 할당 (개선된 로직 사용)
        for node in line_customers:
            assigned = False
            popped_routes = []
            node_demand = problem_info['node_demands'][node]
            while heap and not assigned:
                load, idx, r = heapq.heappop(heap)
                if r.get_delivery_load() + node_demand <= r.get_capacity():
                    r.add_customer(node)
                    heapq.heappush(heap, (r.delivery_load, idx, r))
                    assigned = True
                else:
                    popped_routes.append((load, idx, r))
            
            for item in popped_routes:
                heapq.heappush(heap, item)

            if not assigned:
                # 제약 완화: 용량을 초과하더라도 가장 여유 있는 차량에 할당
                print(f"Warning: Linehaul customer {node} assigned with capacity violation (shuffle)")
                # 가장 적게 로드된 차량을 찾아서 할당
                min_load_route = min(routes, key=lambda r: r.get_delivery_load())
                min_load_route.force_add_customer(node)
                # 힙 재구성
                heap = [(r.delivery_load, idx, r) for idx, r in enumerate(routes)]
                heapq.heapify(heap)

        # 백홀 고객 목록을 섞음
        back_customers = [i for i, t in enumerate(problem_info['node_types']) if t == 2]
        random.shuffle(back_customers)

        heap = [(r.pickup_load, idx, r) for idx, r in enumerate(routes)]
        heapq.heapify(heap)

        # 백홀 고객 할당 (개선된 로직 사용)
        for node in back_customers:
            assigned = False
            popped_routes = []
            node_demand = problem_info['node_demands'][node]
            while heap and not assigned:
                load, idx, r = heapq.heappop(heap)
                if r.get_pickup_load() + node_demand <= r.get_capacity():
                    r.add_customer(node)
                    heapq.heappush(heap, (r.pickup_load, idx, r))
                    assigned = True
                else:
                    popped_routes.append((load, idx, r))

            for item in popped_routes:
                heapq.heappush(heap, item)

        # 8) 할당되지 않은 고객이 있는지 확인하고 강제로 할당 (안전장치)
        all_assigned_customers = set()
        for r in routes:
            all_assigned_customers.update(r.get_customers())

        all_customers = set(i for i, t in enumerate(problem_info['node_types']) if t > 0)
        unassigned_customers = list(all_customers - all_assigned_customers)

        if unassigned_customers:
            print(f"Warning: Found unassigned customers: {unassigned_customers}. Forcing assignment.")
            for customer in unassigned_customers:
                # 고객 타입에 따라 가장 적절한 경로에 강제 할당
                is_linehaul = problem_info['node_types'][customer] == 1
                if is_linehaul:
                    target_route = min(routes, key=lambda r: r.get_delivery_load())
                else:
                    eligible_routes = [r for r in routes if r.get_linehaul_count() > 0]
                    if eligible_routes:
                        target_route = min(eligible_routes, key=lambda r: r.get_pickup_load())
                    else:
                        target_route = routes[0]
                
                target_route.force_add_customer(customer)
                print(f"Force-assigned customer {customer} to a route.")
        
        return routes

class GreedyConstructionStrategy:
    """
    매 단계마다 비용(거리+페널티) 증가가 가장 적은 고객과 위치를
    탐욕적으로 선택하여 삽입하는, 보다 정교한 초기해 생성 전략.
    """
    def _get_insertion_cost(self, customer: int, route: Route, index: int, problem_info: dict) -> float:
        """특정 위치에 고객을 삽입할 때의 비용 변화량을 계산 (거리+페널티)"""
        dist_mat = problem_info['dist_mat']
        prev_node = route.get(index - 1)
        next_node = route.get(index)

        # 1. 거리 비용 변화량 계산
        dist_cost_change = (dist_mat[prev_node][customer] + 
                            dist_mat[customer][next_node] - 
                            dist_mat[prev_node][next_node])

        # 2. 페널티 비용 변화량 계산
        demands = problem_info['node_demands']
        types = problem_info['node_types']
        capacity = route.get_capacity()
        customer_demand = demands[customer]
        
        # 생성 단계에서는 Local Search와 다른 페널티율을 사용할 수 있음
        construction_penalty_rate = 1_000 

        penalty_before = route.get_penalty_cost(construction_penalty_rate)

        # 고객을 삽입했을 경우의 새로운 부하량 (가상)
        if types[customer] == 1: # linehaul
            delivery_load_after = route.get_delivery_load() + customer_demand
            pickup_load_after = route.get_pickup_load()
        else: # backhaul
            delivery_load_after = route.get_delivery_load()
            pickup_load_after = route.get_pickup_load() + customer_demand
        
        violation_after = max(0, delivery_load_after - capacity) + max(0, pickup_load_after - capacity)
        penalty_after = violation_after * construction_penalty_rate
        
        penalty_cost_change = penalty_after - penalty_before
        
        return dist_cost_change + penalty_cost_change

    def build_solution(self, problem_info):
        K = problem_info['K']
        routes = [Route(problem_info) for _ in range(K)]
        
        unassigned_customers = [i for i, t in enumerate(problem_info['node_types']) if t > 0]
        
        while unassigned_customers:
            best_insertion = None
            min_cost = float('inf')

            # 모든 미할당 고객에 대해 최적의 삽입 위치 탐색
            for customer in unassigned_customers:
                is_linehaul = problem_info['node_types'][customer] == 1
                
                for route in routes:
                    # 삽입 가능한 위치 탐색
                    if is_linehaul:
                        # 라인홀 고객은 다른 라인홀 고객들 사이에만 삽입 가능
                        start_idx, end_idx = 1, route.get_linehaul_count() + 1
                    else:
                        # 백홀 고객은 백홀 고객들 사이에만 삽입 가능
                        # (단, 라인홀 고객이 최소 1명 있어야 함)
                        if route.get_linehaul_count() == 0:
                            continue
                        start_idx = route.get_linehaul_count() + 1
                        end_idx = route.size() - 1
                    
                    for i in range(start_idx, end_idx + 1):
                        cost = self._get_insertion_cost(customer, route, i, problem_info)
                        if cost < min_cost:
                            min_cost = cost
                            best_insertion = (customer, route, i)

            # 가장 비용이 적게 드는 삽입 실행
            if best_insertion:
                customer, route, index = best_insertion
                route.add_customer(customer, index)
                unassigned_customers.remove(customer)
            else:
                # 만약 삽입 위치를 찾지 못하면(이론상 발생하면 안됨), 안전장치로 강제 할당
                customer_to_assign = unassigned_customers.pop(0)
                target_route = min(routes, key=lambda r: r.get_delivery_load() + r.get_pickup_load())
                target_route.force_add_customer(customer_to_assign)
                print(f"Warning: Could not find best insertion for {customer_to_assign}, forcing assignment.")
        
        return routes
