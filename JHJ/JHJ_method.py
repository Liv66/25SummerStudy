import time

from JHJ_Solution import CVRPBSolution

class FirstImprovementStrategy:

    # 첫 번째 개선
  
    def minimize(self, solution, start_time, time_limit, iterations_no_improvement=0):

        # 시간 제한 체크
        if time.time() - start_time >= time_limit:
            return False

        # 3. 2-opt 연산 시도
        if iterations_no_improvement <15 and self._try_two_opt(solution, start_time, time_limit):
            return True

        # 1. Exchange 연산 시도
        if self._try_exchange(solution, start_time, time_limit):
            return True

        # 2. Relocate 연산 시도
        if self._try_relocate(solution, start_time, time_limit):
            return True

        return False
    
    def _try_relocate(self, solution, start_time, time_limit):
        routes = solution.get_routes()
        
        for from_route_idx, from_route in enumerate(routes):
            if time.time() - start_time >= time_limit:
                return False
                
            if not from_route.get_customers():
                continue

            # from_pos는 1부터 (고객 수)까지 순회
            for from_pos in range(1, from_route.size() - 1):
                # to_pos는 1부터 (고객 수 + 1)까지 순회 (맨 뒤에 삽입 가능)
                for to_route_idx, to_route in enumerate(routes):
                    for to_pos in range(1, to_route.size()):
                        if from_route_idx == to_route_idx and (to_pos == from_pos or to_pos == from_pos + 1):
                            continue
                        
                        if self._is_relocate_improving(solution, from_route_idx, from_pos, to_route_idx, to_pos):
                            self._apply_relocate(solution, from_route_idx, from_pos, to_route_idx, to_pos)
                            return True
        return False
    
    def _try_exchange(self, solution, start_time, time_limit):
        routes = solution.get_routes()
        
        for r1_idx, route1 in enumerate(routes):
            if time.time() - start_time >= time_limit:
                return False
                
            if not route1.get_customers():
                continue
                
            for pos1 in range(1, route1.size() - 1):
                for r2_idx, route2 in enumerate(routes):
                    if r2_idx < r1_idx:
                        continue
                        
                    if not route2.get_customers():
                        continue

                    start_pos2 = pos1 + 1 if r1_idx == r2_idx else 1
                    for pos2 in range(start_pos2, route2.size() - 1):
                        
                        if self._is_exchange_improving(solution, r1_idx, pos1, r2_idx, pos2):
                            self._apply_exchange(solution, r1_idx, pos1, r2_idx, pos2)
                            return True
        return False
    
    def _try_two_opt(self, solution, start_time, time_limit):

        routes = solution.get_routes()
        
        for route_idx, route in enumerate(routes):
            if time.time() - start_time >= time_limit:
                return False
                
            if route.get_customers_count() < 4:  # 최소 4개 고객 필요
                continue
                
            for i in range(1, route.size() - 2):
                for j in range(i + 2, route.size() - 1):
                    
                    if self._is_two_opt_improving(solution, route_idx, i, j):
                        self._apply_two_opt(solution, route_idx, i, j)
                        return True
        return False
    
    def _is_relocate_improving(self, solution, from_route_idx, from_pos, to_route_idx, to_pos):

        routes = solution.get_routes()
        from_route = routes[from_route_idx]
        to_route = routes[to_route_idx]
        
        customer = from_route.get(from_pos)
        problem_info = solution.get_problem_info()
        dist = problem_info['dist_mat']
        
        # 거리 비용 변화 계산
        prev_from = from_route.get(from_pos - 1)
        next_from = from_route.get(from_pos + 1) if from_pos < from_route.size() - 1 else 0
        prev_to = to_route.get(to_pos - 1)
        next_to = to_route.get(to_pos) if to_pos < to_route.size() else 0
        
        distance_gain = (dist[prev_from][customer] + dist[customer][next_from] + dist[prev_to][next_to]) - \
                       (dist[prev_from][next_from] + dist[prev_to][customer] + dist[customer][next_to])
        
        # 페널티 비용 변화 계산
        penalty_gain = self._calculate_relocate_penalty_gain(solution, from_route_idx, from_pos, to_route_idx, customer)
        
        # 순서 제약 체크
        if not self._check_relocate_order(solution, from_route_idx, from_pos, to_route_idx, to_pos, customer):
            return False
        
        # 개선 여부 판단 (페널티 우선, 거리 차선)
        return penalty_gain > 1e-9 or (abs(penalty_gain) < 1e-9 and distance_gain > 1e-9)
    
    def _is_exchange_improving(self, solution, r1_idx, pos1, r2_idx, pos2):

        routes = solution.get_routes()
        route1, route2 = routes[r1_idx], routes[r2_idx]
        customer1, customer2 = route1.get(pos1), route2.get(pos2)
        
        problem_info = solution.get_problem_info()
        types = problem_info['node_types']
        
        # 같은 타입의 고객만 교환 가능
        if types[customer1] != types[customer2]:
            return False
        
        dist = problem_info['dist_mat']
        
        # 거리 비용 변화 계산
        prev1 = route1.get(pos1 - 1)
        next1 = route1.get(pos1 + 1) if pos1 < route1.size() - 1 else 0
        prev2 = route2.get(pos2 - 1)
        next2 = route2.get(pos2 + 1) if pos2 < route2.size() - 1 else 0
        
        pre_cost = dist[prev1][customer1] + dist[customer1][next1] + dist[prev2][customer2] + dist[customer2][next2]
        post_cost = dist[prev1][customer2] + dist[customer2][next1] + dist[prev2][customer1] + dist[customer1][next2]
        
        # 인접한 경우 조정
        if route1 is route2 and abs(pos1 - pos2) == 1:
            pre_cost -= 2 * dist[customer1][customer2]
        
        distance_gain = pre_cost - post_cost
        
        # 페널티 변화량 계산
        penalty_gain = self._calculate_exchange_penalty_gain(solution, r1_idx, pos1, r2_idx, pos2)
        
        return penalty_gain > 1e-9 or (abs(penalty_gain) < 1e-9 and distance_gain > 1e-9)
    
    def _is_two_opt_improving(self, solution, route_idx, i, j):

        route = solution.get_routes()[route_idx]
        problem_info = solution.get_problem_info()
        dist = problem_info['dist_mat']
        
        # 라인홀-백홀 경계를 건드리지 않는지 체크
        if route.get(i) in route.get_linehaul_customers() and route.get(i+1) in route.get_backhaul_customers():
            return False
        
        c1 = route.get(i)
        c2 = route.get(i + 1)
        c3 = route.get(j)
        c4 = route.get(j + 1) if j + 1 < route.size() else 0
        
        distance_gain = (dist[c1][c2] + dist[c3][c4]) - (dist[c1][c3] + dist[c2][c4])
        
        return distance_gain > 1e-9
    
    def _calculate_relocate_penalty_gain(self, solution, from_route_idx, from_pos, to_route_idx, customer) -> float:

        routes = solution.get_routes()
        from_route = routes[from_route_idx]
        to_route = routes[to_route_idx]
        
        problem_info = solution.get_problem_info()
        demands = problem_info['node_demands']
        types = problem_info['node_types']
        capa = problem_info['capa']
        penalty_rate = solution.penalty_rate
        
        customer_demand = demands[customer]
        is_linehaul = (types[customer] == 1)
        
        # 이동 전 페널티
        penalty_before = from_route.get_penalty_cost(penalty_rate) + to_route.get_penalty_cost(penalty_rate)
        
        # 이동 후 부하량 계산
        from_delivery_after = from_route.get_delivery_load() - (customer_demand if is_linehaul else 0)
        from_pickup_after = from_route.get_pickup_load() - (0 if is_linehaul else customer_demand)
        to_delivery_after = to_route.get_delivery_load() + (customer_demand if is_linehaul else 0)
        to_pickup_after = to_route.get_pickup_load() + (0 if is_linehaul else customer_demand)
        
        # 이동 후 페널티
        from_violation = max(0, from_delivery_after - capa) + max(0, from_pickup_after - capa)
        to_violation = max(0, to_delivery_after - capa) + max(0, to_pickup_after - capa)
        penalty_after = (from_violation + to_violation) * penalty_rate
        
        return penalty_before - penalty_after
    
    def _check_relocate_order(self, solution, from_route_idx, from_pos, to_route_idx, to_pos, customer) -> bool:

        routes = solution.get_routes()
        from_route = routes[from_route_idx]
        to_route = routes[to_route_idx]
        
        problem_info = solution.get_problem_info()
        types = problem_info['node_types']
        is_linehaul = (types[customer] == 1)
        
        if to_pos == 0:  # depot 위치는 불가
            return False
        
        # 백홀 고객을 다른 경로로 이동시킬 때, 대상 경로에 라인홀이 없으면 불가
        if not is_linehaul and from_route_idx != to_route_idx and to_route.get_linehaul_count() == 0:
            return False
        
        # 라인홀 고객을 제거했을 때 해당 경로에 백홀만 남으면 불가
        if is_linehaul and from_route.get_linehaul_count() == 1 and from_route.get_backhaul_count() > 0:
            return False
        
        # 삽입 위치 체크
        if is_linehaul:
            return to_pos <= to_route.get_linehaul_count() + 1
        else:
            return to_pos > to_route.get_linehaul_count()
    
    def _apply_relocate(self, solution, from_route_idx, from_pos, to_route_idx, to_pos):

        routes = solution.get_routes()
        from_route = routes[from_route_idx]
        to_route = routes[to_route_idx]
        
        customer = from_route.get(from_pos)
        
        # 정확한 gain 계산 (apply 전에)
        problem_info = solution.get_problem_info()
        dist = problem_info['dist_mat']
        
        prev_from = from_route.get(from_pos - 1)
        next_from = from_route.get(from_pos + 1) if from_pos < from_route.size() - 1 else 0
        
        insert_pos = to_pos
        if from_route_idx == to_route_idx and from_pos < to_pos:
            insert_pos -= 1
            
        prev_to = to_route.get(insert_pos - 1)
        next_to = to_route.get(insert_pos) if insert_pos < to_route.size() else 0
        
        distance_gain = (dist[prev_from][customer] + dist[customer][next_from] + dist[prev_to][next_to]) - \
                       (dist[prev_from][next_from] + dist[prev_to][customer] + dist[customer][next_to])
        
        penalty_gain = self._calculate_relocate_penalty_gain(solution, from_route_idx, from_pos, to_route_idx, customer)
        
        # 실제 move 적용
        from_route.remove_customer(from_pos)
        to_route.add_customer(customer, insert_pos)
        
        # Delta evaluation으로 비용 업데이트
        total_gain = penalty_gain + distance_gain
        solution.update_cost_with_gain(total_gain)
    
    def _calculate_exchange_penalty_gain(self, solution, r1_idx, pos1, r2_idx, pos2) -> float:

        routes = solution.get_routes()
        route1, route2 = routes[r1_idx], routes[r2_idx]
        
        problem_info = solution.get_problem_info()
        demands = problem_info['node_demands']
        capa = problem_info['capa']
        penalty_rate = solution.penalty_rate
        
        customer1, customer2 = route1.get(pos1), route2.get(pos2)
        demand1, demand2 = demands[customer1], demands[customer2]
        
        # 교환 전 페널티
        penalty_before = route1.get_penalty_cost(penalty_rate)
        if route1 is not route2:
            penalty_before += route2.get_penalty_cost(penalty_rate)
            
        # 교환 후 부하량 계산
        if route1 is route2:
            # 같은 경로 내에서는 부하량 변화 없음
            return 0.0
        
        r1_delivery_after = route1.get_delivery_load() - demand1 + demand2
        r1_pickup_after = route1.get_pickup_load() # 같은 타입 교환이므로 한쪽만 고려
        r2_delivery_after = route2.get_delivery_load() - demand2 + demand1
        r2_pickup_after = route2.get_pickup_load()
        
        if route1.get(pos1) in route1.get_backhaul_customers():
             r1_delivery_after = route1.get_delivery_load()
             r1_pickup_after = route1.get_pickup_load() - demand1 + demand2
             r2_delivery_after = route2.get_delivery_load()
             r2_pickup_after = route2.get_pickup_load() - demand2 + demand1
        
        # 교환 후 페널티
        r1_violation = max(0, r1_delivery_after - capa) + max(0, r1_pickup_after - capa)
        r2_violation = max(0, r2_delivery_after - capa) + max(0, r2_pickup_after - capa)
        penalty_after = (r1_violation + r2_violation) * penalty_rate
        
        return penalty_before - penalty_after
    
    def _apply_exchange(self, solution, r1_idx, pos1, r2_idx, pos2):

        routes = solution.get_routes()
        route1, route2 = routes[r1_idx], routes[r2_idx]
        
        customer1, customer2 = route1.get(pos1), route2.get(pos2)
        
        # 정확한 gain 계산 (apply 전에)
        problem_info = solution.get_problem_info()
        dist = problem_info['dist_mat']
        
        prev1 = route1.get(pos1 - 1)
        next1 = route1.get(pos1 + 1) if pos1 < route1.size() - 1 else 0
        prev2 = route2.get(pos2 - 1)
        next2 = route2.get(pos2 + 1) if pos2 < route2.size() - 1 else 0
        
        pre_cost = dist[prev1][customer1] + dist[customer1][next1] + dist[prev2][customer2] + dist[customer2][next2]
        post_cost = dist[prev1][customer2] + dist[customer2][next1] + dist[prev2][customer1] + dist[customer1][next2]
        
        # 인접한 경우 조정
        if route1 is route2 and abs(pos1 - pos2) == 1:
            pre_cost -= 2 * dist[customer1][customer2]
            
        distance_gain = pre_cost - post_cost
        penalty_gain = self._calculate_exchange_penalty_gain(solution, r1_idx, pos1, r2_idx, pos2)
        
        # 실제 move 적용
        if route1 is route2:
            # 같은 경로 내 교환
            if pos1 > pos2:
                route1.remove_customer(pos1)
                route1.remove_customer(pos2)
                route1.add_customer(customer1, pos2)
                route1.add_customer(customer2, pos1)
            else:
                route1.remove_customer(pos2)
                route1.remove_customer(pos1)
                route1.add_customer(customer2, pos1)
                route1.add_customer(customer1, pos2)
        else:
            # 다른 경로간 교환
            route1.remove_customer(pos1)
            route2.remove_customer(pos2)
            route1.add_customer(customer2, pos1)
            route2.add_customer(customer1, pos2)
        
        # 비용 업데이트
        total_gain = penalty_gain + distance_gain
        solution.update_cost_with_gain(total_gain)
    
    def _apply_two_opt(self, solution, route_idx, i, j):

        route = solution.get_routes()[route_idx]
        problem_info = solution.get_problem_info()
        
        # 정확한 gain 계산 (apply 전에)
        dist = problem_info['dist_mat']
        c1 = route.get(i)
        c2 = route.get(i + 1)
        c3 = route.get(j)
        c4 = route.get(j + 1) if j + 1 < route.size() else 0
        
        distance_gain = (dist[c1][c2] + dist[c3][c4]) - (dist[c1][c3] + dist[c2][c4])
        
        # 실제 move 적용
        customers = route.get_customers()
        new_customers = customers[:i+1] + customers[i+1:j+1][::-1] + customers[j+1:]
        route.set_customers(new_customers, problem_info)
        
        #  비용 업데이트 (페널티 변화 없음)
        solution.update_cost_with_gain(distance_gain)