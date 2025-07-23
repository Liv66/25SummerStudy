import random

def calculate_total_cost(solution, Cost_matrix):

        total_cost = 0
        # 1. 전체 경로 리스트를 하나씩 순회합니다.
        for route in solution:
            # 경로에 노드가 2개 이상 있어야만 비용이 발생합니다.
            if len(route) < 2:
                continue

            # 2. 각 경로의 비용을 계산합니다.
            current_route_cost = 0
            for i in range(len(route) - 1):
                # 3. 경로 내의 현재 노드와 다음 노드 간의 거리를 누적합니다.
                from_node = route[i]
                to_node = route[i+1]
                # print(f"from_node: {from_node}, to_node: {to_node}")
                current_route_cost += Cost_matrix[from_node][to_node]
            # 4. 계산된 경로 비용을 총비용에 더합니다.
            total_cost += current_route_cost
            
        return total_cost


def route_load(nodes, route, linehaul=True, NUM_LINEHAUL=10):
    """주어진 경로의 총 수요(load)를 계산합니다."""
    if linehaul:
        # 라인홀 노드의 수요를 계산합니다.
        return sum(nodes[customer]['demand'] for customer in route if (customer != 0) and (customer <= NUM_LINEHAUL))
    
    return sum(nodes[customer]['demand'] for customer in route if (customer != 0) and (customer > NUM_LINEHAUL))


def init_solution(nodes, NUM_VEHICLES, CAPACITY, cost_matrix):
    """
    Greedy Insertion 방식으로 모든 고객을 포함하는 고품질 초기 해를 생성합니다.
    (IndexError 수정 완료)
    """
    customers = [node['id'] for node in nodes if node['id'] != 0]
    # 라인홀 고객을 먼저 배정하기 위해 정렬
    customers.sort(key=lambda c: nodes[c]['type'] == 'backhaul')

    routes = [[0, 0] for _ in range(NUM_VEHICLES)]

    for cust_id in customers:
        best_insertion = {'cost': float('inf'), 'route_idx': -1, 'pos': -1}

        for i, route in enumerate(routes):
            # 루프 범위를 수정하여 IndexError를 해결합니다.
            for j in range(1, len(route)):
                potential_route = route[:j] + [cust_id] + route[j:]

                # 1. 용량 제약 검사
                linehaul_demand = sum(nodes[c]['demand'] for c in potential_route if nodes[c]['type'] == 'linehaul')
                backhaul_demand = sum(nodes[c]['demand'] for c in potential_route if nodes[c]['type'] == 'backhaul')
                if linehaul_demand > CAPACITY or backhaul_demand > CAPACITY:
                    continue

                # 2. 순서 제약 검사 (L-B)
                is_backhaul_started = False
                valid_order = True
                for node_id in potential_route[1:-1]:
                    if nodes[node_id]['type'] == 'backhaul':
                        is_backhaul_started = True
                    if nodes[node_id]['type'] == 'linehaul' and is_backhaul_started:
                        valid_order = False
                        break
                if not valid_order:
                    continue
                
                # 3. 백홀 제약 검사 (라인홀 없이 백홀만 존재 불가)
                if backhaul_demand > 0 and linehaul_demand == 0:
                    continue

                # 비용 증가 계산
                prev_node = potential_route[j-1]
                next_node = potential_route[j+1]
                cost_increase = cost_matrix[prev_node][cust_id] + cost_matrix[cust_id][next_node] - cost_matrix[prev_node][next_node]

                if cost_increase < best_insertion['cost']:
                    best_insertion = {'cost': cost_increase, 'route_idx': i, 'pos': j}

        if best_insertion['route_idx'] != -1:
            r_idx, pos = best_insertion['route_idx'], best_insertion['pos']
            routes[r_idx].insert(pos, cust_id)

    # print(f"초기 해 생성 완료. 포함된 고객 수: {sum(1 for r in routes for c in r if c != 0)}")
    return routes

class destroy_solution:
    def __init__(self, nodes, NUM_VEHICLES, CAPACITY, Cost_matrix):
        self.nodes = nodes
        self.NUM_VEHICLES = NUM_VEHICLES
        self.CAPACITY = CAPACITY
        self.Cost_matrix = Cost_matrix

        
    def random_removal(self, current_routes , num_to_remove=1):
        all_customers = [
        customer for route in current_routes for customer in route if customer != 0
        ]

        num_to_remove = min(num_to_remove, len(self.nodes))
        removed_list = random.sample(all_customers, num_to_remove)
        removed_set = set(removed_list)
        new_solution = []
        for route in current_routes:
            # 현재 경로에서 제거될 고객들을 필터링합니다.
            new_route = [customer for customer in route if customer not in removed_set]
            new_solution.append(new_route)

        return new_solution, removed_list
    
    def worst_removal(self, current_routes, num_to_remove):
        """비용(거리)을 가장 많이 차지하는 고객을 찾아 제거합니다."""
        costs = []
        for route in current_routes:
            for i in range(1, len(route) - 1):
                prev_node = route[i-1]
                customer_node = route[i]
                next_node = route[i+1]
                
                cost_saving = (self.Cost_matrix[prev_node][customer_node] +
                               self.Cost_matrix[customer_node][next_node] -
                               self.Cost_matrix[prev_node][next_node])
                
                costs.append((cost_saving, customer_node))

        # 비용이 큰 순서대로 정렬
        costs.sort(key=lambda x: x[0], reverse=True)
        
        # 제거할 고객 목록 선택
        removed_list = [customer for cost, customer in costs[:num_to_remove]]
        removed_set = set(removed_list)
        
        # 고객 제거 후 새 경로 생성
        new_solution = [[c for c in r if c not in removed_set] for r in current_routes]
        return new_solution, removed_list

    def route_removal(self, current_routes, num_to_remove = 1):

        # 1. 고객이 한 명이라도 있는, 즉 비어있지 않은 경로의 인덱스를 찾습니다.
        non_empty_route_indices = [
            i for i, route in enumerate(current_routes) if len(route) > 2
        ]

        # 제거할 경로가 없으면 아무것도 하지 않습니다.
        if not non_empty_route_indices:
            return current_routes, []

        # 2. 실제로 제거할 경로 수를 정합니다. (전체 경로 수보다 많을 수 없음)
        actual_num_to_remove = min(num_to_remove, len(non_empty_route_indices))

        # 3. 제거할 경로의 인덱스를 무작위로 선택합니다.
        indices_to_remove = random.sample(non_empty_route_indices, actual_num_to_remove)
        indices_to_remove_set = set(indices_to_remove)

        # 4. 새로운 해와 제거된 고객 목록을 만듭니다.
        new_solution = []
        removed_list = []

        for i, route in enumerate(current_routes):
            if i in indices_to_remove_set:
                # '제거 대상 경로'인 경우:
                # a. 경로 내 모든 고객을 '제거된 고객 목록'에 추가합니다.
                removed_list.extend([customer for customer in route if customer != 0])
                # b. 경로는 비어있는 상태 [0, 0]으로 변경합니다.
                new_solution.append([0, 0])
            else:
                # '유지 대상 경로'인 경우:
                # 원래 경로를 그대로 추가합니다.
                new_solution.append(route)

        return new_solution, removed_list
    
class repair_solution:
    def __init__(self, nodes, NUM_VEHICLES, CAPACITY, NUM_LINEHAUL, NUM_BACKHAUL, Cost_matrix):
        self.nodes = nodes
        self.NUM_VEHICLES = NUM_VEHICLES
        self.CAPACITY = CAPACITY
        self.NUM_LINEHAUL = NUM_LINEHAUL
        self.NUM_BACKHAUL = NUM_BACKHAUL
        self.Cost_matrix = Cost_matrix

    def is_route_valid(self, route):
        """
        주어진 단일 경로가 Linehaul/Backhaul 순서 및 용량 제약을 모두 만족하는지 검사합니다.
        """
        # 1. Linehaul-Backhaul 순서 검사
        is_backhaul_started = False
        for customer_id in route:
            if customer_id == 0: continue
            is_linehaul = (customer_id <= self.NUM_LINEHAUL)
            
            if not is_linehaul: # 현재 노드가 Backhaul이면
                is_backhaul_started = True
            
            if is_linehaul and is_backhaul_started: # Backhaul 시작 후 Linehaul이 나오면 무효
                return False

        # 2. Linehaul 용량 검사
        linehaul_demand = sum(self.nodes[c]['demand'] for c in route if 0 < c <= self.NUM_LINEHAUL)
        if linehaul_demand > self.CAPACITY:
            return False

        # 3. Backhaul 용량 검사
        backhaul_demand = sum(self.nodes[c]['demand'] for c in route if c > self.NUM_LINEHAUL)
        if backhaul_demand > self.CAPACITY:
            return False
        
        # 4. linehaul이 최소한 하나의 경로가 있어야 한다는 제약
        if backhaul_demand > 0 and linehaul_demand == 0:
            return False
    
        # 모든 제약을 통과하면 유효
        return True

    def greedy_insertion(self, new_solution, removed_list):
        repaired_solution = [route[:] for route in new_solution]
        
        line_nodes = sorted([c for c in removed_list if c <= self.NUM_LINEHAUL])
        back_nodes = sorted([c for c in removed_list if c > self.NUM_LINEHAUL])

        # --- 1. Linehaul 고객 우선 삽입 ---
        for customer in line_nodes:
            best_insertion = {'cost_increase': float('inf'), 'route_idx': -1, 'pos': -1, 'is_new': False}
            
            # 옵션 A: 기존 경로에 삽입
            for i, route in enumerate(repaired_solution):
                for j in range(1, len(route)):
                    potential_route = route[:j] + [customer] + route[j:]
                    if not self.is_route_valid(potential_route):
                        continue
                    
                    cost_increase = self.Cost_matrix[potential_route[j-1]][customer] + \
                                    self.Cost_matrix[customer][potential_route[j+1]] - \
                                    self.Cost_matrix[potential_route[j-1]][potential_route[j+1]]
                    
                    if cost_increase < best_insertion['cost_increase']:
                        best_insertion.update({'cost_increase': cost_increase, 'route_idx': i, 'pos': j, 'is_new': False})

            # 옵션 B: 새 차량에 배차
            potential_new_route = [0, customer, 0]
            if self.is_route_valid(potential_new_route):
                # new_route_cost = 2 * calculate_distance2(self.nodes[0], self.nodes[customer])
                new_route_cost = 2 * self.Cost_matrix[0][customer]
                if new_route_cost < best_insertion['cost_increase']:
                    best_insertion.update({'cost_increase': new_route_cost, 'route_idx': -1, 'is_new': True})

            # 최종 결정에 따라 삽입 실행
            if best_insertion['cost_increase'] != float('inf'):
                
                # 2. 유효한 위치를 찾았다면, 그제야 '새 경로'인지 '기존 경로'인지 확인합니다.
                if best_insertion['is_new']:
                    # 첫 번째 비어있는 경로를 찾아 할당
                    for i, route in enumerate(repaired_solution):
                        if len(route) <= 2:
                            repaired_solution[i] = [0, customer, 0]
                            break
                else:
                    repaired_solution[best_insertion['route_idx']].insert(best_insertion['pos'], customer)
    

        # --- 2. Backhaul 고객 삽입 ---
        for customer in back_nodes:
            best_insertion = {'cost_increase': float('inf'), 'route_idx': -1, 'pos': -1}
            
            for i, route in enumerate(repaired_solution):
                for j in range(1, len(route)):
                    potential_route = route[:j] + [customer] + route[j:]
                    if not self.is_route_valid(potential_route):
                        continue
                    
                    cost_increase = self.Cost_matrix[potential_route[j-1]][customer] + \
                                    self.Cost_matrix[customer][potential_route[j+1]] - \
                                    self.Cost_matrix[potential_route[j-1]][potential_route[j+1]]

                    if cost_increase < best_insertion['cost_increase']:
                        best_insertion.update({'cost_increase': cost_increase, 'route_idx': i, 'pos': j})
            
            # Backhaul은 새 경로를 만들지 않으므로, 유효한 자리가 있을 때만 삽입
            if best_insertion['route_idx'] != -1:
                repaired_solution[best_insertion['route_idx']].insert(best_insertion['pos'], customer)

        # print(sum(1 for route in repaired_solution for customer in route if customer != 0))        
        return repaired_solution
    
    def regret_insertion(self, new_solution, removed_list):
        repaired_solution = [route[:] for route in new_solution]
        customers_to_insert = list(removed_list)
        customers_to_insert = sorted(customers_to_insert)
        while customers_to_insert:
            regret_values = []

            # 1. 아직 삽입되지 않은 모든 고객에 대해 후회 값 계산
            for customer in customers_to_insert:
                insert_costs = []

                # --- 모든 유효한 삽입 위치와 비용을 찾습니다 ---
                for i, route in enumerate(repaired_solution):
                    # 옵션 A: 기존 경로에 삽입
                    for j in range(1, len(route)):
                        potential_route = route[:j] + [customer] + route[j:]
                        if self.is_route_valid(potential_route):

                            cost_increase = self.Cost_matrix[potential_route[j-1]][customer] + \
                                            self.Cost_matrix[customer][potential_route[j+1]] - \
                                            self.Cost_matrix[potential_route[j-1]][potential_route[j+1]]
                            insert_costs.append({'cost': cost_increase, 'route_idx': i, 'pos': j, 'is_new': False})

                # 옵션 B: 새 차량에 배차 (Linehaul 고객만 가능)
                if self.nodes[customer]['type'] == 'linehaul':
                    potential_new_route = [0, customer, 0]
                    if self.is_route_valid(potential_new_route):
                        # new_route_cost = 2 * calculate_distance2(self.nodes[0], self.nodes[customer])
                        new_route_cost = 2 * self.Cost_matrix[0][customer]
                        # 새 경로 옵션은 비어있는 어떤 경로든 가능하므로, route_idx는 -1로 표시
                        insert_costs.append({'cost': new_route_cost, 'route_idx': -1, 'is_new': True})

                # 2. 후회 값 계산
                if not insert_costs: continue
                insert_costs.sort(key=lambda x: x['cost'])
                
                best_info = insert_costs[0]
                regret = insert_costs[1]['cost'] - best_info['cost'] if len(insert_costs) >= 2 else 0
                regret_values.append({'regret': regret, 'customer': customer, 'info': best_info})

            # 3. 후회 값이 가장 큰 고객을 찾아 삽입
            if not regret_values: break
            regret_values.sort(key=lambda x: x['regret'], reverse=True)
            
            customer_to_insert_info = regret_values[0]
            info = customer_to_insert_info['info']
            cust_id = customer_to_insert_info['customer']

            if info['is_new']:
                # 첫 번째 비어있는 경로를 찾아 할당
                for i, r in enumerate(repaired_solution):
                    if len(r) <= 2:
                        repaired_solution[i] = [0, cust_id, 0]
                        break
            else:
                repaired_solution[info['route_idx']].insert(info['pos'], cust_id)
            
            customers_to_insert.remove(cust_id)
        # print(sum(1 for route in repaired_solution for customer in route if customer != 0))
        return repaired_solution
                
    def random_insertion(self, new_solution, removed_list):
        # print(f"removed_list: {removed_list}")
        repaired_solution = [route[:] for route in new_solution]
        customers_to_insert = list(removed_list)
        customers_to_insert = sorted(customers_to_insert)
        # print(f"customers_to_insert: {customers_to_insert}")
        for customer in customers_to_insert:
            possible_insertions = []
            # print(f"customer: {customer}")
            # --- 모든 유효한 삽입 위치를 찾습니다 ---
            # 옵션 A: 기존 경로에 삽입
            for i, route in enumerate(repaired_solution):
                for j in range(1, len(route)):
                    potential_route = route[:j] + [customer] + route[j:]
                    if self.is_route_valid(potential_route):
                        possible_insertions.append({'route_idx': i, 'pos': j, 'is_new': False})

            # 옵션 B: 새 차량에 배차 (Linehaul 고객만 가능)
            if self.nodes[customer]['type'] == 'linehaul':
                potential_new_route = [0, customer, 0]
                if self.is_route_valid(potential_new_route):
                    # 새 경로 옵션은 비어있는 어떤 경로든 가능하므로, route_idx는 -1로 표시
                    possible_insertions.append({'route_idx': -1, 'is_new': True})

            # print(f"possible_insertions: {possible_insertions}")

            # 3. 유효한 위치 중 하나를 무작위로 선택하여 실행
            if possible_insertions:
                chosen_option = random.choice(possible_insertions)
                # print(f"chosen_option: {chosen_option}")
                if chosen_option['is_new']:
                    for i, r in enumerate(repaired_solution):
                        # print(r)
                        if len(r) <= 2:
                            repaired_solution[i] = [0, customer, 0]
                else:
                    repaired_solution[chosen_option['route_idx']].insert(chosen_option['pos'], customer)
            # print(f"삽입 후 경로: {repaired_solution}")
            # print(f"삽입 후 경로 길이: {sum(1 for route in repaired_solution for customer in route if customer != 0)}")
   
        # print(sum(1 for route in repaired_solution for customer in route if customer != 0))
        return repaired_solution
    