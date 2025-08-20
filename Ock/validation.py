def check_feasible(problem_info, sol, elapsed, timelimit):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    if elapsed > timelimit + 1:
        print("Time Out")
        return 0

    if len(sol) > K:
        print(f"vehicle 수는 {K}대까지 사용 가능합니다. 현재 : {len(sol)}")
        return 0

    total_cost = 0
    visit = [0] * len(node_type)
    visit[0] = 1

    for idx, route in enumerate(sol):
        if route[0] != 0:
            print(f"depot에서 출발해야 합니다. {idx}번째 차량 경로 {route}")
            return 0

        if route[-1] != 0:
            print(f"depot으로 도착해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if node_type[route[1]] == 2:
            print(f"차량은 backhaul만 갈 수 없습니다. {idx}번째 차량 경로 {route}")
            return 0
        cost = 0
        load = 0

        pre = 0
        flag = False
        route_type = [0] * len(route)

        for i in range(1, len(route) - 1):
            nxt = route[i]
            if visit[nxt]:
                print(f"{nxt} 노드 2번 이상 방문")
                return 0
            visit[nxt] = 1
            cost += dist_mat[pre][nxt]
            load += node_demand[nxt]
            route_type[i] = node_type[nxt]
            if nxt == 0:
                print(f"{idx}번째 차량 depot을 3번 이상 방문 {route}")
                return 0

            if node_type[pre] == 1 and node_type[nxt] == 2:
                if flag:
                    print(f"{idx}번째 차량 line -> backhaul -> line 경로 존재")
                    print(node_type)
                    return 0
                flag = True
                load = 0

            if load > capa:
                if flag:
                    print(f"{idx}번째 차량의 back 용량 초과 {load}, {capa}")
                else:
                    print(f"{idx}번째 차량의 line 용량 초과 {load}, {capa}")
                return 0
            pre = nxt
        cost += dist_mat[pre][0]
        total_cost += cost

    if sum(visit) < len(node_type):
        yet_visit = [i for i in range(len(node_type)) if not visit[i]]
        print(f"다음 노드들을 방문하지 않았습니다. {yet_visit}")
        return 0

    return total_cost
