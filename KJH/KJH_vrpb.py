import random


class Route:
    def __init__(self):
        self.hist = []
        self.line_load = 0
        self.back_load = 0
        self.use = False
        self.line_idx = 1
        self.cost = 0


class Construction:
    def __init__(self, K, node_type, node_demand, capa, dist_mat):
        self.K = K
        self.capa = capa
        self.node_type = node_type
        self.node_demand = node_demand
        self.dist_mat = dist_mat
        self.routes = []

    def parallel_insertion(self, routes, cl, isLine):
        cheapestInsertion = random.random() < 0.5
        gamma = random.randint(0, 34) / 20.0

        while cl:
            foundClient = False
            best_insertion_cost = 9999999
            best_route = -1
            best_node = 999
            best_pos = 0
            node_where = 0
            for idx, i in enumerate(cl):
                for k in range(len(routes)):
                    load = routes[k].line_load if isLine else routes[k].back_load
                    if load + self.node_demand[i] > self.capa:
                        continue
                    for j in range(routes[k].line_idx, len(routes[k].hist)):
                        l, r = routes[k].hist[j - 1], routes[k].hist[j]
                        if cheapestInsertion:
                            # 정교한 비용, Cheapest Feasible Insertion Criterion
                            insertion_cost = self.dist_mat[l][i] + self.dist_mat[i][r] - self.dist_mat[l][
                                r] - 2 * gamma * \
                                             self.dist_mat[0][i]
                        else:
                            # 가까운 거리 비용, Nearest Feasible Insertion Criterion
                            insertion_cost = self.dist_mat[l][i]

                        if insertion_cost < best_insertion_cost:
                            best_insertion_cost = insertion_cost
                            best_node = i
                            best_pos = j
                            foundClient = True
                            best_route = k
                            node_where = idx

            if foundClient:
                routes[best_route].hist.insert(best_pos, best_node)
                if isLine:
                    routes[best_route].line_load += self.node_demand[best_node]
                else:
                    routes[best_route].back_load += self.node_demand[best_node]
                cl.pop(node_where)
            else:
                break

        return len(cl)

    def sequential_insertion(self, routes, cl, isLine):
        cheapestInsertion = random.random() < 0.5
        gamma = random.randint(0, 34) / 20.0
        routes_full = [0] * len(routes)

        k = 0
        while cl:
            foundClient = False
            best_insertion_cost = 9999999
            best_node = 999
            best_pos = 0
            node_where = 0
            load = routes[k].line_load if isLine else routes[k].back_load

            for idx, node in enumerate(cl):
                if load + self.node_demand[node] > self.capa:
                    continue
                for pos in range(routes[k].line_idx, len(routes[k].hist)):
                    pre, suc = routes[k].hist[pos - 1], routes[k].hist[pos]
                    if cheapestInsertion:
                        # 정교한 비용, Cheapest Feasible Insertion Criterion
                        insertion_cost = self.dist_mat[pre][node] + self.dist_mat[node][suc] - self.dist_mat[pre][suc] - 2 * gamma * \
                                         self.dist_mat[0][node]
                    else:
                        # 가까운 거리 비용, Nearest Feasible Insertion Criterion
                        insertion_cost = self.dist_mat[pre][node]

                    if insertion_cost < best_insertion_cost:
                        best_insertion_cost = insertion_cost
                        best_node = node
                        best_pos = pos
                        foundClient = True
                        node_where = idx
            if foundClient:
                routes[k].hist.insert(best_pos, best_node)
                if isLine:
                    routes[k].line_load += self.node_demand[best_node]
                else:
                    routes[k].back_load += self.node_demand[best_node]
                cl.pop(node_where)
            else:
                routes_full[k] = 1

            if sum(routes_full) == len(routes):
                break

            if k == self.K - 1:
                k = -1
            k += 1
        return len(cl)

    # insertion strategy, 경로들에 어떤 방식으로 insertion 할 것인가
    def sequential_strategy(self):
        routes = [Route() for _ in range(self.K)]

        cl_line = []
        cl_back = []
        for i in range(1, len(self.node_type)):
            if self.node_type[i] == 1:
                cl_line.append(i)
            else:
                cl_back.append(i)
        random.shuffle(cl_line)
        random.shuffle(cl_back)

        for k in range(self.K):
            if not cl_line:
                routes[k].hist = [0, 0]
                continue
            c = cl_line.pop()
            routes[k].hist = [0, c, 0]
            routes[k].line_load += self.node_demand[c]

        line_flag = self.sequential_insertion(routes, cl_line, True)
        if line_flag:
            return False

        used_route = []
        for k in range(self.K):
            if routes[k].line_load > 0:
                routes[k].use = True
                routes[k].line_idx = len(routes[k].hist) - 1
                used_route.append(routes[k])

        back_flag = self.sequential_insertion(used_route, cl_back, False)
        if back_flag:
            return False

        for k in range(self.K):
            pre = 0
            for nxt in range(1, len(routes[k].hist)):
                routes[k].cost += self.dist_mat[routes[k].hist[pre]][routes[k].hist[nxt]]
                pre = nxt

        self.routes = routes

        return True

    def parallel_strategy(self):
        routes = [Route() for _ in range(self.K)]

        cl_line = []
        cl_back = []
        for i in range(1, len(self.node_type)):
            if self.node_type[i] == 1:
                cl_line.append(i)
            else:
                cl_back.append(i)
        random.shuffle(cl_line)
        random.shuffle(cl_back)

        for k in range(self.K):
            if not cl_line:
                routes[k].hist = [0, 0]
                continue
            c = cl_line.pop()
            routes[k].hist = [0, c, 0]
            routes[k].line_load += self.node_demand[c]

        line_flag = self.parallel_insertion(routes, cl_line, True)
        if line_flag:
            return False

        used_route = []
        for k in range(self.K):
            if routes[k].line_load > 0:
                routes[k].use = True
                routes[k].line_idx = len(routes[k].hist) - 1
                used_route.append(routes[k])

        back_flag = self.parallel_insertion(used_route, cl_back, False)
        if back_flag:
            return False

        for k in range(self.K):
            pre = 0
            for nxt in range(1, len(routes[k].hist)):
                routes[k].cost += self.dist_mat[routes[k].hist[pre]][routes[k].hist[nxt]]
                pre = nxt

        self.routes = routes

        return True

    def construct(self):
        while True:
            constructionType = random.random() < 0.7
            if constructionType:
                foundSolution = self.parallel_strategy()
            else:
                foundSolution = self.sequential_strategy()
            if foundSolution:
                break

