from math import sqrt
import time
import matplotlib.pyplot as plt

def get_distance(nodes_coord):
    N = len(nodes_coord)
    return [[int(sqrt((nodes_coord[i][0] - nodes_coord[j][0]) ** 2 +
                      (nodes_coord[i][1] - nodes_coord[j][1]) ** 2)) for i in range(N)] for j in range(N)]

def two_opt(depot, nodes, dist_matrix, nodes_coord, show=False):
    improved = True
    result_route = [i for i in nodes] + [depot]
    st = time.time()
    result_cost = sum(dist_matrix[result_route[i]][result_route[i + 1]] for i in range(len(nodes)))
    while improved:
        improved = False
        for i in range(1, len(nodes) - 1):
            for j in range(i + 1, len(nodes)):
                before = (dist_matrix[result_route[i - 1]][result_route[i]] + dist_matrix[result_route[j]][result_route[j + 1]])
                after = (dist_matrix[result_route[i - 1]][result_route[j]] + dist_matrix[result_route[i]][result_route[j + 1]])
                if after - before < 0:
                    result_cost += after - before
                    result_route = result_route[:i] + list(reversed(result_route[i:j + 1])) + result_route[j + 1:]
                    improved = True
    if show:
        print("2opt------------------------------")
        print(f"cost : {result_cost}")
        print(f"route : {result_route}")
        print(f"소요시간 {time.time() - st}")
        plot(nodes_coord, result_route, f'ObjVal : {int(result_cost)}')
    return result_cost, result_route

def plot(nodes_coord, route, title=''):
    points_x = [nodes_coord[x][0] for x in route]
    points_y = [nodes_coord[x][1] for x in route]
    plt.scatter(points_x, points_y)
    plt.plot(points_x, points_y, linestyle='-', color='blue', label='Line')
    plt.title(title)
    plt.show()

def plot_cvrp(nodes_coord, best_result_route, title=''):
    for route in best_result_route:
        points_x = [nodes_coord[x][0] for x in route]
        points_y = [nodes_coord[x][1] for x in route]
        plt.scatter(points_x, points_y)
        plt.plot(points_x, points_y, linestyle='-', label='Line')
    plt.title(title)
    plt.show()

import time

def three_opt(depot, nodes, dist_matrix, nodes_coord=None, show=False):
    improved = True
    result_route = [depot] + nodes + [depot]  # depot을 경로 처음과 끝에 포함
    n = len(result_route)
    st = time.time()

    def calc_cost(route):
        return sum(dist_matrix[route[i]][route[i + 1]] for i in range(len(route) - 1))

    result_cost = calc_cost(result_route)

    while improved:
        improved = False
        for i in range(1, n - 5):        # i, j, k 위치는 depot 제외한 내부 노드 인덱스
            for j in range(i + 1, n - 3):
                for k in range(j + 1, n - 1):
                    A, B = result_route[i - 1], result_route[i]
                    C, D = result_route[j - 1], result_route[j]
                    E, F = result_route[k - 1], result_route[k]

                    # 현재 세 간선 비용
                    cost_before = dist_matrix[A][B] + dist_matrix[C][D] + dist_matrix[E][F]

                    # 7가지 3-opt 재배치 경우들
                    # 1) no change (건너뜀)
                    # 2) Case 1: reverse segment (i,j-1)
                    new_route = (
                        result_route[:i]
                        + result_route[i:j][::-1]
                        + result_route[j:k]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                    # 3) Case 2: reverse segment (j,k-1)
                    new_route = (
                        result_route[:i]
                        + result_route[i:j]
                        + result_route[j:k][::-1]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                    # 4) Case 3: reverse segments (i,j-1) and (j,k-1)
                    new_route = (
                        result_route[:i]
                        + result_route[i:j][::-1]
                        + result_route[j:k][::-1]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                    # 5) Case 4: swap segments (i,j-1) and (j,k-1) without reversing
                    new_route = (
                        result_route[:i]
                        + result_route[j:k]
                        + result_route[i:j]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                    # 6) Case 5: reverse (i,j-1), then swap with (j,k-1)
                    new_route = (
                        result_route[:i]
                        + result_route[j:k]
                        + result_route[i:j][::-1]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                    # 7) Case 6: reverse (j,k-1), then swap with (i,j-1)
                    new_route = (
                        result_route[:i]
                        + result_route[j:k][::-1]
                        + result_route[i:j]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                    # 8) Case 7: reverse both segments, then swap
                    new_route = (
                        result_route[:i]
                        + result_route[j:k][::-1]
                        + result_route[i:j][::-1]
                        + result_route[k:]
                    )
                    cost_after = calc_cost(new_route)

                    if cost_after < result_cost:
                        result_route = new_route
                        result_cost = cost_after
                        improved = True
                        break

                if improved:
                    break
            if improved:
                break

    if show:
        print("3-opt finished")
        print(f"cost : {result_cost}")
        print(f"route : {result_route}")
        print(f"elapsed time : {time.time() - st:.3f} sec")
        if nodes_coord is not None:
            # plot 함수가 있으면 호출 (필요시 직접 구현하세요)
            plot(nodes_coord, result_route, f'ObjVal : {int(result_cost)}')

    # depot 빼고 리턴하려면
    return result_cost, result_route[1:-1]
