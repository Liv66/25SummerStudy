from matplotlib import pyplot as plt

def plot_vrpb(problem_info, best_result_route=[], title=''):
    nodes_coord = problem_info['node_coords']
    nodes_type = problem_info['node_types']
    for i in range(len(nodes_type)):
        if nodes_type[i] == 0:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=150, marker='s', color='black')
        elif nodes_type[i] == 1:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=35, color='blue')
        else:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=35, marker='^', color='red')

    for route in best_result_route:
        points_x = [nodes_coord[x][0] for x in route]
        points_y = [nodes_coord[x][1] for x in route]
        plt.plot(points_x, points_y, linestyle='-')

    plt.title(title)
    plt.xticks([])
    plt.yticks([])
    plt.show()

# 수동으로 입력한 route
best_result_route = [
    [0, 18, 15, 3, 8, 10, 9, 19, 17, 0], [0, 4, 16, 5, 12, 1, 2, 0]
]

# VRPB 인스턴스 로드 예시
import json
with open("/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/problem_20_0.7.json") as f:
    problem_info = json.load(f)

# 시각화 호출
plot_vrpb(problem_info, best_result_route, title="VRPB")