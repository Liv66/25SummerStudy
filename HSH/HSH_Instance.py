import random, math
import numpy as np
import matplotlib.pyplot as plt

def generate_instance(num_nodes: int, linehaul_ratio: float, capacity: int, num_vehicles: int, seed: int = 42):
    assert 25 <= num_nodes <= 150, "노드 수: 25~150"
    assert linehaul_ratio in [0.5, 0.66, 0.8], "linehaul 비율: 50%, 66%, 80%"
    assert 3 <= num_vehicles <= 12, "차량 대수: 3~12"

    random.seed(seed)
    np.random.seed(seed)

    # 노드 좌표 생성
    node_coords = [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(num_nodes)]
    depot_coord = (12000, 16000)
    all_coords = node_coords + [depot_coord]
    depot_index = num_nodes  # depot이 마지막 index

    # 수요 생성
    demands = [max(1, int(np.random.normal(500, 200))) for _ in range(num_nodes)] # 1: 음수 방지

    # linehaul, backhaul 노드 지정
    num_linehaul = int(num_nodes * linehaul_ratio)
    node_types = ['linehaul'] * num_linehaul + ['backhaul'] * (num_nodes - num_linehaul)
    random.shuffle(node_types) # linehaul, backhaul 노드 랜덤 배치

    # 거리 행렬 생성
    def euclidean(p1, p2):
        return int(math.hypot(p1[0] - p2[0], p1[1] - p2[1]))

    # 전체 노드들 간 거리 계산(depot 포함), (num_nodes + 1) x (num_nodes + 1) 거리 행렬
    dist_matrix = [
        [euclidean(all_coords[i], all_coords[j]) for j in range(num_nodes + 1)]
        for i in range(num_nodes + 1)
    ]

    # 인스턴스
    instance = {
        'num_nodes': num_nodes,
        'num_vehicles': num_vehicles,
        'capacity': capacity,
        'node_coords': node_coords,
        'depot_coord': depot_coord,
        'depot_index': depot_index,
        'demands': demands,
        'node_types': node_types,
        'dist_matrix': dist_matrix
    }

    return instance

def plot_nodes(instance):
    node_coords = instance['node_coords']
    node_types = instance['node_types']
    depot_coord = instance['depot_coord']

    # 좌표 구분
    linehaul_x = [node_coords[i][0] for i in range(instance['num_nodes']) if node_types[i] == 'linehaul']
    linehaul_y = [node_coords[i][1] for i in range(instance['num_nodes']) if node_types[i] == 'linehaul']

    backhaul_x = [node_coords[i][0] for i in range(instance['num_nodes']) if node_types[i] == 'backhaul']
    backhaul_y = [node_coords[i][1] for i in range(instance['num_nodes']) if node_types[i] == 'backhaul']

    depot_x, depot_y = depot_coord

    # 시각화
    plt.figure(figsize=(8, 10))
    plt.scatter(linehaul_x, linehaul_y, color='red', label='linehaul', marker='o')
    plt.scatter(backhaul_x, backhaul_y, color='blue', label='backhaul', marker='^')
    plt.scatter(depot_x, depot_y, color='black', label='depot', marker='s', s=200)

    plt.title('VRPB Node Plot')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.show()

# 예시 실행
if __name__ == '__main__':
    instance = generate_instance(
        num_nodes = 50,
        linehaul_ratio = 0.66,
        capacity = 5000,
        num_vehicles = 6
    )

    print("[Instance]")
    print(f"노드 수: {instance['num_nodes']}")
    print(f"차량 대수: {instance['num_vehicles']}")
    print(f"linehaul 노드 수: {instance['node_types'].count('linehaul')}")
    print(f"backhaul 노드 수: {instance['node_types'].count('backhaul')}")
    print(f"depot 위치: {instance['depot_coord']}")
    print(f"0번 노드 수요, 좌표, 유형: {instance['demands'][0]}, {instance['node_coords'][0]}, {instance['node_types'][0]}")
    print(f"거리(0->1): {instance['dist_matrix'][0][1]}")

    plot_nodes(instance)