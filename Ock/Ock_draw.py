import matplotlib.pyplot as plt
import numpy as np


# 결과를 시각화하는 함수
def draw_routes(nodes, routes):
    plt.figure(figsize=(12, 12))
    
    # 1. 노드 그리기
    depot = nodes[0]
    linehaul_nodes = [node for node in nodes if node['type'] == 'linehaul']
    backhaul_nodes = [node for node in nodes if node['type'] == 'backhaul']

    # Depot
    plt.scatter(depot['x'], depot['y'], c='red', s=200, marker='*', label='Depot')
    # Linehaul 노드
    plt.scatter([n['x'] for n in linehaul_nodes], [n['y'] for n in linehaul_nodes], 
                c='blue', s=100, marker='o', label='Linehaul')
    # Backhaul 노드
    plt.scatter([n['x'] for n in backhaul_nodes], [n['y'] for n in backhaul_nodes], 
                c='green', s=100, marker='^', label='Backhaul')

    # 각 노드에 ID 텍스트 표시
    for node in nodes:
        plt.text(node['x'] + 1, node['y'] + 1, str(node['id']))

    # 2. 경로 그리기
    colors = plt.cm.get_cmap('gist_rainbow', len(routes))
    for i, route in enumerate(routes):
        route_color = colors(i)
        # 경로의 노드 좌표들을 순서대로 가져옴
        route_nodes = [nodes[node_id] for node_id in route]
        
        # 선으로 경로 연결
        plt.plot([n['x'] for n in route_nodes], [n['y'] for n in route_nodes], 
                 color=route_color, linewidth=2, linestyle='-',
                 label=f'Vehicle {i+1}')

    plt.title('VRPB Solution Visualization', fontsize=20)
    plt.xlabel('X Coordinate', fontsize=14)
    plt.ylabel('Y Coordinate', fontsize=14)
    plt.legend()
    plt.grid(True)
    plt.show()


def statistic_result(result, value):
    data_to_plot = []
    for input_group in result:
        extracted_values = [item[value] for item in input_group]
        data_to_plot.append(extracted_values)

    # 통계량 계산 및 출력
    print("--- 각 입력값 그룹별 통계량 ---")
    for i, values in enumerate(data_to_plot):
        print(f"\n입력값 그룹 {i+1}:")
        print(f"  평균 (Mean): {np.mean(values):.4f}")
        print(f"  중앙값 (Median): {np.median(values):.4f}")
        print(f"  표준편차 (Standard Deviation): {np.std(values):.4f}")
        print(f"  최소값 (Min): {np.min(values):.4f}")
        print(f"  최대값 (Max): {np.max(values):.4f}")
        print(f"  사분위수 (25%, 75%): {np.percentile(values, 25):.4f}, {np.percentile(values, 75):.4f}")

    # 박스 플롯 그리기
    plt.figure(figsize=(10, 6))
    plt.boxplot(data_to_plot, patch_artist=True, medianprops={'color': 'black'}) # patch_artist로 박스 색상 지정 가능

    # X축 라벨 설정 (입력값 그룹의 인덱스를 사용)
    # 만약 각 입력값에 대한 특정 이름이 있다면 labels 파라미터를 사용하여 지정할 수 있습니다.
    # 예: plt.boxplot(data_to_plot, labels=['Input A', 'Input B'], ...)
    plt.xticks(range(1, len(data_to_plot) + 1), [f'Input {i+1}' for i in range(len(data_to_plot))])

    plt.title('Performance Distribution per Input Value Group')
    plt.xlabel('Input Value Group')
    plt.ylabel('Performance Value (e.g., Cost, Time, etc.)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()