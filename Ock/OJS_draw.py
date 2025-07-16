import matplotlib.pyplot as plt

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