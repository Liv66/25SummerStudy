import random


def genrate_point(nodes, NUM_LINEHAUL=10, NUM_BACKHAUL=5):

    for i in range(1, NUM_LINEHAUL + 1):
        nodes.append({
            'id': i,
            'type': 'linehaul',
            'x': random.randint(0, 100),
            'y': random.randint(0, 100),
            'demand': random.randint(5, 15) # 5~15 사이의 배송량
        })

    # 백홀 고객 정보 생성 (수거)
    for i in range(NUM_LINEHAUL + 1, NUM_LINEHAUL + NUM_BACKHAUL + 1):
        nodes.append({
            'id': i,
            'type': 'backhaul',
            'x': random.randint(0, 100),
            'y': random.randint(0, 100),
            'demand': random.randint(5, 15) # 5~15 사이의 수거량
        })
    return nodes

def generate_vrpb_parameters(NUM_LINEHAUL=10, NUM_BACKHAUL=5, NUM_VEHICLES=4, CAPACITY=50):

    # --------------------------------------------------
    # 2. 고객 및 차고지 데이터 생성
    # --------------------------------------------------
    # 노드(고객, 차고지) 정보를 담을 리스트
    # 0번 노드는 차고지로 사용
    nodes = []

    # 차고지(Depot) 정보 추가 (0번 노드)
    nodes.append({
        'id': 0,
        'type': 'depot',
        'x': 50,  # x 좌표
        'y': 50,  # y 좌표
        'demand': 0 # 수요량
    })

    nodes = genrate_point(nodes, NUM_LINEHAUL, NUM_BACKHAUL)
    # 전체 노드 수
    NUM_NODES = len(nodes)

    # --------------------------------------------------
    # 3. 비용 행렬(Cost Matrix) 계산
    # --------------------------------------------------
    # # 두 지점 간의 유클리드 거리를 계산하는 함수
    # def calculate_distance(node1, node2):
    #     return math.sqrt((node1['x'] - node2['x'])**2 + (node1['y'] - node2['y'])**2)

    # # 비용 행렬 초기화 (모든 값을 0으로)
    # cost_matrix = [[0] * NUM_NODES for _ in range(NUM_NODES)]

    # # 모든 노드 쌍에 대해 거리를 계산하여 비용 행렬 채우기
    # for i in range(NUM_NODES):
    #     for j in range(NUM_NODES):
    #         cost_matrix[i][j] = calculate_distance(nodes[i], nodes[j])

    # # --------------------------------------------------
    # # 4. 생성된 파라미터 확인
    # # --------------------------------------------------
    # print("### VRPB 문제 파라미터 ###\n")
    # print(f"총 노드 수: {NUM_NODES}")
    # print(f"차량 수: {NUM_VEHICLES}, 차량 용량: {CAPACITY}\n")

    # print("--- 고객 정보 (일부) ---")
    # for i in range(5):
    #     print(nodes[i])

    # print("\n--- 비용 행렬 (일부) ---")
    # for i in range(5):
    #     # 소수점 둘째 자리까지 반올림하여 출력
    #     print([round(cost, 2) for cost in cost_matrix[i][:]])
    
    return nodes