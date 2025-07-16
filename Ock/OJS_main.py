import OJS_parameters as para
import OJS_heuristic as heu
from OJS_draw import draw_routes
from OJS_heuristic import destroy_solution
from OJS_heuristic import repair_solution
from OJS_ALNS import ALNS
import time

# Generate VRPB parameters
# This will create the nodes and cost matrix needed for the VRPB model
# 입력값으로는 NUM_LINEHAUL, NUM_BACKHAUL, NUM_VEHICLES, CAPACITY를 받는다.
# 이 함수는 노드와 비용 행렬을 생성하여 VRPB 모델에 필요한 파라미터를 반환한다.
NUM_LINEHAUL = 75  # 라인홀(배송) 고객 수
NUM_BACKHAUL = 75  # 백홀(수거) 고객 수
NUM_VEHICLES = 20  # 전체 차량 수
CAPACITY = 50      # 차량 한 대의 최대 용량


nodes = para.generate_vrpb_parameters(NUM_LINEHAUL, NUM_BACKHAUL, NUM_VEHICLES, CAPACITY)

# 2. 초기 해 생성
initial_routes = heu.init_solution(nodes, NUM_VEHICLES, CAPACITY)
initial_cost = heu.calculate_total_cost(nodes=nodes, solution=initial_routes)
print(f"초기 해 생성 완료. 비용: {initial_cost:.2f}")
# draw_routes(nodes, initial_routes)
# 3. 파괴 및 재구성 객체 생성
destroyer = destroy_solution(nodes=nodes, NUM_VEHICLES=NUM_VEHICLES, CAPACITY=CAPACITY)
repairer = repair_solution(nodes=nodes, NUM_VEHICLES=NUM_VEHICLES, CAPACITY=CAPACITY, NUM_LINEHAUL=NUM_LINEHAUL, NUM_BACKHAUL=NUM_BACKHAUL)

# 4. ALNS 솔버 생성 및 실행
# ALNS 파라미터 설정
iterations = 10000       # 총 반복 횟수
start_temperature = 100 # 초기 온도
cooling_rate = 0.999    # 냉각 비율

solver = ALNS(
    initial_solution=initial_routes,
    nodes=nodes,
    destroyer=destroyer,
    repairer=repairer
)

start_time = time.time()
# ALNS 실행
best_routes, best_cost = solver.run(iterations, start_temperature, cooling_rate)

end_time = time.time()
elapsed_time = end_time - start_time

print(f"\n--- 최종 결과 ---")
print(f"총 실행 시간: {elapsed_time:.2f}초") # <<<!!! 4. 측정된 실행 시간 출력 !!!>>>
print(f"초기 비용: {initial_cost:.2f}")
print(f"최종 비용: {best_cost:.2f}")
print(f"개선율: {(initial_cost - best_cost) / initial_cost * 100:.2f}%")
draw_routes(nodes, best_routes)
print("최적 경로:")
print(best_routes)
