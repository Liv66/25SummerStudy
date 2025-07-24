from HSH_GeneratePool_NI_3 import run_pooling_loop
from HSH_SP import run_set_partitioning, validate_solution
from HSH_loader import load_instance_from_json
import random, time

def HSH_main(problem_info):

    pool = run_pooling_loop(problem_info, duration_seconds = 59)
    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")
    selected_routes = run_set_partitioning(problem_info, pool)
    end_time = time.time()
    print(f"\n[전체 알고리즘 실행 시간] {end_time - start_time:.2f}초")
    if selected_routes:
        validate_solution(instance, selected_routes)

if __name__ == "__main__":
    random.seed(42)
    instance = load_instance_from_json(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_20_0.7.json")
    instance["node_demands"] = [abs(d) for d in instance["node_demands"]]
    start_time = time.time()
    HSH_main(instance)