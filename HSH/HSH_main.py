# HSH_main: main
from HSH.HSH_GeneratePool import run_pooling_loop
from HSH.HSH_SP import run_set_partitioning, validate_solution
from HSH.HSH_loader import load_instance_from_json
import random, time

def HSH_run(problem_info):

    pool = run_pooling_loop(problem_info, duration_seconds = 50)
    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")
    selected_routes = run_set_partitioning(problem_info, pool)
    end_time = time.time()
    return selected_routes

if __name__ == "__main__":
    random.seed(42)
    instance = load_instance_from_json(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_130_0.85.json")
    instance["node_demands"] = [abs(d) for d in instance["node_demands"]]
    start_time = time.time()
    HSH_run(instance)