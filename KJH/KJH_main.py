from KJH.KJH_vrpb import *
import json
from pathlib import Path

from util import plot_cvrp

def KJH_main(problem_info):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    c = Construction(K, node_type, node_demand, capa, dist_mat)
    c.construct()
    sol = [route.hist for route in c.routes]
    return sol

# 2) JSON 로드 + 실행 + 시각화 랩퍼
def run_kjh_problem(json_path: Path):
    with open(json_path, "r", encoding="utf-8") as f:
        problem_info = json.load(f)

    routes = KJH_main(problem_info)

    # ── ① VRPB obj 계산 -----------------------------
    dist = problem_info['dist_mat']  # 거리 행렬
    obj = sum(  # 총 이동거리
        dist[r[i]][r[i + 1]] for r in routes for i in range(len(r) - 1)
    )
    print(f"[INFO] VRPB objective = {obj:.1f}")
    # -----------------------------------------------

    # 좌표가 있으면 플롯
    if 'node_coords' in problem_info:
        plot_cvrp(problem_info['node_coords'], routes,
                  title=f"VRPB obj: {obj:.1f}")

    return routes, obj          # ← KNY 쪽과 맞추려면 이렇게 반환

if __name__ == "__main__":
    ROOT = Path(__file__).resolve().parents[1]          # 프로젝트 루트
    PROBLEM_JSON = ROOT / "instances" / "problem_100_0.7.json"

    run_kjh_problem(PROBLEM_JSON)                       # ← 이것만 호출
