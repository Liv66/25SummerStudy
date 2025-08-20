# cwj_computational_study.py
import os
import re
import io
import json
import time
import glob
import contextlib
import pandas as pd
import sys
import multiprocessing as mp

# --- cwj_run 가져오기 ---
try:
    from cwj_main import cwj_run
except Exception:
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(this_dir)
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)
    try:
        from cwj.cwj_main import cwj_run
    except Exception:
        from cwj_main import cwj_run


# --- 로그 파싱 함수 ---
def parse_metrics_from_log(log_text: str):
    re_initial = re.compile(r"Initial\s+RMP\(LP\)\s*ObjVal\s*=\s*([0-9]+(?:\.[0-9]+)?)")
    re_total = re.compile(r"Total\s*Cost\s*:\s*([0-9]+(?:\.[0-9]+)?)")
    re_time = re.compile(r"실행\s*시간\s*:\s*([0-9]+(?:\.[0-9]+)?)\s*초")

    initial_vals = re_initial.findall(log_text)
    total_vals = re_total.findall(log_text)
    time_vals = re_time.findall(log_text)

    initial_obj = float(initial_vals[-1]) if initial_vals else None
    total_cost = float(total_vals[-1]) if total_vals else None
    runtime_s = float(time_vals[-1]) if time_vals else None

    return initial_obj, total_cost, runtime_s


# --- 워커: 별도 프로세스에서 실행 (stdout 캡처) ---
def _worker_run(problem_info, conn):
    buf = io.StringIO()
    start = time.time()
    with contextlib.redirect_stdout(buf):
        try:
            _ = cwj_run(problem_info)
        except Exception as e:
            print(f"[ERROR] {e}")
    end = time.time()
    log_text = buf.getvalue()
    elapsed = round(end - start, 6)
    conn.send({"log": log_text, "elapsed": elapsed})
    conn.close()


def run_one_instance(json_path: str, timeout_sec: int = 60):
    """단일 인스턴스를 timeout_sec 내에 실행. 결과: (initial_obj, total_cost, runtime_s, status)"""
    with open(json_path, "r", encoding="utf-8") as f:
        problem_info = json.load(f)

    parent_conn, child_conn = mp.Pipe(duplex=False)
    p = mp.Process(target=_worker_run, args=(problem_info, child_conn))
    p.start()
    p.join(timeout=timeout_sec)

    if p.is_alive():
        # 타임아웃: 프로세스 종료
        p.terminate()
        p.join()
        # 기록: TIMEOUT
        return None, None, float(timeout_sec), "TIMEOUT"

    # 정상 종료: 수신된 로그 파싱
    result = parent_conn.recv() if parent_conn.poll() else {"log": "", "elapsed": None}
    log_text = result.get("log", "")
    elapsed = result.get("elapsed", None)

    initial_obj, total_cost, runtime_s = parse_metrics_from_log(log_text)
    if runtime_s is None:
        runtime_s = elapsed

    return initial_obj, total_cost, runtime_s, "OK"


def run_all_instances(
    instance_dir: str = "/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/",
    output_csv: str = "/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/cwj/output/vrpb_results.csv",
    timeout_sec: int = 60
):
    json_files = sorted(glob.glob(os.path.join(instance_dir, "*.json")))
    if not json_files:
        print(f"[WARN] No JSON files found in: {instance_dir}")
        return

    rows = []
    for jp in json_files:
        print(f"[RUN] {os.path.basename(jp)} ...")
        initial_obj, total_cost, runtime_s, status = run_one_instance(jp, timeout_sec=timeout_sec)

        rows.append({
            "instance": os.path.basename(jp),
            "initial_rmp_lp_objval": initial_obj,
            "total_cost": total_cost,
            "runtime_sec": runtime_s,
            "status": status,  # OK 또는 TIMEOUT
        })

    # 출력 폴더 생성
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    df = pd.DataFrame(rows, columns=["instance", "initial_rmp_lp_objval", "total_cost", "runtime_sec", "status"])
    df.to_csv(output_csv, index=False)
    print(f"[DONE] Saved to CSV: {os.path.abspath(output_csv)}")


if __name__ == "__main__":
    # macOS/Windows 모두 동작하도록 spawn 권장
    mp.set_start_method("spawn", force=True)
    run_all_instances()