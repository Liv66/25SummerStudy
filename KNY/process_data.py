import pandas as pd

# --- 설정 ---
RAW_FILE = 'raw_results.csv'
PROCESSED_FILE = 'processed_results.csv'


def process_results():
    """
    raw_results.csv 파일을 읽어, 인스턴스별로 통계를 계산하고
    processed_results.csv 파일로 저장합니다.
    """
    print(f"'{RAW_FILE}' 파일을 읽는 중입니다...")

    # 1. 원본 CSV 파일 읽기
    try:
        df_raw = pd.read_csv(RAW_FILE)
    except FileNotFoundError:
        print(f"[오류] '{RAW_FILE}' 파일을 찾을 수 없습니다. 파일이 같은 폴더에 있는지 확인하세요.")
        return

    print("데이터 집계를 시작합니다...")

    # 2. 'instance'를 기준으로 그룹화하여 통계 계산
    #    - obj: 평균, 표준편차, 최소값, 최대값 계산
    #    - num_vehicle: 평균, 표준편차 계산
    #    - total_time: 평균 계산
    #    - std_dist: 평균 계산 (경로 균형도의 평균)
    df_processed = df_raw.groupby('instance').agg(
        obj_mean=('obj', 'mean'),
        obj_std=('obj', 'std'),
        obj_min=('obj', 'min'),
        obj_max=('obj', 'max'),
        vehicles_mean=('num_vehicle', 'mean'),
        vehicles_std=('num_vehicle', 'std'),
        time_mean=('total_time', 'mean'),
        route_balance_mean=('std_dist', 'mean')  # 경로 균형도(std_dist)의 평균
    ).round(2)  # 소수점 둘째 자리까지 반올림

    # 3. 인덱스를 다시 열로 변환
    df_processed = df_processed.reset_index()

    # 4. 가공된 데이터를 새로운 CSV 파일로 저장
    try:
        df_processed.to_csv(PROCESSED_FILE, index=False)
        print("-" * 30)
        print(f"🎉 성공! 가공된 데이터가 '{PROCESSED_FILE}' 파일로 저장되었습니다.")
        print("생성된 데이터 미리보기:")
        print(df_processed.head())
        print("-" * 30)
    except Exception as e:
        print(f"[오류] 파일을 저장하는 중 문제가 발생했습니다: {e}")


if __name__ == '__main__':
    process_results()