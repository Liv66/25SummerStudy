import pandas as pd

# --- 설정 ---
RAW_FILE = 'raw_results.csv'
PROCESSED_FILE = 'processed_results2.csv' # 저장할 파일 이름 변경

# 비교할 7개의 인스턴스 목록
INSTANCES_TO_COMPARE = [
    "problem_50_0.5",
    "problem_50_0.85",
    "problem_100_0.5",
    "problem_100_0.7",
    "problem_100_0.85",
    "problem_150_0.5",
    "problem_150_0.85"
]


def process_ls_comparison():
    """
    Local Search(LS) 실행 유무에 따른 성능 차이를 비교 분석하고
    결과를 processed_results2.csv 파일로 저장합니다.
    """
    print(f"'{RAW_FILE}' 파일을 읽는 중입니다...")

    try:
        # 1. 원본 CSV 파일 전체를 읽기
        df_all_raw = pd.read_csv(RAW_FILE)
    except FileNotFoundError:
        print(f"[오류] '{RAW_FILE}' 파일을 찾을 수 없습니다. 파일이 같은 폴더에 있는지 확인하세요.")
        return

    print("데이터 필터링을 시작합니다...")

    # 2-1. Local Search 실행 데이터 (LS_ON) 필터링
    # 101~175행(인덱스 100~174)을 먼저 선택
    df_slice = df_all_raw.iloc[100:175].copy()
    # 그 안에서 7개 인스턴스만 필터링
    df_ls_on = df_slice[df_slice['instance'].isin(INSTANCES_TO_COMPARE)].copy()
    # 분석 그룹을 'LS_ON'으로 명시
    df_ls_on['analysis_group'] = 'LS_ON'
    print(f"Local Search ON 데이터 처리 완료: {len(df_ls_on)}개의 행을 추출했습니다.")

    # 2-2. Local Search 미실행 데이터 (LS_OFF) 필터링
    # 전체 데이터에서 method가 4인 데이터만 추출
    df_method4 = df_all_raw[df_all_raw['method'] == 4].copy()
    # 그 안에서 7개 인스턴스만 필터링
    df_ls_off = df_method4[df_method4['instance'].isin(INSTANCES_TO_COMPARE)].copy()
    # 분석 그룹을 'LS_OFF'으로 명시
    df_ls_off['analysis_group'] = 'LS_OFF'
    print(f"Local Search OFF 데이터 처리 완료: {len(df_ls_off)}개의 행을 추출했습니다.")

    # 3. 필터링된 두 종류의 데이터를 하나로 합치기
    df_combined = pd.concat([df_ls_on, df_ls_off], ignore_index=True)

    if df_combined.empty:
        print("[오류] 처리할 데이터가 없습니다. 원본 파일의 내용을 다시 확인해주세요.")
        return

    print("데이터 집계를 시작합니다...")

    # 4. 'analysis_group'과 'instance'를 기준으로 핵심 지표 통계 계산
    df_processed = df_combined.groupby(['analysis_group', 'instance']).agg(
        obj_mean=('obj', 'mean'),
        obj_std=('obj', 'std'),
        obj_min=('obj', 'min'),
        time_mean=('total_time', 'mean')
    ).round(2)

    df_processed = df_processed.reset_index()

    # 5. 최종 결과를 'processed_results2.csv' 파일로 저장
    try:
        df_processed.to_csv(PROCESSED_FILE, index=False)
        print("-" * 30)
        print(f"🎉 성공! LS 비교 분석 결과가 '{PROCESSED_FILE}' 파일로 저장되었습니다.")
        print("생성된 데이터 미리보기:")
        print(df_processed)
        print("-" * 30)
    except Exception as e:
        print(f"[오류] 파일을 저장하는 중 문제가 발생했습니다: {e}")


if __name__ == '__main__':
    process_ls_comparison()
