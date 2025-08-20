import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# --- 설정 ---
# process_data.py에서 생성한 LS 비교 결과 파일을 지정합니다.
PROCESSED_FILE = 'processed_results2.csv'
# 안정성(분포) 비교를 위해 원본 데이터 파일도 지정합니다.
RAW_FILE = 'raw_results.csv'

# process_data.py와 동일한 인스턴스 목록을 사용해야 합니다.
INSTANCES_TO_COMPARE = [
    "problem_50_0.5",
    "problem_50_0.85",
    "problem_100_0.5",
    "problem_100_0.7",
    "problem_100_0.85",
    "problem_150_0.5",
    "problem_150_0.85"
]


def create_ls_comparison_visualizations():
    """
    Local Search(LS) 실행 유무에 따른 성능 차이를 시각화합니다.
    """
    print("LS 효과 비교 시각화 스크립트를 시작합니다...")

    # 1. 가공된 데이터 파일 불러오기
    try:
        df_processed = pd.read_csv(PROCESSED_FILE)
    except FileNotFoundError:
        print(f"[오류] '{PROCESSED_FILE}' 파일을 찾을 수 없습니다.")
        print("이전 단계의 'process_data.py'를 먼저 실행하여 결과 파일을 생성했는지 확인하세요.")
        return

    # --- 시각화 1: 평균 성능 비교 (obj_mean) ---
    plt.figure(figsize=(15, 8))
    sns.barplot(data=df_processed, x='instance', y='obj_mean', hue='analysis_group', palette='coolwarm')
    plt.title('Local Search 유무에 따른 평균 성능 비교 (obj_mean)', fontsize=18, pad=20)
    plt.xlabel('인스턴스 (Instance)', fontsize=12)
    plt.ylabel('평균 obj 값 (낮을수록 좋음)', fontsize=12)
    plt.xticks(rotation=45, ha='right')
    plt.legend(title='LS Status')
    plt.tight_layout()

    # --- 시각화 2: 최고 성능 비교 (obj_min) ---
    plt.figure(figsize=(15, 8))
    sns.barplot(data=df_processed, x='instance', y='obj_min', hue='analysis_group', palette='RdYlBu')
    plt.title('Local Search 유무에 따른 최고 성능 비교 (obj_min)', fontsize=18, pad=20)
    plt.xlabel('인스턴스 (Instance)', fontsize=12)
    plt.ylabel('최소 obj 값 (낮을수록 좋음)', fontsize=12)
    plt.xticks(rotation=45, ha='right')
    plt.legend(title='LS Status')
    plt.tight_layout()

    # --- 시각화 3: 계산 효율성 비교 (time_mean) ---
    plt.figure(figsize=(15, 8))
    sns.barplot(data=df_processed, x='instance', y='time_mean', hue='analysis_group', palette='PRGn')
    plt.title('Local Search 유무에 따른 평균 계산 시간 비교 (time_mean)', fontsize=18, pad=20)
    plt.xlabel('인스턴스 (Instance)', fontsize=12)
    plt.ylabel('평균 계산 시간 (초)', fontsize=12)
    plt.xticks(rotation=45, ha='right')
    plt.legend(title='LS Status')
    plt.tight_layout()

    # --- 시각화 4: 알고리즘 안정성 비교 (Box Plot) ---
    try:
        print(f"안정성 분석을 위해 '{RAW_FILE}' 파일을 읽는 중입니다...")
        df_all_raw = pd.read_csv(RAW_FILE)

        # process_data.py에서 사용한 필터링 로직을 그대로 적용하여 원본 데이터 추출
        # LS_ON 데이터
        df_slice = df_all_raw.iloc[100:175].copy()
        df_ls_on = df_slice[df_slice['instance'].isin(INSTANCES_TO_COMPARE)].copy()
        df_ls_on['analysis_group'] = 'LS_ON'

        # LS_OFF 데이터
        df_method4 = df_all_raw[df_all_raw['method'] == 4].copy()
        df_ls_off = df_method4[df_method4['instance'].isin(INSTANCES_TO_COMPARE)].copy()
        df_ls_off['analysis_group'] = 'LS_OFF'

        df_raw_filtered = pd.concat([df_ls_on, df_ls_off])

        plt.figure(figsize=(15, 8))
        sns.boxplot(data=df_raw_filtered, x='instance', y='obj', hue='analysis_group', palette='BrBG')
        plt.title('Local Search 유무에 따른 안정성 비교 (obj 분포)', fontsize=18, pad=20)
        plt.xlabel('인스턴스 (Instance)', fontsize=12)
        plt.ylabel('obj 값 분포 (상자가 짧을수록 안정적)', fontsize=12)
        plt.xticks(rotation=45, ha='right')
        plt.legend(title='LS Status')
        plt.tight_layout()

    except FileNotFoundError:
        print(f"[경고] 안정성 비교를 위한 '{RAW_FILE}'을 찾을 수 없어 박스 플롯을 건너뜁니다.")
    except Exception as e:
        print(f"[오류] 박스 플롯 생성 중 문제가 발생했습니다: {e}")

    # 생성된 모든 그래프를 화면에 보여주기
    print("모든 그래프를 화면에 출력합니다...")
    plt.show()


if __name__ == '__main__':
    # 한글 폰트가 깨지지 않도록 설정
    try:
        plt.rcParams['font.family'] = 'Malgun Gothic'
        plt.rcParams['axes.unicode_minus'] = False  # 마이너스 기호 깨짐 방지
    except:
        plt.rcParams['font.family'] = 'AppleGothic'
        plt.rcParams['axes.unicode_minus'] = False  # 마이너스 기호 깨짐 방지

    create_ls_comparison_visualizations()
