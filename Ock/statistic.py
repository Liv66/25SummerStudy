import pandas as pd
import json

# 사용자께서 알려주신 파일 경로
file_path = 'results/summary_of_all_results.jsonl'

# JSONL 파일을 한 줄씩 읽어 리스트에 추가
data = []
with open(file_path, 'r') as f:
    for line in f:
        data.append(json.loads(line))

# 리스트를 pandas DataFrame으로 변환
df = pd.DataFrame(data)

# 'problem_name'으로 그룹화하고 'final_cost'의 통계량 (min, max, mean) 계산
statistics = df.groupby('problem_name')['final_cost'].agg(['min', 'max', 'mean']).reset_index()

# 보기 편하도록 컬럼 이름 변경
statistics.rename(columns={'min': '최소값', 'max': '최대값', 'mean': '평균값', 'problem_name':'문제 이름'}, inplace=True)

# 결과 출력
print("문제별 final_cost 통계량:")
print(statistics)

# 결과를 CSV 파일로 저장
statistics.to_csv('final_cost_statistics.csv', index=False)