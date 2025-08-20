import json
import time

from JHJ.JHJ_run import JHJ_run
from JHJ.JHJ_util import *


def JHJ_main(problem_info, time_limit=60, log=False):

    start = time.time()
    sol = JHJ_run(problem_info, time_limit)

    elapsed = round(time.time() - start,2)

    obj = check_feasible(problem_info, sol, elapsed, time_limit)
    plot_vrpb(problem_info, sol, f'obj : {obj} elapsed : {elapsed}')

    if log:
        print(obj, elapsed)

    return obj, elapsed, sol


if __name__ == '__main__':
    N_list = [50]
    P_list = [0.5]

    results_list = []

    for n in N_list:
        for p in P_list:
            with open(f'C:/Users/hyeon/Desktop/study/25SummerStudy/instances/problem_{n}_{p}.json', 'r') as f:
                problem_info = json.load(f)

            instance_name = f'problem_{n}_{p}'
            obj, elapsed, sol = JHJ_main(problem_info)

            # result_data = {
            #     'problem_instance': instance_name, 'obj': obj, 'time': elapsed
            # }

            # results_list.append(result_data)

    # df_results = pd.DataFrame(results_list)
    #
    # df_results.to_csv('raw_results.csv', index=False, encoding='utf-8-sig')
