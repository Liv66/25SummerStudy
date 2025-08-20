import json
import random
import os, sys


from JHJ.JHJ_run import JHJ_run
from util import *


def JHJ_main(problem_info, time_limit=60, log=True):

    start = time.time()
    sol = JHJ_run(problem_info, time_limit)

    elapsed = round(time.time() - start,2)

    obj = check_feasible(problem_info, sol, elapsed, time_limit)
    plot_vrpb(problem_info, sol, f'obj : {obj} elapsed : {elapsed}')

    if log:
        print(obj, elapsed)

    return sol


if __name__ == '__main__':
    N_list = [70]
    P_list = [0.5]
    for n in N_list:
        for p in P_list:
            with open(f'C:/Users/hyeon/Desktop/study/25SummerStudy/instances/problem_{n}_{p}.json', 'r') as f:
                problem_info = json.load(f)

    sol = JHJ_main(problem_info)
