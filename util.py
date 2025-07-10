from ortools.linear_solver import pywraplp


def bin_packing(weights, capa, log=False):
    num_items = len(weights)
    num_bins = num_items  # 최대 필요 bin 수 (각 아이템이 개별 bin에 들어갈 수도 있으므로)

    solver = pywraplp.Solver.CreateSolver("SCIP")
    if not solver:
        print("Solver를 생성할 수 없습니다.")
        return

    # 변수: x[i][j] = 아이템 i가 bin j에 배정되었는지 여부 (0 또는 1)
    x = {}
    for i in range(num_items):
        for j in range(num_bins):
            x[i, j] = solver.BoolVar(f'x_{i}_{j}')

    # 변수: y[j] = bin j가 사용되었는지 여부 (0 또는 1)
    y = {}
    for j in range(num_bins):
        y[j] = solver.BoolVar(f'y_{j}')

    # 제약조건 1: 각 아이템은 정확히 하나의 bin에만 배정되어야 함
    for i in range(num_items):
        solver.Add(solver.Sum(x[i, j] for j in range(num_bins)) == 1)

    # 제약조건 2: 각 bin의 총 무게는 bin 용량을 넘을 수 없음
    for j in range(num_bins):
        solver.Add(
            solver.Sum(weights[i] * x[i, j] for i in range(num_items)) <= capa * y[j]
        )

    # 목적함수: 사용된 bin 수 최소화
    solver.Minimize(solver.Sum(y[j] for j in range(num_bins)))

    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        if log:
            print(f'최소 사용 bin 수: {round(solver.Objective().Value())}')
            for j in range(num_bins):
                if y[j].solution_value() > 0.5:
                    bin_items = [i for i in range(num_items) if x[i, j].solution_value() > 0.5]
                    total_weight = sum(weights[i] for i in bin_items)
                    print(f'Bin {j}: items={bin_items}, total_weight={total_weight}')
        return round(solver.Objective().Value())
    else:
        print('해를 찾을 수 없습니다.')
        print(f'weights : {weights}')
        print(f'capa : {capa}')
        quit()

