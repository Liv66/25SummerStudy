from cwj_utils import *
from cwj_initial_patterns import *
import gurobipy as gp
from gurobipy import GRB

# -----------------------------
# Master Problem (RMP) with Gurobi
# -----------------------------
def solve_master_problem_gurobi(route_pool, node_types, K, relax: bool = True,
                                print_lp_vars: bool = False, lp_print_top: int = 12,
                                customers_override=None):
    """
    Set Partitioning (vehicle limit):
      min sum c_r x_r
      s.t. sum_{r: i in r} x_r = 1  for all customers i in 'customers'
           sum_r x_r <= K
           x_r in {0,1} or [0,1] if relax=True

    Returns: (selected_solution, duals_dict, mu, model_obj, model)
    """

    # 고객 집합 정의 (잔여 고객만 덮도록)
    all_customers = [i for i, t in enumerate(node_types) if t in (1, 2)]
    customers = list(customers_override) if customers_override is not None else all_customers

    R = len(route_pool)
    m = gp.Model("VRPB_RMP")
    m.Params.OutputFlag = 0
    vtype = GRB.CONTINUOUS if relax else GRB.BINARY
    x = m.addVars(R, vtype=vtype, name="x", lb=0.0, ub=1.0)

    costs = [c for (_, c) in route_pool]
    m.setObjective(gp.quicksum(costs[r] * x[r] for r in range(R)), GRB.MINIMIZE)

    # Cover constraints only for 'customers'
    cov_constr = {}
    for i in customers:
        cov_constr[i] = m.addConstr(
            gp.quicksum(x[r] for r in range(R) if i in route_pool[r][0]) == 1,
            name=f"cover_{i}"
        )

    k_constr = m.addConstr(gp.quicksum(x[r] for r in range(R)) <= K, name="veh_count")

    m.optimize()
    status = m.Status
    if status != GRB.OPTIMAL and status != GRB.SUBOPTIMAL:
        # infeasible or other => relax branch returns infeasible
        if relax:
            return [], None, None, float('inf'), m
        else:
            return [], None, None, float('inf'), m

    if relax:
        duals = {i: cov_constr[i].Pi for i in customers}
        mu = k_constr.Pi

        if print_lp_vars:
            nz = []
            for r in range(R):
                val = m.getVarByName(f"x[{r}]").X
                if val > 1e-9:
                    nz.append((r, val))
            nz.sort(key=lambda t: -t[1])
            sum_x = sum(val for _, val in nz)
            print(f"[LP] Nonzero x count = {len(nz)}, sum(x) = {sum_x:.6f} (<= K={K})")
            show = nz if len(nz) <= lp_print_top else nz[:lp_print_top]
            for r, val in show:
                route, c = route_pool[r]
                print(f"[LP] x[{r}] = {val:.6f} | cost = {c:.1f} | route = {route}")
            if len(nz) > lp_print_top:
                print(f"[LP] ... and {len(nz) - lp_print_top} more")

        return [], duals, mu, m.objVal, m
    else:
        sol = []
        for r in range(R):
            if m.getVarByName(f"x[{r}]").X > 0.5:
                sol.append(route_pool[r])
        return sol, None, None, m.objVal, m