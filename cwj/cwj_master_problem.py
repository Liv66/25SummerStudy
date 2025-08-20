# cwj_master_problem.py

from gurobipy import Model, GRB, quicksum

def solve_master_problem(route_pool, node_types, dist_mat, K, relax=False):
    N = len(node_types)
    customer_indices = list(range(1, N))

    model = Model("VRPB_Master")
    model.setParam("OutputFlag", 1)

    z = []
    for idx, (route, cost) in enumerate(route_pool):
        vtype = GRB.CONTINUOUS if relax else GRB.BINARY
        z_var = model.addVar(vtype=vtype, obj=cost, name=f"z_{idx}")
        z.append(z_var)

    for i in customer_indices:
        model.addConstr(
            quicksum(z[j] for j, (route, _) in enumerate(route_pool) if i in set(route) - {0}) >= 1,
            name=f"visit_{i}"
        )

    model.addConstr(quicksum(z) <= K, name="vehicle_limit")

    model.ModelSense = GRB.MINIMIZE
    model.optimize()

    selected_routes = []
    duals = [0] * N
    missed_customers = set()

    if model.status == GRB.OPTIMAL:
        for j, z_j in enumerate(z):
            if z_j.X > 0.5:
                selected_routes.append(route_pool[j])
        for i in customer_indices:
            duals[i] = model.getConstrByName(f"visit_{i}").Pi
        duals[0] = model.getConstrByName("vehicle_limit").Pi
    elif model.status == GRB.INFEASIBLE:
        print("[ERROR] Master problem is infeasible!")
        model.computeIIS()
        model.write("infeasible.ilp")
        
        covered = set()
        for route, _ in route_pool:
            covered.update(route[1:-1])
        missed_customers = set(customer_indices) - covered

    return selected_routes, duals, missed_customers