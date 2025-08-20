import gurobipy as gp
from gurobipy import GRB
import time

# 다른 모듈에서 함수 및 클래스 import
from LSJ_util import VRPColumn, Label, get_route_type, calculate_path_cost, is_valid_vrpb_route
from LSJ_heuristics import generate_multi_start_initial_columns

def split_to_line_back_pools(routes, nodes, depot=0):
    line_pool, back_pool = set(), set()
    for route in routes:
        if len(route) <= 2: continue
        line_part, back_part, is_back_started = [], [], False
        for node_idx in route[1:-1]:
            if nodes[node_idx]['type'] == 'backhaul': is_back_started = True
            (back_part if is_back_started else line_part).append(node_idx)
        if line_part: line_pool.add(tuple([depot] + line_part))
        if back_part: back_pool.add(tuple([depot] + back_part))
    return list(line_pool), list(back_pool)

def solve_pricing_spp_with_labeling(nodes, dist_matrix, capacity, duals, customer_type, visit_counts, penalty_factor, beam_width=10, neighborhood_size=30):
    depot = 0
    customer_indices = {i for i, n in enumerate(nodes) if n['type'] == customer_type}
    all_relevant_nodes = [depot] + list(customer_indices)
    nearest_neighbors = {u: sorted([v for v in customer_indices if v != u], key=lambda v: dist_matrix[u][v])[:neighborhood_size] for u in all_relevant_nodes}
    labels = {i: [] for i in all_relevant_nodes}
    labels[depot].append(Label(cost=0.0, load=0.0, node=depot, path=[depot]))
    for _ in range(25):
        new_labels_this_iteration = {i: [] for i in all_relevant_nodes}
        for u in all_relevant_nodes:
            if not labels[u]: continue
            for v in nearest_neighbors.get(u, []):
                for label_u in labels[u]:
                    if v not in label_u.path:
                        new_load = label_u.load + nodes[v]['demand']
                        if new_load <= capacity:
                            penalty = penalty_factor * visit_counts.get(v, 0)
                            reduced_cost_arc = dist_matrix[u][v] - duals.get(v, 0) + penalty
                            new_cost = label_u.cost + reduced_cost_arc
                            new_path = label_u.path + [v]
                            new_label = Label(cost=new_cost, load=new_load, node=v, path=new_path)
                            new_labels_this_iteration[v].append(new_label)
        for v in all_relevant_nodes:
            if not new_labels_this_iteration[v]: continue
            new_labels_this_iteration[v].sort(key=lambda l: (l.cost, l.load))
            for new_label in new_labels_this_iteration[v]:
                if not any(el.cost <= new_label.cost and el.load <= new_label.load for el in labels[v]):
                    surviving_labels = [el for el in labels[v] if not (new_label.cost <= el.cost and new_label.load <= el.load)]
                    surviving_labels.append(new_label)
                    surviving_labels.sort(key=lambda l: (l.cost, l.load))
                    labels[v] = surviving_labels[:beam_width]
    negative_rc_paths = []
    for node_idx, node_labels in labels.items():
        if node_idx == depot: continue
        for label in node_labels:
            if label.cost < -1e-6:
                negative_rc_paths.append(tuple(label.path))
    return list(set(negative_rc_paths))

def solve_subproblem_hybrid(line_pool, back_pool, nodes, dist_matrix, capacity, duals, visit_counts, penalty_factor, permanent_column_pool):
    v_dual = duals.get('vehicle', 0)
    cols_from_pool = []
    for col in permanent_column_pool:
        recalculated_rc = col.cost - sum(duals.get(c, 0) for c in col.path[1:-1]) - v_dual
        if recalculated_rc < -1e-6:
            new_col = VRPColumn(col.path, col.cost, col.route_type); new_col.reduced_cost = recalculated_rc
            cols_from_pool.append(new_col)
    new_line_paths = solve_pricing_spp_with_labeling(nodes, dist_matrix, capacity, duals, 'linehaul', visit_counts, penalty_factor, beam_width=5, neighborhood_size=20)
    new_back_paths = solve_pricing_spp_with_labeling(nodes, dist_matrix, capacity, duals, 'backhaul', visit_counts, penalty_factor, beam_width=5, neighborhood_size=20)
    updated_line_pool = list(set(line_pool) | set(new_line_paths)); updated_back_pool = list(set(back_pool) | set(new_back_paths))
    def calculate_fragment_rc(path):
        cost = calculate_path_cost(path, dist_matrix)
        rc = cost - sum(duals.get(c, 0) for c in path[1:])
        return rc
    line_paths_with_rc = sorted([(p, calculate_fragment_rc(p)) for p in updated_line_pool], key=lambda x: x[1])
    top_line_paths = [path for path, rc in line_paths_with_rc[:200]]
    back_paths_with_rc = sorted([(p, calculate_fragment_rc(p)) for p in updated_back_pool], key=lambda x: x[1])
    top_back_paths = [path for path, rc in back_paths_with_rc[:200]]
    cols_from_combination = []; added_path_tuples = set()
    candidate_paths = [list(l_path) + [0] for l_path in top_line_paths]
    for l_path in top_line_paths:
        for b_path in top_back_paths:
            candidate_paths.append(list(l_path) + list(b_path)[1:] + [0])
    for path in candidate_paths:
        path_tuple = tuple(path)
        if path_tuple in added_path_tuples: continue
        added_path_tuples.add(path_tuple)
        if is_valid_vrpb_route(path, nodes, capacity):
            cost = calculate_path_cost(path, dist_matrix)
            r_cost = cost - sum(duals.get(c, 0) for c in path[1:-1]) - v_dual
            if r_cost < -5:
                col = VRPColumn(path, cost, get_route_type(path, nodes)); col.reduced_cost = r_cost
                cols_from_combination.append(col)
    all_candidate_cols = cols_from_pool + cols_from_combination
    if not all_candidate_cols: return []
    unique_cols = list({tuple(c.path): c for c in all_candidate_cols}.values())
    unique_cols.sort(key=lambda c: c.reduced_cost)
    return unique_cols[:500]

def solve_vrp_with_column_generation(nodes, dist_matrix, num_vehicles, capacity, instance_data, total_time_limit=61, final_mip_time_limit=7):
    start_time = time.time()
    cg_time_limit = total_time_limit - final_mip_time_limit
    cg_end_time = start_time + cg_time_limit
    num_customers = len(nodes) - 1

    initial_columns, a_valid_initial_solution = generate_multi_start_initial_columns(nodes, dist_matrix, num_vehicles, capacity, num_starts=100)
    if not initial_columns:
        print("\n!!! Aborting: Failed to generate initial columns. !!!")
        return []
    
    all_columns = initial_columns[:]
    permanent_column_pool = {tuple(c.path): c for c in initial_columns}
    initial_routes = [col.path for col in initial_columns]
    line_pool, back_pool = split_to_line_back_pools(initial_routes, nodes, 0)
    customer_visit_counts = {i: 0 for i in range(1, num_customers + 1)}
    PENALTY_FACTOR = 0.5; DECAY_FACTOR = 0.95
    master_model = gp.Model("VRP_Master"); master_model.setParam('OutputFlag', 0)
    lambda_vars = master_model.addVars(len(all_columns), vtype=GRB.CONTINUOUS, obj=[c.cost for c in all_columns], name="lambda")
    cust_constraints = {i: master_model.addConstr(gp.quicksum(lambda_vars[j] for j, col in enumerate(all_columns) if i in col.path) == 1, name=f"cust_{i}") for i in range(1, num_customers + 1)}
    vehicle_constraint = master_model.addConstr(gp.quicksum(lambda_vars) <= num_vehicles, name="vehicle")

    for iter_count in range(100):
        if time.time() >= cg_end_time: break
        master_model.optimize()
        if master_model.status != GRB.OPTIMAL: break
        for customer_id in customer_visit_counts: customer_visit_counts[customer_id] *= DECAY_FACTOR
        duals = {'vehicle': vehicle_constraint.Pi, **{i: constr.Pi for i, constr in cust_constraints.items()}}
        new_cols = solve_subproblem_hybrid(line_pool, back_pool, nodes, dist_matrix, capacity, duals, customer_visit_counts, PENALTY_FACTOR, list(permanent_column_pool.values()))
        if not new_cols: break
        new_paths_for_fragmentation = []
        for col in new_cols:
            path_tuple = tuple(col.path)
            if path_tuple not in permanent_column_pool: permanent_column_pool[path_tuple] = col
            if path_tuple not in {tuple(c.path) for c in all_columns}:
                new_paths_for_fragmentation.append(col.path)
                for customer_id in col.path[1:-1]: customer_visit_counts[customer_id] += 1
                coeffs = gp.Column([1.0] * (len(col.path) - 2), [cust_constraints[i] for i in col.path[1:-1]]); coeffs.addTerms(1.0, vehicle_constraint)
                all_columns.append(col)
                lambda_vars[len(all_columns)-1] = master_model.addVar(obj=col.cost, vtype=GRB.CONTINUOUS, name=f"lambda_{len(all_columns)-1}", column=coeffs)
        new_line_fragments, new_back_fragments = split_to_line_back_pools(new_paths_for_fragmentation, nodes, 0)
        line_pool = list(set(line_pool) | set(new_line_fragments)); back_pool = list(set(back_pool) | set(new_back_fragments))
        master_model.update()

    if a_valid_initial_solution:
        path_to_index_map = {tuple(col.path): j for j, col in enumerate(all_columns)}
        for var in lambda_vars.values(): var.Start = 0.0
        for route_path in a_valid_initial_solution:
            route_tuple = tuple(route_path)
            if route_tuple in path_to_index_map:
                lambda_vars[path_to_index_map[route_tuple]].Start = 1.0
    
    for v in lambda_vars.values(): v.VType = GRB.BINARY
    master_model.setParam(GRB.Param.RINS, 10); master_model.setParam(GRB.Param.Heuristics, 1)
    master_model.setParam(GRB.Param.TimeLimit, final_mip_time_limit); master_model.setParam('OutputFlag', 0)
    master_model.optimize()
    
    if master_model.status in [GRB.OPTIMAL, GRB.SUBOPTIMAL, GRB.TIME_LIMIT] and master_model.SolCount > 0:
        raw_solution_routes = [all_columns[j].path for j, var in lambda_vars.items() if var.X > 0.5]
        return raw_solution_routes
    else:
        return []
