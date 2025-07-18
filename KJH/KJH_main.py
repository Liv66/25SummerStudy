from KJH.KJH_vrpb import *

def KJH_main(problem_info):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    c = Construction(K, node_type, node_demand, capa, dist_mat)
    c.construct()
    sol = [route.hist for route in c.routes]
    return sol


if __name__ == "__main__":
    KJH_main()

