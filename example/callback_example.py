import random
from math import sqrt
from example.callback_util import *
from example.callback_heuristic import *
from util import plot_vrpb


def run_example():
    N = 100
    nodes_coord = [(random.uniform(0, 2000), random.uniform(0, 2500)) for _ in range(N)]
    dist_mat = [
        [int(sqrt((nodes_coord[i][0] - nodes_coord[j][0]) ** 2 + (nodes_coord[i][1] - nodes_coord[j][1]) ** 2)) for i in
         range(N)] for j in
        range(N)]
    # sol, obj, elapsed = solve_subtour_heur(dist_mat, heur=patchcb)
    # plot_tsp(dist_mat, nodes_coord, sol, obj, elapsed, 'Patch')
    # sol, obj, elapsed = solve_subtour_heur(dist_mat, heur=greedycb)
    # plot_tsp(dist_mat, nodes_coord, sol, obj, elapsed, 'Greedy')
    # sol, obj, elapsed = solve_subtour_heur(dist_mat, heur=swapcb)
    # plot_tsp(dist_mat, nodes_coord, sol, obj, elapsed, 'Swap')
    sol, obj, elapsed = solve_MTZ(dist_mat)
    plot_tsp(dist_mat, nodes_coord, sol, obj, elapsed, 'MTZ')
    sol, obj, elapsed = solve_subtour(dist_mat)
    plot_tsp(dist_mat, nodes_coord, sol, obj, elapsed, 'Subtour')
    sol, obj, elapsed = two_opt(dist_mat)
    plot_tsp(dist_mat, nodes_coord, sol, obj, elapsed, '2Opt')


if __name__ == '__main__':
    run_example()
