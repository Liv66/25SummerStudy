import json
import pandas as pd
import math


def load_from_json(file_path):
    with open(file_path, "r") as file:
        data = json.load(file)
    return data

def convert_data(problem_info):
    NUM_VEHICLES = problem_info['K']  # 전체 차량 수
    CAPACITY = problem_info['capa']

    demand = problem_info['node_demands']
    node_types = problem_info['node_types']
    coords = problem_info['node_coords']

    nodes = {'types' : node_types,
            'coords' : coords,
            'demands' : demand}

    df = pd.DataFrame(nodes)
    df = df.sort_values(by='types').reset_index(drop=True)  # types에 따라 정렬
    NUM_LINEHAUL = df[df['types'] == 1].shape[0]
    NUM_BACKHAUL = df[df['types'] == 2].shape[0]
    type_map = {0: 'depot', 1: 'linehaul', 2: 'backhaul'}
    df['type'] = df['types'].map(type_map)
    df['x'] = df['coords'].str[0]
    df['y'] = df['coords'].str[1]
    df['id'] = df.index
    nodes = df[['id', 'type', 'x', 'y', 'demands']].rename(columns={'demands': 'demand'}).to_dict('records')

    return nodes, NUM_LINEHAUL, NUM_BACKHAUL, NUM_VEHICLES, CAPACITY



def calculate_cost(nodes):
    cost_matrix = [[0] * len(nodes) for _ in range(len(nodes))]
    for i in range(len(nodes)):
        for j in range(len(nodes)):
            if i != j:
                cost_matrix[i][j] = int(round(math.sqrt((nodes[i]['x'] - nodes[j]['x'])**2 + (nodes[i]['y'] - nodes[j]['y'])**2)))

    return cost_matrix