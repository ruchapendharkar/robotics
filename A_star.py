# Load the PIL and numpy libraries
from PIL import Image
import numpy as np
import heapq
import math
from matplotlib import pyplot as plt

# Read image from disk using PIL; please enter path to the occupancy map
occupancy_map_img = Image.open('/home/rucha/Mobile Robotics/HW2/occupancy_map.png')

# Interpret this image as a numpy array, and threshold its values to→ {0,1}
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

#print(occupancy_grid.shape)

Vertex = {}
for row in range(occupancy_grid.shape[0]):
    for col in range(occupancy_grid.shape[1]):
       Vertex[(row,col)] = occupancy_grid[row][col]

start = (635, 140)
goal = (350,400)

#print(Vertex)

def d_function(v1,v2):
    r1, c1 = v1
    r2, c2 = v2
    distance = math.sqrt((r1-r2)**2 + (c1-c2)**2)
    return distance

def h_function(v1,v2):
    r1, c1 = v1
    r2, c2 = v2
    heuristic = math.sqrt((r1-r2)**2 + (c1-c2)**2)
    return heuristic

def edge_weight(v1,v2):
    r1, c1 = v1
    r2, c2 = v2
    edge_weight = math.sqrt((r1-r2)**2 + (c1-c2)**2)
    return edge_weight

def N_function(v,vertex_list):
    row, col = v
    neighbour_points = []
    #Each point will have 8 neighbouring points 

    list_of_neighbours = [(row -1,col -1),(row , col -1),(row+1, col-1),(row + 1, col), (row+1,col+1), (row, col+1), (row-1, col+1),(row-1, col)]

    for i in list_of_neighbours:
        if vertex_list[i] == 1:
            neighbour_points.append(i)
    return neighbour_points

def recover_path(start, goal, pred):
    path = []
    node = goal
    while node!= start:
        path.append(node)
        node = pred.get(node)
    return path


def a_star_search(V, start, goal, N, w, h):
    # Initialization
    CostTo = {v: float('inf') for v in V}
    EstTotalCost = {v: float('inf') for v in V}
    EstTotalCost[start] = h(start, goal)
    CostTo[start] = 0
    pred = {v: None for v in V}
    Q = [(EstTotalCost[start], start)]

    # Main loop
    while Q:
        _, v = heapq.heappop(Q) #discarding the first variable 
        if v == goal:
            print(recover_path(start,goal, pred))
            return recover_path(start, goal, pred)
        for i in N(v):
            pvi = CostTo[v] + w(v, i)
            if pvi < CostTo[i]:
                pred[i] = v
                CostTo[i] = pvi
                EstTotalCost[i] = pvi + h(i, goal)
                heapq.heappush(Q, (EstTotalCost[i], i)) 
    return []


path = a_star_search(Vertex, start, goal, lambda v: N_function(v, Vertex), edge_weight, h_function)

def total_distance(path):
    distance = 0
    for i in range(len(path) - 1):
        distance += d_function(path[i], path[i + 1])
    return distance

path_length = total_distance(path)
print("The total path is",path_length)


# Read the image using the PIL library
img = Image.open("occupancy_map.png")

# Plot the path using the plot function
path_x = [x[1] for x in path]
path_y = [x[0] for x in path]
plt.scatter(path_x, path_y, s=0.5)
plt.imshow(img)
plt.show()