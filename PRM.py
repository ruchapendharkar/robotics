# Load the PIL and numpy libraries
from PIL import Image
import numpy as np
import networkx as nx
import math
from matplotlib import pyplot as plt

# Read image from disk using PIL; please enter path to the occupancy map
occupancy_map_img = Image.open('/home/rucha/Mobile Robotics/HW2/occupancy_map.png')

# Interpret this image as a numpy array, and threshold its values to→ {0,1}
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

G = nx.Graph()

start = (635, 140)
goal = (350,400)

def d_function(v1,v2):
    r1, c1 = v1
    r2, c2 = v2
    distance = math.sqrt((r1-r2)**2 + (c1-c2)**2)
    return distance

def sample_vertex(M):
    rows, cols = len(M), len(M[0])
    while True:
        r = int(np.random.uniform(0, rows - 1))
        c = int(np.random.uniform(0, cols - 1))
        #If unoccupied 
        if M[r][c] == 1:   
            return (r, c)


def reachability_check(M, v1, v2, d_max):
    if v1 == v2:
        return False
    #Unpacking vertices into co-ordinates
    x1, y1 = v1
    x2, y2 = v2
    dx = x2 - x1
    dy = y2 - y1
    steps = max(abs(dx), abs(dy))
    if steps > d_max:
        return False
    for i in range(steps + 1):
        y = y1 + i * dy // steps
        x = x1 + i * dx // steps
        #If occupied
        if M[x][y] == 0:   
            return False
    return True


def add_vertex(G, new_vertex, dmax):
    for v in G.nodes:
        if reachability_check(occupancy_grid, new_vertex, v, dmax) and d_function(v, new_vertex) <= dmax:
            distance = d_function(v, new_vertex)
            G.add_edge(new_vertex, v, weight = distance)

def constructPRM(N,dmax):

    #Adding the start and end points to the graph
    G.add_node(start)
    G.add_node(goal)
    for i in range(N):
        new_vertex = sample_vertex(occupancy_grid)
        #print(new_vertex)
        G.add_node(new_vertex)
        add_vertex(G, new_vertex, dmax)
    return G

prm_graph = constructPRM(N=2500, dmax=75)
print(prm_graph)

#Calculating and plotting Astar

path = nx.astar_path(prm_graph, start , goal)
#print(path)

def total_distance(points):
    distance = 0
    for i in range(len(points) - 1):
        v1 = points[i]
        v2 = points[i + 1]
        r1, c1 = v1
        r2, c2 = v2
        distance += math.sqrt((r1-r2)**2 + (c1-c2)**2)
    return distance

path_length = total_distance(path) 
print("The length of the path is", path_length)


#Plotting the network on the graph
pos = {}
for i in prm_graph.nodes():
    pos[i] = (i[1], i[0])
nx.draw_networkx(prm_graph, pos = pos , node_size = 0.85, with_labels=0, width = 0.5, edge_color = 'orange')

path_x = [x[1] for x in path]
path_y = [x[0] for x in path]
plt.plot(path_x, path_y)


plt.imshow(occupancy_map_img)
plt.show()
