import numpy
import dubins
from math import *
from func import *


grid = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]

init = [0, 0]

CellSize = 3
goal = [len(grid)-1, len(grid[0])-1]
cost = CellSize

### Need for dynamic programming heuristic ###
delta = [[-1, 0],   ## up
		[0, -1],    ## left
		[1, 0],     ## down
		[0, 1]]     ## right

### discretised theta ###
num_of_orientations = 36  

dp_h = compute_value(grid, goal, cost, delta)
h1 = numpy.zeros((len(grid), len(grid[0])))
for i in range(len(dp_h)):
    for j in range(len(dp_h[0])):
            h1[i][j] = dp_h[i][j]

### Coordinates of center (x,y) of each grid cell ### 
nx = numpy.zeros((len(grid), len(grid[0])))
ny = numpy.zeros((len(grid), len(grid[0])))

for i in range(len(grid)):
	for j in range(len(grid[0])):
		nx[i][j] = j*CellSize
		ny[i][j] = -i*CellSize

#print nx
#print ny

ngoal =  (nx[len(grid)-1][len(grid[0])-1], ny[len(grid)-1][len(grid[0])-1], -pi/4.0)   ## -pi/4  is orientation
initial_pose = (nx[0][0], ny[0][0], 0.0)

length = 2.5
turning_radius = 1.76  ## Wheelbase, L=2.5, maximum steering angle = pi/4, R = L/(2*sin(pi/4)) = 1.76 meters 
step_size = 0.4999     ## step_size for dubins_model

h2 = numpy.zeros((len(grid), len(grid[0]), num_of_orientations))
heuristic = numpy.zeros((len(grid), len(grid[0]), num_of_orientations))

orientations = numpy.linspace(0, 2*pi, num_of_orientations)

for i in range(len(grid)):
    for j in range(len(grid[0])):

        for k in range(len(orientations)):
            nstart = (nx[i][j], ny[i][j], orientations[k])  
            h2[i][j][k] = dubins.path_length(nstart, ngoal, turning_radius)

            heuristic[i][j][k] = max(h1[i][j], h2[i][j][k])

#print 'h2'

#print 'heuristic'

## Given x and y, find out which node does it lie in ##

find_node = numpy.zeros((len(grid), len(grid[0]), 4))   ## 2 4 one for start_x, end_x, start_y, end_y

offset = CellSize/2.0

for i in range(len(grid)):
    for j in range(len(grid[0])):
        find_node[i][j][0] = j*CellSize - offset    ## start_x
        find_node[i][j][1] = j*CellSize + offset    ## end_x
        find_node[i][j][2] = -i*CellSize + offset
        find_node[i][j][3] = -i*CellSize - offset

#print find_node

def find_node_from_coordinates((x, y, theta)):

    pose_in_world = False
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if (x >= find_node[i][j][0]) and (x <=find_node[i][j][1]) and (y<= find_node[i][j][2])  and (y>= find_node[i][j][3]):
                (i_ans, j_ans) = (i,j)
                pose_in_world = True

    if not pose_in_world:
        return None


    theta = theta % (2*pi)
    if (theta < 0.0):
        theta = theta + (2*pi)
    k1 = int(theta/(2*pi/36))
    if k1 is 36:
        k_ans = 35
    else:
        k_ans = int(theta/(2*pi/36))

    return (i_ans, j_ans, k_ans) 

## Given theta where does it lie in 36-discrete space, to find k in final_h[i][j][k]
#print orientations

###################################

max_steering = pi/4
min_steering = -pi/4
num_of_actions = 9

actions = numpy.linspace(max_steering, min_steering, 9)

#print actions

#initial_pose = (0.0, 0.0, 0.0)

distance = 2.5

(i,j,k) = find_node_from_coordinates(initial_pose)
g = 0
h = heuristic[i][j][k]
f = g + h

class Node(object):
    def __init__(self, pose, f, g):
        self.pose = pose
        self.f = f
        self.g = g
        self.parent = None
        self.children = []
        self.open = Heap()

    def add_child(self, obj):
        self.children.append(obj)

    def add_parent(self, obj):
        self.parent = obj

root = Node(initial_pose, f, g)

root.open.push(f,root)

## while loop from here

#my_node = root
#obj = my_node.open.pop()
#node = obj[1]
#print node.g
#print node.pose

resign = False
found = False
stop = False

expanded_node = root

while not resign and not found and not stop:

    buffer = []

    if len(expanded_node.open) == 0:
        resign = True

  
    else:
        prev_pose = expanded_node.open.pop()
        expanded_node = prev_pose[1]
        #print expanded_node.pose
        for action in actions:
            #print prev_pose
            a = find_next_pose(expanded_node.pose, action, distance, length)
            if a is not None:
                buffer.append(a)

        #print len(buffer)
        #print buffer
        #if buffer is empty need to do backtrace

        for pose in buffer:
                ## Check if it is goal ##
                if (pose[0] <= ngoal[0] + 1.0) and (pose[0] >= ngoal[1]-1.0) and (pose[1] <= ngoal[1]+1.0) and (pose[1] >= ngoal[1] - 1.0) and compare_theta(pose[2], ngoal[2]): 
                    found = True
                    final_path = []
                    final_path.append(pose)
                    traverse_obj = expanded_node
                else:
                    g2 = expanded_node.g
                    a = find_node_from_coordinates(pose)
                    #print a
                    if a is not None:

                        (i,j,k) = (a[0], a[1], a[2])
                        
                        g2 += dubins.path_length(expanded_node.pose, pose, turning_radius)

                        f2 = heuristic[i][j][k] + g2
                        child = Node(pose, f2, g2)
                        expanded_node.open.push(f2, child)
                    ## also add node in tree
                        expanded_node.add_child(child)
                        child.add_parent(expanded_node)

                    #expanded_node = child.parent
        if (len(expanded_node.children)==0):
            expanded_node = expanded_node.parent

        if (len(expanded_node.open)==0):
            expanded_node = expanded_node.parent

### add parent too

#print root

#for item in expanded_node.children:
#    print item.pose

while traverse_obj is not root:
    final_path.append(traverse_obj.pose)
    traverse_obj = traverse_obj.parent

final_path.append(root.pose)

final_path.reverse()

print resign
print found

print final_path
## Found is true. I am reaching goal from initial pose but I need to store coordinates of path taken ##