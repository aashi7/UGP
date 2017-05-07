#!/usr/bin/python

import math

grid = [[0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 1],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0]]

goal = [len(grid)-1, len(grid[0])-1]
#print goal
heuristic = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]

def heuristic_cal():
  global heuristic    
  for j in range(0,len(grid),1):
    for i in range(0,len(grid[0]),1):
      distance_sq = (goal[0]-j)**2 + (goal[1]-i)**2   
      heuristic[j][i] = math.sqrt(distance_sq)
 # print heuristic	
  return heuristic 

init = [0, 0]

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ],
         [-1, -1],
	 [-1, 1 ],
	 [ 1, 1 ],
	 [ 1, -1]] 

delta_name = ['^', '<', 'v', '>','&','$','@','%']


cost1 = 1
cost2 = math.sqrt(2)
def search():
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    heuristic_cal()
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    path = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    x = init[0]
    y = init[1]
    g = 0
    f = heuristic[x][y]      ## should be g+heuristic[x][y]
    open = [[f,g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    path_estimate = []
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return [[init[0],init[1]],[init[0],init[1]]]
        else:
	    #print open
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[2]
            y = next[3]
            g = next[1]
            f = next[0]
            expand[x][y] = count
            count += 1
            #print x,y,goal[0],goal[1]
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            if delta[i][0] == 0 or delta[i][1] == 0:
			      s = 1
			      d = 1
			      g2 = g + cost1
			    else:
                              s = 3
			      d = 3 
			      if grid[x2][y] == 0 and grid[x][y2] == 0:
			       d = 3
			       g2 = g + cost2
			      else:
			       d = 2
			    if d==3 or s == 1:     
                             f2 = g2 + heuristic[x2][y2] 
                             open.append([f2,g2, x2, y2])
                             closed[x2][y2] = 1
                             action[x2][y2] = i
			      
 		#	    print action[x2][y2] 
    #print action
    a = goal[0]
    b = goal[1]	
    path[a][b] = 'goal'
    path_estimate.append([a,b])
    while a!=0 or b!=0: 
	a1 = a - delta[action[a][b]][0]
        b1 = b - delta[action[a][b]][1]	 
        path[a1][b1] = delta_name[action[a][b]] 
	#if action[a][b] is not action[a1][b1]:
	path_estimate.append([a1,b1])
        a = a1
        b = b1 
	
	  
    
    path_estimate.reverse() 
    return path_estimate

print search()
