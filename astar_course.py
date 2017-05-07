# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------

#grid = [[0, 1, 0, 0, 0, 0],
#        [0, 1, 0, 0, 0, 0],
#        [0, 1, 0, 0, 0, 0],
#        [0, 1, 0, 0, 0, 0],
#        [0, 0, 0, 0, 1, 0]]


grid = [[0,0,0,1], [0,0,0,0], [0,0,0,0]]

goal = [len(grid)-1, len(grid[0])-1]

'''        
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]
'''

cost_dp = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0]==x and goal[1]==y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        change = True
                elif grid[x][y]==0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if x2>=0 and x2<len(grid) and y2>=0 and y2<len(grid[0]) and grid[x2][y2]==0:
                            v2 = value[x2][y2] + cost_dp       #### ignore the non-holonomic nature of car
                            if v2<value[x][y]:
                                change=True
                                value[x][y] = v2

    #for i in range(len(value)):
    #    print value[i]

    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    return value 

heuristic = compute_value(grid, goal, cost_dp)

print heuristic

init = [0, 0]

cost = 1


def search(grid,init,goal,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h

    open = [[f, g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[2]
            y = next[3]
            g = next[1]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = heuristic[x][y]
                            f2 = g2 + h2
                            open.append([f2, g2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

    a = goal[0]
    b = goal[1] 
    path_estimate = []
    #path[a][b] = 'goal'
    path_estimate.append([a,b])
    while a!=0 or b!=0: 
        a1 = a - delta[action[a][b]][0]
        b1 = b - delta[action[a][b]][1]  
        #path[a1][b1] = delta_name[action[a][b]] 
        path_estimate.append([a1,b1])
        a = a1
        b = b1 
    
      
    
    path_estimate.reverse() 

    return path_estimate

print search(grid, init, goal, cost, heuristic)