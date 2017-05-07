from math import *


import heapq
class Heap(object):

    def __init__(self):
        self._heap = []

    def push(self, priority, item):
        assert priority >= 0
        heapq.heappush(self._heap, (priority, item))

    def pop(self):
        """ Returns the item with lowest priority. """
    #    item = heapq.heappop(self._heap)[1] # (prio, item)[1] == item
        item = heapq.heappop(self._heap)
        return item

    def __len__(self):
        return len(self._heap)

    def __iter__(self):
        """ Get all elements ordered by asc. priority. """
        return self

    def next(self):
        """ Get all elements ordered by their priority (lowest first). """
        try:
            return self.pop()
        except IndexError:
            raise StopIteration

def compute_value(grid, goal, cost, delta):

	value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
	change = True

	while change:
		change = False
		for x in range(len(grid)):
			for y in range(len(grid[0])):
				if goal[0]==x and goal[1]==y:
					if value[x][y]>0:
						value[x][y] = 0
						change = True
				elif grid[x][y]==0:
					for a in range(len(delta)):
						x2 = x + delta[a][0]
						y2 = y + delta[a][1]

						if x2>=0 and x2<len(grid) and y2>=0 and y2<len(grid[0]) and grid[x2][y2]==0:
							v2 = value[x2][y2] + cost
							if v2 < value[x][y]:
								change = True
								value[x][y] = v2

	#for i in range(len(value)):
	#	print value[i]

	return value


def find_next_pose(prev_pose, action, distance, length):

	prev_x = prev_pose[0]
	prev_y = prev_pose[1]
	prev_theta = prev_pose[2]

	beta = float(distance/length)*tan(action)

	if beta > 0.001:   ### Not a straight line
		R = float(distance/beta)   ## Turning radius
		CX = prev_x - (sin(prev_theta)*R)
		CY = prev_y + (cos(prev_theta)*R)
		next_theta = (prev_theta + beta) % (2*pi)
		next_x = CX + sin(next_theta)*R
		next_y = CY - cos(next_theta)*R

	else:
		next_x = prev_x + distance*cos(prev_theta)
		next_y = prev_y + distance*sin(prev_theta)
		next_theta = prev_theta + beta

	return (next_x, next_y, next_theta)

def compare_theta(theta, goal):
	if (theta < 0.0):
		theta = theta + (2*pi)
	if (goal < 0.0):
		goal = goal + (2*pi)
	if (theta <= goal + pi/10.0) and (theta >= goal - pi/10.0):
		return True
	else:
		return False

'''
prev_cost = 0.0
iter = 0
while flag:
    buffer = []
    f = numpy.zeros(9)   ## Each time exactly 9 nodes expand
    for action in actions:
        buffer.append(find_next_pose(prev_pose, action, distance, length))
    index = 0
    min_f = 1000
    #print buffer
    for pose in buffer:
        cost = 0.0
        if find_node_from_coordinates(pose[0], pose[1]) is not None:

            (i,j) = find_node_from_coordinates(pose[0], pose[1])
            k = find_orientation_from_theta(pose[2])
            #print (i,j,k)
            qs, _ = dubins.path_sample(prev_pose, pose, turning_radius, step_size)

            for p in range(len(qs)-1):
                sampled_pose0 = qs[p]
                sampled_pose1 = qs[p+1]
                l = dubins.path_length( sampled_pose0, sampled_pose1, turning_radius)
                cost += l

            f[index] = final_h[i][j][k] + cost

            if f[index] < min_f:
                min_f = f[index]
                indices = (i,j,k)
                next_pose = pose

            index = index + 1

    prev_pose = next_pose
    print prev_pose
    iter = iter + 1
    if (iter > 7):
        flag = False

print min_f, indices, next_pose
'''