### s = ( x1, y1, theta1) (Start Position) ###
### g = ( x2, y2, theta2) (Goal Position)  ###

from math import *
import random
import numpy as np

x1=0.0
y1=0.0 
theta1=0.0
start = (x1, y1, theta1)

x2 = 30.0
y2 = -30.0
theta2 = 0.0  ### Not feasible at (0,0,0) to (6,0,pi/3)
final_goal = (x2, y2, theta2)


### atan2(y,x) ###
### length: Wheelbase of car ###
world_size = 100.0
length_of_car = 3.0 
steeringMax = pi/4
rmin = length_of_car/(2*sin(steeringMax))   ## 2.12 metres
#print rmin

class robot:

	# --------

	# init: 
	#	creates robot and initializes location/orientation 
	#

	def __init__(self, length = 3.0):
		self.x = random.random() * world_size # initial x position
		self.y = random.random() * world_size # initial y position
		self.orientation = random.random() * 2.0 * pi # initial orientation
		self.length = length # length of robot
	

	def __repr__(self):
		return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
		# --------
		# set: 
		#	sets a robot coordinate
		#

	def set(self, new_x, new_y, new_orientation):

		if new_orientation < 0 or new_orientation >= 2 * pi:
			raise ValueError, 'Orientation must be in [0..2pi]'
		self.x = float(new_x)
		self.y = float(new_y)
		self.orientation = float(new_orientation)

	############# ONLY ADD/MODIFY CODE BELOW HERE ###################

	# --------
	# move:
	#   move along a section of a circular path according to motion
	#

	def move(self, motion): # Do not change the name of this function


	# motion[0] is the steering angle, motion[1] is the distance
		beta = float(motion[1]/self.length)*tan(motion[0])

		if beta>0.001:
			R = float(motion[1]/beta)
			CX = self.x - (sin(self.orientation)*R)
			CY = self.y + (cos(self.orientation)*R)
			self.orientation = (self.orientation+beta)%(2*pi)
			self.x = CX + sin(self.orientation)*R
			self.y = CY - cos(self.orientation)*R


		else:
			self.x = self.x + motion[1]*cos(self.orientation)
			self.y = self.y + motion[1]*sin(self.orientation)
			self.orientation = self.orientation + beta

	# ADD CODE HERE
		result = self

		return result # make sure your move function returns an instance
					  # of the robot class with the correct coordinates.

### turning right means negative steering angle - verified ###

### Vec1 = (x,y)   Starting point ###
### Vec2           Destination point ###
def ArcLength(Vec1, Vec2, left):

	theta = atan2(Vec2[1],Vec2[0]) - atan2(Vec1[1],Vec1[0])
	#print Vec1, Vec2, atan2(Vec2[1],Vec2[0]), atan2(Vec1[1],Vec1[0]), theta
	if theta < 0.0 and left:                 ### 1 indicates left
		theta = theta + (2*pi)
	else: 
		if theta > 0.0 and not left:
			theta = theta - (2*pi)

	return abs(theta*rmin)



def turn_right_min_radius(pose):
	x1 = pose[0]
	y1 = pose[1]
	theta1 = pose[2]
	theta1 -= pi/2
	if (theta1 < -pi):
		theta1 += 2*pi 
	center = (x1 + rmin*cos(theta1),  y1 + rmin*sin(theta1))
	return center

def turn_left_min_radius(pose):
	x1 = pose[0]
	y1 = pose[1]
	theta1 = pose[2]
	theta1 += pi/2
	if theta1 > pi:
		theta1 -= 2*pi
	center = (x1 + rmin*cos(theta1),  y1 + rmin*sin(theta1))
	return center

def euclidean_distance(p1, p2):
	dist = sqrt( pow(p2[0]-p1[0],2) + pow(p2[1]-p1[1],2) )
	return dist

def tangent_points_on_circles(pc1, pc2):
	Points = []
	D = euclidean_distance(pc1, pc2)
	#print 'D ' + str(D)
	vx = (pc2[0]-pc1[0])/D
	#print 'vx ' + str(vx)
	vy = (pc2[1]-pc1[1])/D
	#print 'vy ' + str(vy) 
	r1 = r2 = rmin

	x1 = pc1[0]
	y1 = pc1[1]
	x2 = pc2[0]
	y2 = pc2[1]

	for sign1 in (1, -1):
		c = (r1 - sign1*r2)/D 
		if c*c > 1.0:
			continue
		h = sqrt(max(0, 1 - c*c))

		for sign2 in (1, -1):
			nx = vx * c - sign2 * h * vy
			ny = vy * c + sign2 * h * vx
			Points.append( (x1 + r1*nx, y1 + r1*ny, x2 + sign1*r2*nx, y2 + sign1*r2*ny) )

	return Points

DELTA = 1.0

controls = {'RSR':[], 'LSL':[], 'RSL':[], 'LSR':[], 'RLR':[], 'LRL':[]}

def RSRTrajectory(RRTangents, agentRight, queryRight):

	controls['RSR']=[]
	if len(RRTangents) > 0:

		steeringAngle = -1*steeringMax
		left = False
		Vec1 =  (start[0]-agentRight[0], start[1]-agentRight[1])
		Vec2 =  (RRTangents[0][0] - agentRight[0], RRTangents[0][1] - agentRight[1])
		arcL1 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL1/DELTA 
		controls['RSR'].append([steeringAngle, timesteps])

		steeringAngle = 0.0
		## tangent pts function returns outer tangents for RR connection first
		p1 = (RRTangents[0][0], RRTangents[0][1])
		p2 = (RRTangents[0][2], RRTangents[0][3])
		arcL2 = euclidean_distance(p1, p2)
		timesteps = arcL2/DELTA
		controls['RSR'].append([steeringAngle, timesteps])

		steeringAngle = -1*steeringMax
		left = False
		Vec1 =  (RRTangents[0][2] - queryRight[0], RRTangents[0][3] - queryRight[1])
		Vec2 =  (goal[0]-queryRight[0], goal[1]-queryRight[1])
		arcL3 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL3/DELTA
		controls['RSR'].append([steeringAngle, timesteps])

		length = arcL1 + arcL2 + arcL3

		#print arcL1, arcL2, arcL3, length

		return length


def LSLTrajectory(LLTangents, agentLeft, queryLeft):
	controls['LSL'] = []
	if len(LLTangents) > 1:
		## tangent pts function returns outer tangents for LL connection second
		#print LLTangents[1]

		steeringAngle = steeringMax
		left = True
		Vec1 =  (start[0]-agentLeft[0], start[1]-agentLeft[1])
		Vec2 =  (LLTangents[1][0] - agentLeft[0], LLTangents[1][1] - agentLeft[1])
		#print Vec1
		#print Vec2
		arcL1 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL1/DELTA
		controls['LSL'].append([steeringAngle, timesteps])

		steeringAngle = 0.0
		p1 = (LLTangents[1][0], LLTangents[1][1])
		p2 = (LLTangents[1][2], LLTangents[1][3])
		arcL2 = euclidean_distance(p1, p2)
		timesteps = arcL2/DELTA
		controls['LSL'].append([steeringAngle, timesteps])


		steeringAngle = steeringMax
		left = True
		Vec1 =  (LLTangents[1][2] - queryLeft[0], LLTangents[1][3] - queryLeft[1])
		Vec2 =  (goal[0]-queryLeft[0], goal[1]-queryLeft[1])
		#print 'Vec1 ' + str(Vec1)
		#print 'Vec2 ' + str(Vec2)
		arcL3 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL3/DELTA
		controls['LSL'].append([steeringAngle, timesteps])

		length = arcL1 + arcL2 + arcL3

		#print arcL1, arcL2, arcL3, length

		return length

## tangent pts function returns inner tangents for RL connection third

def RSLTrajectory(RRTangents, agentRight, queryLeft):
	controls['RSL'] = []
	if len(RLTangents) > 2:

		steeringAngle = -1*steeringMax
		left = False
		Vec1 =  (start[0]-agentRight[0], start[1]-agentRight[1])
		Vec2 =  (RLTangents[2][0] - agentRight[0], RLTangents[2][1] - agentRight[1])
		arcL1 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL1/DELTA
		controls['RSL'].append([steeringAngle, timesteps])

		steeringAngle = 0.0
		## tangent pts function returns outer tangents for RR connection first
		p1 = (RLTangents[2][0], RLTangents[2][1])
		p2 = (RLTangents[2][2], RLTangents[2][3])
		arcL2 = euclidean_distance(p1, p2)
		timesteps = arcL2/DELTA
		controls['RSL'].append([steeringAngle, timesteps])

		steeringAngle = steeringMax
		left = True
		Vec1 =  (RLTangents[2][2] - queryLeft[0], RLTangents[2][3] - queryLeft[1])
		Vec2 =  (goal[0]-queryLeft[0], goal[1]-queryLeft[1])
		arcL3 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL3/DELTA
		controls['RSL'].append([steeringAngle, timesteps])

		length = arcL1 + arcL2 + arcL3

		#print arcL1, arcL2, arcL3, length

		return length

def LSRTrajectory(LRTangents, agentLeft, queryRight):
	## tangent pts function returns inner tangents for LR connection fourth 
	controls['LSR'] = []
	if len(LRTangents) > 3:

		steeringAngle = steeringMax
		left = True
		Vec1 =  (start[0]-agentLeft[0], start[1]-agentLeft[1])
		Vec2 =  (LRTangents[3][0] - agentLeft[0], LRTangents[3][1] - agentLeft[1])
		arcL1 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL1/DELTA
		controls['LSR'].append([steeringAngle, timesteps])

		steeringAngle = 0.0
		p1 = (LRTangents[3][0], LRTangents[3][1])
		p2 = (LRTangents[3][2], LRTangents[3][3])
		arcL2 = euclidean_distance(p1, p2)
		timesteps = arcL2/DELTA
		controls['LSR'].append([steeringAngle, timesteps])

		steeringAngle = -1*steeringMax
		left = False
		Vec1 =  (LRTangents[3][2] - queryRight[0], LRTangents[3][3] - queryRight[1])
		Vec2 =  (goal[0]-queryRight[0], goal[1]-queryRight[1])
		arcL3 = ArcLength(Vec1, Vec2, left)
		timesteps = arcL3/DELTA
		controls['LSR'].append([steeringAngle, timesteps])
		length = arcL1 + arcL2 + arcL3

		#print arcL1, arcL2, arcL3, length

		return length



def RLRTrajectory(agentRight, queryRight):
	controls['RLR'] = []
	### RLR ###
	D = euclidean_distance(agentRight, queryRight)
	if (D < 4*rmin):
		theta = acos(D/(4*rmin))
		theta += atan2(queryRight[1] - agentRight[1],  queryRight[0] - agentRight[0])
		pc3 = (agentRight[0] + 2*rmin*cos(theta), agentRight[1] + 2*rmin*sin(theta))
		agentTan1 = (pc3[0] + agentRight[0])/2.0
		agentTan2 = (pc3[1] + agentRight[1])/2.0

		queryTan1 = (pc3[0] + queryRight[0])/2.0
		queryTan2 = (pc3[1] + queryRight[1])/2.0

		steeringAngle = -1.0 * steeringMax
		Vec1 = (start[0] - agentRight[0] , start[1] - agentRight[1])
		Vec2 = (agentTan1 - agentRight[0] , agentTan2 - agentRight[1])
		arcL1 = ArcLength(Vec1, Vec2, False)
		timesteps = arcL1/DELTA
		controls['RLR'].append([steeringAngle, timesteps])

		steeringAngle = steeringMax 
		Vec1 = (agentTan1 - pc3[0], agentTan2 - pc3[1])
		Vec2 = (queryTan1 - pc3[0], queryTan2 - pc3[1])
		arcL2 = ArcLength(Vec1, Vec2, True)
		timesteps = arcL2/DELTA
		controls['RLR'].append([steeringAngle, timesteps])

		steeringAngle = -1.0*steeringMax
		Vec1 =  (queryTan1 - queryRight[0], queryTan2 - queryRight[1])
		Vec2 =  (goal[0] - queryRight[0], goal[1] - queryRight[1])
		arcL3 = ArcLength(Vec1, Vec2, False)
		timesteps = arcL3/DELTA
		controls['RLR'].append([steeringAngle, timesteps])
		length = arcL1 + arcL2 + arcL3

		
		return length


def LRLTrajectory(agentLeft, queryLeft):
	controls['LRL'] = []
	D = euclidean_distance(agentLeft, queryLeft)
	if (D < 4*rmin):
		theta = acos(D/(4*rmin))
		theta = atan2(queryLeft[1] - agentLeft[1], queryLeft[0] - agentLeft[0]) - theta
		pc3 = (agentLeft[0] + 2*rmin*cos(theta), agentLeft[1] + 2*rmin*sin(theta))

		agentTan1 = (pc3[0] + agentLeft[0])/2.0
		agentTan2 = (pc3[1] + agentLeft[1])/2.0

		queryTan1 = (pc3[0] + queryLeft[0])/2.0
		queryTan2 = (pc3[1] + queryLeft[1])/2.0

		steeringAngle = -1.0 * steeringMax
		Vec1 = (start[0] - agentLeft[0] , start[1] - agentLeft[1])
		Vec2 = (agentTan1 - agentLeft[0] , agentTan2 - agentLeft[1])
		arcL1 = ArcLength(Vec1, Vec2, True)
		timesteps = arcL1/DELTA
		controls['LRL'].append([steeringAngle, timesteps])

		steeringAngle = steeringMax 
		Vec1 = (agentTan1 - pc3[0], agentTan2 - pc3[1])
		Vec2 = (queryTan1 - pc3[0], queryTan2 - pc3[1])
		arcL2 = ArcLength(Vec1, Vec2, False)
		timesteps = arcL2/DELTA
		controls['LRL'].append([steeringAngle, timesteps])

		steeringAngle = -1.0*steeringMax
		Vec1 =  (queryTan1 - queryLeft[0], queryTan2 - queryLeft[1])
		Vec2 =  (goal[0] - queryLeft[0], goal[1] - queryLeft[1])
		arcL3 = ArcLength(Vec1, Vec2, True)
		timesteps = arcL3/DELTA
		controls['LRL'].append([steeringAngle, timesteps])

		length = arcL1 + arcL2 + arcL3

		return length
	



myrobot = robot(length_of_car)
myrobot.set(0.0, 0.0, 0.0)


## Update Equations ##
myrobot.x = myrobot.x + (DELTA*cos(myrobot.orientation))
myrobot.y = myrobot.y + (DELTA*sin(myrobot.orientation))


turningRadius = 0.0
straightLine = True

if (abs(myrobot.steering_angle)>1e-5):
	turningRadius = myrobot.length/(2*sin(myrobot.steering_angle))
	straightLine = False

if not straightLine:
	myrobot.orientation = myrobot.orientation + (DELTA/turningRadius)
	if (myrobot.orientation > pi):
		myrobot.orientation = myrobot.orientation - (2*pi)
	else:
		if (myrobot.orientation < -pi):
			myrobot.orientation = myrobot.orientation + (2*pi) 

goal = (myrobot.x, myrobot.y, myrobot.orientation)
print goal

num_orientations = 36
possible_orientations = np.linspace(-pi,pi,36)

agentRight = turn_right_min_radius(start)
#print 'agentRight ' + str(agentRight)
queryRight = turn_right_min_radius(goal)
#print 'queryRight ' + str(queryRight)
agentLeft  = turn_left_min_radius(start)
#print 'agentLeft ' + str(agentLeft)
queryLeft  = turn_left_min_radius(goal)
#print 'queryLeft ' + str(queryLeft)

RRTangents = tangent_points_on_circles(agentRight, queryRight)
#for tangent in RRTangents:
#	print tangent
#	print 
LLTangents = tangent_points_on_circles(agentLeft, queryLeft)
#for tangent in LLTangents:
#	print tangent
#	print 
RLTangents = tangent_points_on_circles(agentRight, queryLeft)
LRTangents = tangent_points_on_circles(agentLeft, queryRight)
#print LRTangents		

length = RSRTrajectory(RRTangents, agentRight, queryRight)

shortestlength = length
label = 'RSR'

length = LSLTrajectory(LLTangents, agentLeft, queryLeft)
if (shortestlength > length):
	shortestlength = length
	label = 'LSL'

length = RSLTrajectory(RLTangents, agentRight, queryLeft)
if (shortestlength > length):
	shortestlength = length
	label = 'RSL'


length = LSRTrajectory(LRTangents, agentLeft, queryRight)
if (shortestlength > length):
	shortestlength = length
	label = 'LSR'

length = RLRTrajectory(agentRight, queryRight)
if (shortestlength > length):
	shortestlength = length
	label = 'RLR'

length = LRLTrajectory(agentLeft, queryLeft)
if (shortestlength > length):
	shortestlength = length
	label = 'LRL'


if shortestlength is not None:
	print 'Answer:  ' + str(shortestlength) + '  ' + str(label) + ' ' +str(theta)
	print controls[label]
else:
	print "Not feasible"


'''
motions = controls[label]
for motion in motions:
	goal = myrobot.move(motion)
	print goal.x, goal.y, goal.orientation
'''


