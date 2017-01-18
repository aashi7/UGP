# -*- coding: utf-8 -*-

import Image
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sys

mpl.rcParams['toolbar'] = 'None'


if(len(sys.argv) != 3):
    print("Usage : {} <failure_probability(0.0,1.0)> <maze_img>".format(sys.argv[0]))
    print("e.g. {} 0.5 maze1.pnm".format(sys.argv[0]))
    print("e.g. {} 0.5 maze4.ppm".format(sys.argv[0]))
    sys.exit(-1)

failure_probability = float(sys.argv[1])
if(not (0 <= failure_probability <= 1)):
    print("Oups, failure probability must lie in [0.0, 1.0]")
    sys.exit(-1)

maze_img = sys.argv[2]


################## Tools

def load_maze_from_image(img_filename):
    ''' Loads an image and convert it into a numpy array
    with False for walls and True for possible positions'''
    im = Image.open(img_filename).convert("L")
    data = np.asarray( im, dtype="bool" )
    return data

### GUI

myPlots = []
myOptions = {'show_true_state' : False}
myData = {'current_belief': 0 , 
          'current_state': 0 , 
          'boolean_maze' : 0 ,
          'probObs' : 0,
          'probTrans' : 0,
          'failure_probability' : failure_probability
          }

def show_maze():
    global myPlots, myOptions, myData

    belief = myData['current_belief']
    boolean_maze = myData['boolean_maze']
    current_state = myData['current_state']

    img_maze = np.zeros(boolean_maze.shape)
    img_maze[boolean_maze] = 0
    img_maze[np.logical_not(boolean_maze)] = 1

    img_belief = np.zeros(boolean_maze.shape)
    img_belief[np.logical_not(boolean_maze)] = np.nan
    img_belief[boolean_maze] = belief

    # make the colormaps
    cmap = mpl.colors.LinearSegmentedColormap.from_list('my_cmap',['white','yellow','orange','red'],1024)
    cmap_norm = mpl.colors.LogNorm(0.01, 1.0)

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    if(myOptions['show_true_state']):
        plt_current_state, = ax1.plot(current_state[1], current_state[0], 'o')
    else:
        plt_current_state, = ax1.plot(-1, -1, 'o')

    plt_img_maze = plt.imshow(img_maze, cmap='binary', interpolation='None')
    plt_img_belief = plt.imshow(img_belief, cmap=cmap, clim=[0.01,1], interpolation='None', norm=cmap_norm)
    plt.gca().invert_yaxis()
    plt.colorbar()

    myPlots = [plt_current_state, plt_img_maze, plt_img_belief, fig]
    cid = fig.canvas.mpl_connect('key_press_event', onkey)

    plt.show()


def reset():
    global myData
    # Our very initial belief
    belief = np.ones((N_possible_states, )) / N_possible_states

    # For testing the drawing of the maze (especially the colormaps)
    #belief = np.random.random((N_possible_states, )) 
    
    # Our initial position 
    s = possible_states[np.random.randint(0, N_possible_states)]
    
    # From which we can compute the initial belief
    obs = observe(s, boolean_maze)
    belief = prob_obs[:,observation_to_int(obs)] * belief # term by term multiplication
    belief /= belief.sum()

    myData['current_belief'] = belief
    myData['current_state'] = s  
    myData['img_index'] = 0  

def savefig():
    global myData
    print('Saving img to %s' % ('maze-gui-%i.png' % myData['img_index']))
    plt.savefig('maze-gui-%i.png' % myData['img_index'], bbox_inches='tight')
    myData['img_index'] += 1

def onkey(event):
    global myPlots, myOptions, myData
    if(event.key == 'h'):
        myOptions['show_true_state'] = not(myOptions['show_true_state'])
    elif(event.key == 'r'):
        print("Resetting the position and belief of the agent")
        reset()
    elif(event.key == 's'):
        savefig()
    elif(event.key == 'n'):
        s = myData['current_state']
        belief = myData['current_belief']
        boolean_maze = myData['boolean_maze']
        prob_obs = myData['probObs']
        prob_trans = myData['probTrans']
        failure_prob = myData['failure_probability']

        a = policy(s, belief, boolean_maze)
        s1 = next_state(s, a, boolean_maze)
        o = observe(s1, boolean_maze, failure_prob)
        belief = update_belief(belief, a, o, prob_obs, prob_trans)

        myData['current_state'] = s1
        myData['current_belief'] = belief

    elif(event.key in ['left', 'up','right','down']):
        s = myData['current_state']
        belief = myData['current_belief']
        boolean_maze = myData['boolean_maze']
        prob_obs = myData['probObs']
        prob_trans = myData['probTrans']
        failure_prob = myData['failure_probability']

        a = ['left', 'up','right','down'].index(event.key)
        s1 = next_state(s, a, boolean_maze)
        o = observe(s1, boolean_maze, failure_prob)
        belief = update_belief(belief, a, o, prob_obs, prob_trans)

        myData['current_state'] = s1
        myData['current_belief'] = belief       

    else:
        print(event.key)

    # Update the plots
    plt_current_state, plt_img_maze, plt_img_belief, fig = myPlots

    if(myOptions['show_true_state']):
        plt_current_state.set_data([myData['current_state'][1], myData['current_state'][0]])
    else:
        plt_current_state.set_data([-1, -1])

    img_maze = np.zeros(myData['boolean_maze'].shape)
    img_maze[myData['boolean_maze'] ] = 0
    img_maze[np.logical_not(myData['boolean_maze'])] = 1

    plt_img_maze.set_data(img_maze)

    img_belief = np.zeros(myData['boolean_maze'].shape)
    img_belief[np.logical_not(myData['boolean_maze'])] = np.nan
    img_belief[myData['boolean_maze']] = myData['current_belief']

    plt_img_belief.set_data(img_belief)

    fig.canvas.draw()

    

def observation_to_int(obs):
    # Observation : (left_empty?, top_empty?, right_empty?, bottom_empty?) all as a boolean
    '''
    Returns the int value of the observation
    (0, 0, 0, 0) -> 0
    (0, 0, 0, 1) -> 1
    ....
    (1, 1, 1, 1) -> 15
    '''
    int_obs = 0
    for i in range(4):
        int_obs += obs[3 - i] * 2**i
    return int_obs

def int_to_observation(i):
    obs = tuple(map(int, list(bin(i))[2:]))
    obs = (0,) * (4 - len(obs)) + obs
    return obs

def observation_probabilities(boolean_maze, failure_probability=0):
    ''' 
    Computes the observation probabilities for the maze
    Returns a Ns x No prob_obs matrix 
    where prob_obs[i,j] is the probability of observing j in state i
    There are No = 2^4 possible observations
    And Ns = number of valid positions (True) in boolean maze
    failure_probability is the probability to observe something different 
    from the true observation
    '''
    No = 2**4
    Ns = boolean_maze.sum()
    prob_obs = np.ones((Ns, No))
    array_possible_positions = np.where(boolean_maze)
    possible_states = [(i, j) for i, j in zip(array_possible_positions[0], array_possible_positions[1])]
    for i, s in enumerate(possible_states):
        observation = true_observe(s, boolean_maze)
        prob_obs[i,:] *= failure_probability / (No - 1)
        prob_obs[i, observation_to_int(observation)] = 1 - failure_probability
    return prob_obs

def transition_probabilities(boolean_maze):
    ''' 
    Computes the transition probabilities for the maze
    Returns a Ns x Na x Ns prob_trans matrix 
    where prob_trans[i,j,k] is the probability of going from state i to state k with action j
    Na = number of actions = 4
    And Ns = number of valid positions (True) in boolean maze
    '''
    Na = 4
    Ns = boolean_maze.sum()
    prob_trans = np.zeros((Ns, Na, Ns))

    array_possible_positions = np.where(boolean_maze)
    possible_states = [(i, j) for i, j in zip(array_possible_positions[0], array_possible_positions[1])]
    for i, s in enumerate(possible_states):
        for a in range(Na):
            s1 = next_state(s, a, boolean_maze)
            prob_trans[i, a, possible_states.index(s1)] = 1
    return prob_trans

def next_state(s0, a, boolean_maze):
    ''' From state s0, performs action a
        returns the next state, a tuple (row# , column#) '''
    #   Action : {0, 1, 2, 3} for [move left, move up, move right, move down]
    # a       0   1   2   3
    # di      0   1   0   -1
    # dj     -1   0   1   0
    #a%2      0   1   0   1
    #(1-a%2)  1   0   1   0
    di = (a % 2) * (2 - a)
    dj = (1  - (a % 2)) * (a - 1)
    s1 = (s0[0] + di, s0[1] + dj)
    if(is_a_valid_position(s1, boolean_maze)):
        return s1
    else:
        return s0

def is_a_valid_position(s, boolean_maze):
    if(s[0] < 0 or 
       s[0] >= boolean_maze.shape[0] or
       s[1] < 0 or
       s[1] >= boolean_maze.shape[1]):
        return False
    else:
        return boolean_maze[s[0],s[1]]

def true_observe(s, boolean_maze):
    ''' 
    Returns the true current observation from state s 
    It is a boolean tuple with false:wall, true:empty 
    for (west, north, east, south) 
    '''
    observation = ()
    for delta in [(0, -1), (1, 0), (0, 1), (-1, 0)]:
        n_state = (s[0] + delta[0], s[1] + delta[1])
        observation += (is_a_valid_position(n_state, boolean_maze),)
    return observation

def observe(s, boolean_maze, failure_probability=0):
    '''
    Returns the true observation or something different
    depending on failure_probability
    '''
    if(np.random.random() > failure_probability):
        return true_observe(s, boolean_maze)
    else:
        to = true_observe(s, boolean_maze)
        io = observation_to_int(to)
        nio = np.random.randint(0, 16)
        # Take something different from the true observation
        while(nio == io):
            nio = np.random.randint(0, 16)
        return int_to_observation(nio)

def policy(s, belief, boolean_maze):
    return np.random.randint(0, 4)

def update_belief(current_belief, a, o, prob_obs, prob_trans):
    ''' 
    Update the belief states
    given we executed action "a" and observed observation "o"
    a : int in [0; 3]
    o : boolean tuple (west, north, east, south)
    '''
    #p_s_prime = np.dot(np.transpose(prob_trans[a][:, :]), current_belief)
    p_s_prime = np.dot(np.transpose(prob_trans[:, a, :]), current_belief)
    belief = prob_obs[:,observation_to_int(o)] * p_s_prime
    belief /= belief.sum()
    return belief

def print_help():
    print('''
          h : Show/Hide the agent's position
          n : Make one random move
          up/left/down/right : move the agent
          r : reset
          ''')

# State : (row#, column#)
# Observation : (left_empty?, top_empty?, right_empty?, bottom_empty?) all as a boolean
# Action : {0, 1, 2, 3} for [move left, move up, move right, move down]

###########################

boolean_maze = load_maze_from_image(maze_img)
array_possible_positions = np.where(boolean_maze)
print boolean_maze[0]
print boolean_maze[1]
possible_states = [(i, j) for i, j in zip(array_possible_positions[0], array_possible_positions[1])]
N_possible_states = len(possible_states)

print('{} possible states'.format(N_possible_states))

# Computation of the observation and transition probabilities
prob_obs = observation_probabilities(boolean_maze, failure_probability)
print("Observation probabilities computed")
prob_trans = transition_probabilities(boolean_maze)
print("Transition probabilities computed")

############ Initialization
myData['boolean_maze'] = boolean_maze
myData['probObs'] = prob_obs
myData['probTrans'] = prob_trans

reset()

print_help()


show_maze()

