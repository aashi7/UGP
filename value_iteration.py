from operator import add


'''2D environment - None indicates obstacle, rewards specified for possible states'''
environment = [[-0.04, -0.04, -0.04, 1],[-0.04, None, -0.04, -1],[-0.04, -0.04, -0.04, -0.04]]

'''Terminal States - Goal and Pit'''
terminals = []
terminals.append((0,3))
terminals.append((1,3))


#for i in terminals:
#	print environment[i[0]][i[1]]


'''List of possible states'''
states = []

'''Reward'''


for i in range(0,len(environment)):
    for j in range(0,len(environment[i])):
        if environment[i][j] is not None:
            states.append((i,j))

#print states

R = dict([(s,0) for s in states])
for s in states:
    R[s] = environment[s[0]][s[1]]


def turn_right(action):
    if action == (-1,0):
        return (0,1)
    if action == (0,1):
        return (1,0)
    if action == (1,0):
        return (0,-1)
    if action == (0,-1):
        return (-1,0)
    if action == None:
        return None


def turn_left(action):
    if action == (-1,0):
        return (0,-1)
    if action == (0,1):
        return (-1,0)
    if action == (1,0):
        return (0,1)
    if action == (0,-1):
        return (1,0)
    if action == None:
        return None

def go(state, direction):
    "Return the state that results from going in this direction."
    if direction is not None:
        state1 = map(add,state,direction)
        next_state = (state1[0],state1[1])
        if next_state in states:
            return next_state
        else:
            return None


'''Transition Model'''
def T(state, action):
    if action == None:
        return [(0.0, state)]
    else:
        return [(0.8, go(state,action)),(0.1, go(state, turn_right(action))), (0.1, go(state, turn_left(action)))]


start = (2,0)
#direction = (0,1)

#ans = T(state,direction)
#print ans

''' Action List'''
actions = []
actions.append((0,1))    #'>'
actions.append((-1,0))   #'^'
actions.append((0,-1))   #'<'
actions.append((1,0))    #'v'

#print actions

#print R[start]


'''Value Function'''
#V = dict([(s, 0) for s in states])
#print V

#for s in states:
#   print V[s]

def value_iteration(epsilon=0.001):
    U1 = dict([(s, 0) for s in states])
    U1[None] = 0
    gamma = 0.9
    while True:
        U = U1.copy() 
        delta = 0
        for s in states:
            U1[s] = R[s] + gamma * max([sum([p * U[s1] for (p, s1) in T(s, a)]) for a in actions])
            delta = max(delta, abs(U1[s] - U[s]))
        if delta < epsilon * (1 - gamma) / gamma:
             return U

U = value_iteration()

def best_policy(U):
    """Given an MDP and a utility function U, determine the best policy,
    as a mapping from state to action."""
    pi = {}
    for s in states:
        val = 0
        for a in actions:
            if (expected_utility(a, s, U) >= val):
                val = expected_utility(a, s, U)
                pi[s] = a
    return pi

def expected_utility(a, s, U):
    "The expected utility of doing a in state s, according to the MDP and U."
    return sum([p * U[s1] for (p, s1) in T(s, a)])


pi  = best_policy(U)

def to_arrows(policy):
    chars = {(0, 1):'>', (-1, 0):'^', (0, -1):'<', (1, 0):'v', None: '.'}
    return dict([(s, chars[a]) for (s, a) in policy.items()])

print to_arrows(pi)