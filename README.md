# UGP

Hybrid A*

car.py

Given the steering angle and the distance travelled by car maintaining this same angle, equations to find the new state x, y, theta.

Heuristics

1: Non-Holonomic-without-obstacles heuristic

This heuristic value is the length of the optimal Reeds-Shepp path from the pose to the goal without taking any obstacles into account.

To be precomputed offline and then translated and rotated to match the current goal

References:

https://www.projecteuclid.org/download/pdf_1/euclid.pjm/1102645450

https://gieseanw.wordpress.com/2012/11/15/reeds-shepp-cars/

2: Ignore Non-Holonomic, uses obstacle map 

Compute the shortest distance to goal by computing dynamic programming in 2D.

Final Heuristic value - max(1,2)

--

Experimenting using mdp.py 

value_iteration.py | Algorithm MDP_Value_Iteration | Terminal states to be considered - no action could be further taken

Questions:

Policy iteration for MDP
https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-410-principles-of-autonomy-and-decision-making-fall-2010/lecture-notes/MIT16_410F10_lec23.pdf

Pruning of value function constraints in POMDP

Is MDP/POMDP useful in real world applications?

References:
--
mdp.py, utils.py, mdp.txt

MDP in a grid-like environment (Value Iteration)

http://aima.cs.berkeley.edu/python/ 


--
maze1.pnm, maze2.pnm,maze_random_position.py

POMDP in a maze

http://jeremy.fix.free.fr/Softwares/pomdp.html


--
A simple particle filter example

https://github.com/mjl/particle_filter_demo

--
Monte Carlo POMDPs

http://robots.stanford.edu/papers/thrun.mcpomdp.pdf

--
Robot Planning in Partially Observable Continuous Domains

http://www.roboticsproceedings.org/rss01/p29.pdf
