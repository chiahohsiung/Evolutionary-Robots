import numpy as np 
import random
import math
import json


def distance(m1, m2):
    return np.linalg.norm(m1.p-m2.p)


def direction(m1, m2):
    # m2 to m1 
    vector = m2.p - m1.p
    vector /= np.linalg.norm(vector)
    return vector


g = np.array([0, 0, -9.81]).reshape((3, 1))	
ground_k = 100000
class Mass(object):
    """docstring for Mass"""
    
    def __init__(self, index, m, p, v=np.zeros((3, 1)), a=np.zeros((3, 1)), F=np.zeros((3, 1))):
        super(Mass, self).__init__()
        self.m = m # in kg
        self.p = p
        self.v = v
        self.a = a
        self.F = F
        self.index = index
        self.miu_s = 1
        self.miu_k = 0.8
    
    
    def update_pos(self, dt):
        zero_v = False
        if self.p[2] < 0:
            frict, zero_v = self.friction()
            self.F = self.F + frict
             
            self.F = self.F + ground_k * np.array([0,0,float(-self.p[2])]).reshape((3, 1))
            
        self.a = self.F / self.m
        self.a = self.a + g
        self.v = self.v + self.a * dt
        if zero_v:
            self.v = np.concatenate((np.zeros((2, 1)), self.v[2:]))
        self.v = self.v * 0.999
        self.p = self.p + self.v * dt
    
    def friction(self):
        if self.F[2] >= 0:
            return (np.zeros((3, 1)), 0)
        else:
            F_h = np.concatenate((self.F[:2], np.zeros((1, 1))))
            F_n = np.concatenate((np.zeros((2, 1)), self.F[2:]))
            direction_h = F_h / np.linalg.norm(F_h)

            # [1 2 3] 
            # F_h = [1 2 0] F_n = [0 0 3]
            F_h_abs = np.linalg.norm(F_h)
            F_n_abs = -float(F_n[2])

            if F_h_abs >= F_n_abs * self.miu_s: # kinetic friction -< add a ground friction -F_n * miu_k
                return (-direction_h * F_n_abs * self.miu_k, 0)
            
            else: # static friction -> add a ground friction -F_h
                return (direction_h * -F_h_abs, 1)
                
                                
class Spring(object):
    """docstring for Spring"""
    def __init__(self, k, l_0, m1_index, m2_index, b, c):
        super(Spring, self).__init__()
        self.k = k # spring constant
        self.l_0 = l_0 # rest length in meter
        self.m1_index = m1_index # index od the first mass it connects to
        self.m2_index = m2_index
        self.b = b
        self.c = c 


class Material(object):
	"""
	"""
	def __init__(self, center=0, radius=0.1, k=10000, b=0, c=0):
		super(Material, self).__init__()
		self.center = center
		self.radius = radius
		# spring sin equation l_0_init = (1 + b * sin(wt + c))
		self.k = k
		self.b = b
		self.c = c


def robot_shape():
	levels = [[[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.1, 0.1, 0]],
          [[0, 0, 0.1], [0.1, 0, 0.1], [0, 0.1, 0.1], [0.1, 0.1, 0.1]]]
	levels_arr = np.array(levels)
	levels_arr = levels_arr.reshape(-1, levels_arr.shape[-1]) + np.array([0, 0, 0])

	# levels_ext = levels_arr + np.array([0, 0.2, 0])
	# levels_arr = np.vstack((levels_arr, levels_ext)) + np.array([0, 0, 0.1])
	edges = []
	pre_level = None 
	for k in range(2):
	    offset = (k)* 4
	    cur_level = [i+offset for i in range(4)]
	    if not pre_level:
	        for i in range(len(cur_level)):
	            for j in range(i+1, len(cur_level)):
	                edges.append([cur_level[i], cur_level[j]])
	    else:
	        for i in range(len(cur_level)):
	            for j in range(i+1, len(cur_level)):
	                edges.append([cur_level[i], cur_level[j]])

	        for i in range(len(cur_level)):
	            for j in range(len(pre_level)):
	                edges.append([cur_level[i], pre_level[j]])
	    pre_level = cur_level
	# pre_level = []
	# for k in range(3):
	#     offset = (k) * 4 + 12
	#     cur_level = [i+offset for i in range(4)]
	#     if not pre_level:
	#         for i in range(len(cur_level)):
	#             for j in range(i+1, len(cur_level)):
	#                 edges.append([cur_level[i], cur_level[j]])
	#     else:
	#         for i in range(len(cur_level)):
	#             for j in range(i+1, len(cur_level)):
	#                 edges.append([cur_level[i], cur_level[j]])

	#         for i in range(len(cur_level)):
	#             for j in range(len(pre_level)):
	#                 edges.append([cur_level[i], pre_level[j]])
	#     pre_level = cur_level
	# for q in [6, 7, 10, 11]:
	#     for k in [16, 17, 20, 21]:
	#         edges.append([q, k])
	return (levels_arr, edges)


class Robot(object):
	"""docstring for Robot"""
	offset_ls = []

	def __init__(self, levels_and_edges):
		super(Robot, self).__init__()
		self.levels_arr = levels_and_edges[0] # for making mass
		self.edges = levels_and_edges[1] # for making springs
		# [[0.  0.  0. ]
		# [0.  0.  0.1]
		# [0.1 0.  0. ]
		# [0.1 0.  0.1]
		# [0.  0.1 0. ]
		# [0.  0.1 0.1]
		# [0.1 0.1 0. ]
		# [0.1 0.1 0.1]]
		self.mass_ls = self.build_robot_mass()
		self.spring_ls = []

	def apply_material(self, solution):
		# for each spring, find its closest material
		# solution [mat, mat, mat]
		# mat center k b c 

		for comb in self.edges:
		    i, j = comb
		    # spring mid point 
		    spring_p = (self.mass_ls[i].p + self.mass_ls[j].p) / 2

		    min_dis = float('inf')
		    min_index = []

		    for k, mat in enumerate(solution):

		    	cur_dis = np.linalg.norm(spring_p-self.mass_ls[mat.center].p) 
		    	if cur_dis == min_dis: 		    		
		    		min_index.append(k)	
		    	elif cur_dis < min_dis:
		    		min_dis = cur_dis
		    		min_index = [k]
		    # this might be buggy
		    k = np.mean([solution[i].k for i in min_index])
		    b = np.mean([solution[i].b for i in min_index])
		    c = np.mean([solution[i].c for i in min_index])
		    l_0 = distance(self.mass_ls[i], self.mass_ls[j])
		    spring = Spring(k, l_0, i, j, b, c)
		    self.spring_ls.append(spring)
		

	def build_robot_mass(self):
		mass_ls = []
		for i, pos in enumerate(self.levels_arr):
		    mass = Mass(i, m=0.1, p=(pos.reshape(3, 1)))
		    mass_ls.append(mass)
		
		return mass_ls


	def robot_center(self):
		aux_mass_ls = []
		for mass in self.mass_ls:
			aux_mass_ls.append(mass.p)
		return np.mean(np.array(aux_mass_ls), axis=0)


# Variation
def mutation(solution):
	index = np.random.randint(0, len(solution))

	if random.randint(0, 1):
		solution[index].center = min(7, solution[index].center + 1)
	else:
		solution[index].center = max(0, solution[index].center-1)

	solution[index].k = max(1000, solution[index].k + np.random.uniform(-1, 1) * 200)
	solution[index].b = max(0, min(1, solution[index].b + np.random.uniform(-1, 1) * 0.02))
	solution[index].c = solution[index].c + np.random.uniform(-1, 1) * math.pi 


def crossover(solution1, solution2):
	new_solution1 = solution1.copy()
	new_solution2 = solution2.copy()
	index1, index2 = np.random.randint(low=0, high=len(solution1), size=2)
	new_solution1[index1], new_solution2[index2] = new_solution2[index2], new_solution1[index1]

	return new_solution1, new_solution2


def evaluate_sol(solution, g, dt):
	global counter
	counter += 1
	print('counter', counter)
	# evaluate

	# make a robot and apply the solution to it 
	robot = Robot(robot_shape())
	robot.apply_material(solution)

	if counter == 1:
		# initial rest length of each spring
		# only do once add to the Robot class attribute
		offset_ls = []
		for spring in robot.spring_ls:
			l_0 = spring.l_0
			offset_ls.append(l_0)
		Robot.offset_ls = offset_ls

    # simulate the robot moving for (t_end - t) secs  
	robot_start = robot.robot_center()
	
	t = 0
	t_end = 2
	while t < t_end:
		t += dt
		# zero the force applied to every mass
		for i in range(len(robot.mass_ls)):
			robot.mass_ls[i].F = np.zeros((3, 1))

		for j, spring in enumerate(robot.spring_ls):
			m1 = robot.mass_ls[spring.m1_index]
			m2 = robot.mass_ls[spring.m2_index]
			w = 10
			spring.l_0 = (np.sin(w * t + spring.c) * spring.b + 1) * Robot.offset_ls[j]
			l = distance(m1, m2)
			F = spring.k * (l - spring.l_0) # check the direction
			F = direction(m1, m2) * F
			F = F.reshape(F.shape[0], -1)
			robot.mass_ls[spring.m1_index].F = robot.mass_ls[spring.m1_index].F + F
			robot.mass_ls[spring.m2_index].F = robot.mass_ls[spring.m2_index].F - F

		for i in range(len(robot.mass_ls)):
			robot.mass_ls[i].update_pos(dt)

	# calculate the 
	robot_end = robot.robot_center()
	travel_dis = np.linalg.norm(robot_end[:2] - robot_start[:2])
	# print('robot_start', robot_start)
	# print('robot_end', robot_end)
	# print('travel_dis', travel_dis)
	return travel_dis


counter = 0 
def main():
	popsize = 20
	num_of_mat = 3
	total_evaluation = 100
	
	# k = [10000, 2000, 5000]
	k_ls = np.arange(2, 10) * 1000.0
	b_init = 0.25
	# c = [0, 0, 0]
	mass_pos = [0 for i in range(8)]
	g = np.array([0, 0, -9.81]).reshape((3, 1))	
	dt = 0.0001
	
	solutions = {} # store solutions as dict for quick remove and access
	score_ls = {}
	# first generatioin
	for i in range(popsize):
		# each solution contains 3 materials
		solution = []
		for j in range(num_of_mat):
			center = random.randint(0, len(mass_pos)-1)
			k = np.random.choice(k_ls)
			b = np.random.uniform(0, 1) * b_init
			c = np.random.uniform(-1, 1) * math.pi
			
			mat = Material(center, 0.1, k, b, c)
			solution.append(mat)

		solutions[i] = solution
		score_ls[i] = evaluate_sol(solution, g, dt)

	print(score_ls)
	

	print('\nEVOLVING\n')
	print('Popsize: ' popsize)
	print('Evalutation: ', total_evaluation)
	gen = 0
	best_score_ls = []
	while counter < total_evaluation:
		gen += 1
		print('Gen: ', gen)
		# one generation
		index1, index2 = random.sample(range(popsize), 2)
		score_ls_cur = [score_ls[index1], score_ls[index2]]
		print('Chosen parents', index1, index2)

		parent1 = solutions[index1]
		parent2 = solutions[index2]

		# Crossover
		solution1, solution2 = crossover(parent1, parent2)

		# for i, solution in enumerate([solutions[index1], solutions[index2], solution1, solution2]):
		# 	print('solutions', i)
		# 	for mat in solution:
		# 		print(mat.center, mat.k, mat.b, mat.c)

		# Mutation
		mutation(solution1)
		mutation(solution2)

		# for i, solution in enumerate([solution1, solution2]):
		# 	print('solutions', i+2)
		# 	for mat in solution:
		# 		print(mat.center, mat.k, mat.b, mat.c)

		score_ls_cur += [evaluate_sol(solution1, g, dt), evaluate_sol(solution2, g, dt)]
		solutions_cur = [parent1, parent2, solution1, solution2]
		print(score_ls_cur)
		print('counter', counter)

		# Compare 
		print(sorted(range(len(score_ls_cur)), key=lambda k: score_ls_cur[k]))
		second, best = sorted(range(len(score_ls_cur)), key=lambda k: score_ls_cur[k])[-2:]

		score_ls[index1], score_ls[index2] = score_ls_cur[best], score_ls_cur[second]
		solutions[index1], solutions[index2] = solutions_cur[best], solutions_cur[second]
		best_score_ls.append(max(score_ls.values()))
	print('best_score_ls', best_score_ls)
	# save solution
	best_key = max(score_ls, key=score_ls.get) 
	best_solution = solutions[best_key]

	mat_ls = []
	for mat in best_solution:
		print(type(mat.center), type(mat.k), type(mat.b), type(mat.c))
		mat_ls.append([mat.center, mat.k, mat.b, mat.c])
		# exit()
	result = {'solution': mat_ls, 'best_score_ls': best_score_ls}
	print('result', result)
	output_path = 'cube_eval40_pop20.json'
	with open(output_path, 'w') as outfile:
		outfile.write(json.dumps(result, indent=4, sort_keys=True))

if __name__ == '__main__':
	main()



		
