import numpy as np
import random
import math
from copy import deepcopy
from collections import defaultdict
from functools import partial
import json
import open3d as o3d
import time
from numba import jit


def save_result(best_solution, output_path, best_score_ls, dot_list):
	mat_ls = []
	for mat in best_solution.material_ls:
		mat_ls.append([mat.center.tolist(), mat.k, mat.b, mat.c])
	
	point_dict = {}
	for k, v in best_solution.point_dict.items():
		point_dict[k] = v.tolist()
		
	edge_ls = [[float(list(comb)[0]), float(list(comb)[1]) ] for comb in best_solution.edge_ls]
	result = {'mat_ls': mat_ls, 'point_dict': point_dict, 'edge_ls': edge_ls, 
			  'best_score_ls': best_score_ls, 'dot_list': dot_list}

	with open(output_path, 'w') as outfile:
		outfile.write(json.dumps(result, indent=4, sort_keys=True))


def distance(m1, m2):
	return np.linalg.norm(m1.p-m2.p)

@jit(nopython=True)
def direction(m1p, m2p):
	# m2 to m1 
	vector = m2p - m1p
	vector /= np.linalg.norm(vector)
	return vector


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


class Solution(object):
	"""docstring for Solution"""
	def __init__(self, material_ls, point_dict, edge_ls):
		super(Solution, self).__init__()
		self.material_ls = material_ls
		# [Material1, Material2, Material3]
		# Material center k r b c
		self.point_dict = point_dict # level_arr {0:array, 1:array ...}
		# [[0.  0.  0. ]
		# [0.  0.  0.1]
		# [0.1 0.  0. ]
		# [0.1 0.  0.1]
		# [0.  0.1 0. ]
		# [0.  0.1 0.1]
		# [0.1 0.1 0. ]
		# [0.1 0.1 0.1]]
		self.edge_ls = edge_ls # edges 
		# self.bound = [[0, 0.1 ]]


class Robot(object):
	"""docstring for Robot"""
	# offset_ls = []

	def __init__(self):
		super(Robot, self).__init__()
		self.mass_ls = []
		self.spring_ls = []

	
	def apply_solution(self, solution):
		### Shape: Masses and Springs ###
		for i, pos in solution.point_dict.items():
			mass = Mass(i, m=0.1, p=(pos.reshape(3, 1)))
			self.mass_ls.append(mass)

		# for each spring, find its closest material
		# solution [mat, mat, mat]
		# mat center k b c 


		for comb in solution.edge_ls:
			i, j = comb
			# spring mid point 
			# print(len(self.mass_ls))
			# print('i, j ', i, j )
			spring_p = (self.mass_ls[i].p + self.mass_ls[j].p) / 2

			min_dis = float('inf')
			min_index = []
			
			for k, mat in enumerate(solution.material_ls):
				

				cur_dis = np.linalg.norm(spring_p-mat.center) 
				if cur_dis == min_dis: 		    		
					min_index.append(k)	
				elif cur_dis < min_dis:
					min_dis = cur_dis
					min_index = [k]
			
			# this might be buggy
			k = np.mean([solution.material_ls[i].k for i in min_index])
			b = np.mean([solution.material_ls[i].b for i in min_index])
			c = np.mean([solution.material_ls[i].c for i in min_index])
			l_0 = distance(self.mass_ls[i], self.mass_ls[j])
			spring = Spring(k, l_0, i, j, b, c)
			self.spring_ls.append(spring)


	def robot_center(self):
		aux_mass_ls = []
		for mass in self.mass_ls:
			aux_mass_ls.append(mass.p)
		return np.mean(np.array(aux_mass_ls), axis=0)


def evaluate_sol(solution, g, dt):
	global counter
	counter += 1
	print('evaluation: ', counter)
	# evaluate

	# make a robot and apply the solution to it 
	robot = Robot()
	robot.apply_solution(solution)
	# simulate the robot moving for (t_end - t) secs  
	# robot_start = robot.robot_center()
	
	t = 0
	t_end = 2
	is_start = True
	while t < t_end:
		# print('t', t)
		t += dt
		if t > 1.0 and is_start:
			is_start = False
			robot_start = robot.robot_center()
		# zero the force applied to every mass
		for i in range(len(robot.mass_ls)):
			robot.mass_ls[i].F = np.zeros((3, 1))

		for j, spring in enumerate(robot.spring_ls):
			m1 = robot.mass_ls[spring.m1_index]
			m2 = robot.mass_ls[spring.m2_index]
			w = 10
			cur_l_0 = (np.sin(w * t + spring.c) * spring.b + 1) * spring.l_0
			l = distance(m1, m2)
			F = spring.k * (l - cur_l_0) # check the direction
			F = direction(m1.p, m2.p) * F
			F = F.reshape(F.shape[0], -1)
			robot.mass_ls[spring.m1_index].F = robot.mass_ls[spring.m1_index].F + F
			robot.mass_ls[spring.m2_index].F = robot.mass_ls[spring.m2_index].F - F

		for i in range(len(robot.mass_ls)):
			robot.mass_ls[i].update_pos(dt)
		if robot.robot_center()[2] > 0.3:
			print('Fly')
			return 0
	# calculate the 
	robot_end = robot.robot_center()
	travel_dis = np.linalg.norm(robot_end[:2] - robot_start[:2])
	# print('robot_start', robot_start)
	# print('robot_end', robot_end)
	# print('travel_dis', travel_dis)
	return travel_dis



# Variation
def mutation(solution):
	### Material ###
	index = np.random.randint(0, len(solution.material_ls))
	center_x_add, center_y_add = np.random.uniform(-0.1, 0.2, size=2) * 0.3
	center_z_add = np.random.uniform(0.0, 0.1) * 0.1
	solution.material_ls[index].center = solution.material_ls[index].center + np.array([center_x_add, center_y_add, center_z_add]).reshape((3, 1))	
	solution.material_ls[index].center = np.clip(solution.material_ls[index].center, -0.1, 0.2)

	solution.material_ls[index].k = max(1000, solution.material_ls[index].k + np.random.uniform(-1, 1) * 200)
	solution.material_ls[index].b = max(0, min(1, solution.material_ls[index].b + np.random.uniform(-1, 1) * 0.02))
	solution.material_ls[index].c = solution.material_ls[index].c + np.random.uniform(-1, 1) * math.pi / 5

	### Shape ###
	choice = np.random.choice([0, 1, 2, 3], p=[0.3, 0.1, 0.3, 0.3])
	if choice == 0:
		# delete a spring
		delete_edge = random.sample(solution.edge_ls, k=1)[0]
		# print('delete_edge', delete_edge)
		solution.edge_ls.remove(delete_edge)
	
	elif choice == 1:
		# delete a mass
		key = random.choice(list(solution.point_dict.keys()))
		# print('key', key)
		# solution.point_ls = np.delete(solution.point_dict, index, axis=0)
		del solution.point_dict[key]
		# delete springs connected to the mass
		for comb in list(solution.edge_ls): 
			if index in comb:
				solution.edge_ls.remove(comb)

	elif choice == 2 :
		# add a spring 
		index1, index2 = np.random.choice(list(solution.point_dict.keys()), size=2, replace=False)
		# print('index1, index2', (index1, index2))
		if index2 < index1:
			index1, index2 = index2, index1
		if (index1, index2) not in solution.edge_ls:
			# print('index1, index2', (index1, index2))
			solution.edge_ls.add((index1, index2))

	elif choice == 3:
		# add a mass
		
		new_point = np.array(random.choice(new_mass_candidate)) #.reshape((1, 3))
		
		# print('new_point', new_point)
		# randomly select two masses to connect to 
		index1, index2, index3 = np.random.choice(list(solution.point_dict.keys()), size=3, replace=False)	
		# print('index1, index2', index1, index2)
		# put the new point and edges in the solution
		new_key = max(solution.point_dict.keys()) + 1
		solution.point_dict[new_key] = new_point
		solution.edge_ls.add((index1, new_key))
		solution.edge_ls.add((index2, new_key))
		solution.edge_ls.add((index3, new_key))

def crossover(solution1, solution2):
	num_of_mat = 2
	new_solution1 = deepcopy(solution1)
	new_solution2 = deepcopy(solution2)
	index1, index2 = np.random.randint(low=0, high=num_of_mat, size=2)
	new_solution1.material_ls[index1], new_solution2.material_ls[index2] = new_solution2.material_ls[index2], new_solution1.material_ls[index1]

	return new_solution1, new_solution2

counter = 0
g = np.array([0, 0, -9.81]).reshape((3, 1))	
ground_k = 100000

new_mass_candidate = [[-0.1, -0.1, 0],
					  [ 0.0, -0.1, 0],
					  [ 0.1, -0.1, 0],
					  [ 0.2, -0.1, 0],
					 
					  [-0.1,  0.0, 0],
					  [ 0.2,  0.0, 0],
 
					  [-0.1,  0.1, 0],
					  [ 0.2,  0.1, 0],

					  [-0.1,  0.2, 0],
					  [ 0.0,  0.2, 0],
					  [ 0.1,  0.2, 0],
					  [ 0.2,  0.2, 0],

					  [-0.1, -0.1, 0.1],
					  [ 0.0, -0.1, 0.1],
					  [ 0.1, -0.1, 0.1],
					  [ 0.2, -0.1, 0.1],
					 
					  [-0.1,  0.0, 0.1],
					  [ 0.2,  0.0, 0.1],
 
					  [-0.1,  0.1, 0.1],
					  [ 0.2,  0.1, 0.1],

					  [-0.1,  0.2, 0.1],
					  [ 0.0,  0.2, 0.1],
					  [ 0.1,  0.2, 0.1],
					  [ 0.2,  0.2, 0.1]]

def main():
	popsize = 100
	num_of_mat = 3
	total_evaluation = 2000	
	dt = 0.0002
	
	k_ls = np.arange(2, 10) * 500.0
	b_init = 0.25
	# c = [0, 0, 0]
	point_ls_init = np.array([[0.,  0.,  0.01 ],
							  [0.,  0.,  0.11],
							  [0.1, 0.,  0.01 ],
							  [0.1, 0.,  0.11],
							  
							  [0.,  0.1, 0.11],
							  [0.1, 0.1, 0.01],
							  [0.1, 0.1, 0.11]])
	point_dict_init = {}
	for i in range(point_ls_init.shape[0]):
		print('point_ls_init[i]', point_ls_init[i])
		point_dict_init[i] = point_ls_init[i]

	edge_ls_init = set([(i,j) for i in range(len(point_ls_init)) for j in range(i+1, len(point_ls_init))])

	solutions = {} # store solutions as dict for quick remove and access
	score_ls = {}
	dot_list = []
	for i in range(popsize):
		# each solution contains 3 materials

		material_ls = []
		for j in range(num_of_mat):

			# Materials
			center_x, center_y = np.random.uniform(-0.1, 0.2, size=2)
			center_z = np.random.uniform(0.0, 0.1)


			center = np.array([center_x, center_y, center_z]).reshape((3, 1))
			k = np.random.choice(k_ls)
			r = 0.1
			b = np.random.uniform(0, 1) * b_init
			c = np.random.uniform(-1, 1) * math.pi
			mat = Material(center, r, k, b, c)
			material_ls.append(mat)

			# Shape

		solution = Solution(material_ls, point_dict_init, edge_ls_init)
		solutions[i] = solution
		
		score_ls[i] = evaluate_sol(solution, g, dt)
		
	dot_list.append(list(score_ls.values()))	
	print(score_ls)

	# exit()

	print('\nEVOLVING\n')
	print('Popsize: ', popsize)
	print('Evalutation: ', total_evaluation)
	gen = 0
	best_score_ls = []
	while counter < total_evaluation:
		gen += 1
		print('Gen: ', gen)
		# one generation
		index1, index2 = random.sample(range(popsize), 2)
		score_ls_cur = [score_ls[index1], score_ls[index2]]

		parent1 = solutions[index1]
		parent2 = solutions[index2]


		# Crossover
		solution1, solution2 = crossover(parent1, parent2)

		# Mutation
		mutation(solution1)
		mutation(solution2)

		score_ls_cur += [evaluate_sol(solution1, g, dt), evaluate_sol(solution2, g, dt)]
		solutions_cur = [parent1, parent2, solution1, solution2]
		
		# Compare 	
		second, best = sorted(range(len(score_ls_cur)), key=lambda k: score_ls_cur[k])[-2:]

		score_ls[index1], score_ls[index2] = score_ls_cur[best], score_ls_cur[second]
		solutions[index1], solutions[index2] = solutions_cur[best], solutions_cur[second]
		best_score_ls.append(max(score_ls.values()))
		dot_list.append(list(score_ls.values()))	

	print('best_score_ls', best_score_ls)
	print('dot_list', dot_list)
	# save solution
	best_key = max(score_ls, key=score_ls.get) 
	best_solution = solutions[best_key]

	# SAVE RESULT
	save_result(solution, 'phase_c_test.json', best_score_ls, dot_list)

if __name__ == '__main__':
	main()
