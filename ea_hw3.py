import numpy as np
import time 

# Step 1 DS for masses and springs
class Mass(object):
	"""docstring for Mass"""
	g = np.array([0, 0, -9.81]).reshape((3, 1))
	def __init__(self, index, m, p, v=np.zeros((3, 1)), a=np.zeros((3, 1)), F=np.zeros((3, 1))):
		super(Mass, self).__init__()
		self.m = m # in kg
		self.p = p
		self.v = v
		self.a = a
		self.F = F
		self.index = index
		self.ground_k = 1000

	def update_pos(self, dt):
		# print('self.F', self.F)
		# Ground Force
		if self.p[2] < 0:
			print('self.F[2]', self.F[2])
			self.F[2] = self.F[2] + self.ground_k * (0 - self.p[2])
			print('self.F[2]', self.F[2])
		# gravity

		self.a = self.F / self.m
		print('self.a', self.a)
		print('g', g)
		self.a = self.a + g
		print('self.a', self.a)
		# print('self.a * dt', self.a * dt)
		# print('self.v', self.v)
		# print('self.v + self.a * dt', self.v + self.a * dt)
		self.v = self.v + self.a * dt
		# print('self.v', self.v)
		self.p = self.p + self.v * dt

class Spring(object):
	"""docstring for Spring"""
	def __init__(self, k, l_0, m1_index, m2_index):
		super(Spring, self).__init__()
		self.k = k # spring constant
		self.l_0 = l_0 # rest length in meter
		self.m1_index = m1_index # index od the first mass it connects to
		self.m2_index = m2_index

# Populate the DS to a 3D cube
def createGrid(bounds=[[-1, -1, -1], [1, 1, 1]], dr=0.1):
    """
    retrun a grid of points shaped (n,3) given bounds and discritization radius
    the bounds are updated and also returned
    input:
        bounds: [(x_low,y_low,z_low),(x_high,y_high,z_high)]
        dr: discretization radius of the grid
    output:
        xyz_grid: a grid of points numpy array of (n,3)
        bounds: updated bounds
        nx,ny,nz: number of points in x,y,z direction
    """
    # round to integer, type is still float
    bounds = bounds/dr
    bounds = np.stack((np.floor(bounds[0]), np.ceil(bounds[1])))*dr
#     print("bounds=\n", bounds)
    # number of points in x,y,z direction:(nx,ny,nz)
    nx, ny, nz = np.ceil((bounds[1]-bounds[0])/dr).astype(int)
    x = np.linspace(bounds[0, 0], bounds[0, 0]+(nx-1)*dr, num=nx)
    y = np.linspace(bounds[0, 1], bounds[0, 1]+(ny-1)*dr, num=ny)
    z = np.linspace(bounds[0, 2], bounds[0, 2]+(nz-1)*dr, num=nz)
    # a flattened grid of xyzs of the vertices
    xyz_grid = np.stack(np.meshgrid(x, y, z), axis=-1).reshape((-1, 3))
    return xyz_grid, bounds, (nx, ny, nz)


##################### create the points #####################################
bounds = np.array([[-1,-1,-1],[1,1,1]])*0.1 # [lower_bound,upper_bound]
radius_grid = 0.2 # discretization radius of the grid
mass_pos,bounds,_ = createGrid(bounds = bounds, dr = radius_grid) # create a grid of masses (xyzs)
mass_pos = mass_pos + (0, 0, -mass_pos[:,-1].min()) # move the vertices to z>=0

# setup the point cloud data
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(mass_pos)

# initialize first position
mass_pos =np.array([[2, 2, -1], [1, 1, 1]], dtype=float)
mass_ls = []
for i, pos in enumerate(mass_pos):
	mass = Mass(i, m=0.1, p=(pos.reshape(3, 1)))
	mass_ls.append(mass)


def distance(m1, m2):
	return np.linalg.norm(m1.p-m2.p)

spring_ls = []
for i in range(len(mass_ls)):
	for j in range(i+1, len(mass_ls)):
		k = 1000
		l_0 = distance(mass_ls[i], mass_ls[j])

		spring = Spring(k, l_0, i, j)
		spring_ls.append(spring)
	



# Step 3 Global variable
g = np.array([0, 0, -9800]).reshape((3, 1))
dt = 0.01
t = 0

def direction(m1, m2):
	# m2 to m1 
	vector = m2.p - m1.p
	vector /= np.linalg.norm(vector)

	return vector

# simulate
# while t < 5:
t += dt
for spring in spring_ls:
	m1 = mass_ls[spring.m1_index]
	m2 = mass_ls[spring.m2_index]
	l_0 = spring.l_0
	l = distance(m1, m2)
	l = 2.414
	F = spring.k * (l - l_0) # check the direction
	F = direction(m1, m2) * F
	F = F.reshape(F.shape[0], -1)
	mass_ls[spring.m1_index].F, mass_ls[spring.m2_index].F = F, -F

for i in range(len(mass_ls)):
	print('\n')
	mass_ls[i].update_pos(dt)
	print(mass_ls[i].p)















		
