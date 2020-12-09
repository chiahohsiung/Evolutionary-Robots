import numpy as np 
import random
import math
import json
import open3d as o3d
import time
from phase_b_solution import Robot, Material, Spring, Mass, robot_shape, distance, direction

def setView(ctr,camera_pos=(1, 1, 1), lookat=(1, 1, 0.3), up=(0, 0, 0.1)):
    """
    set the view given a view control handel ctr
    """
    ctr.set_constant_z_far(1000) # camera z far clip plane
    ctr.set_constant_z_near(0.01)# camera z near clip plane
    ctr.set_front(camera_pos)
    ctr.set_lookat(lookat)
    ctr.set_up(up)
def customVisualization(geomtry_list):
    """
    helper function to create a visualization given a list of o3d geometry
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for g in geomtry_list:
        vis.add_geometry(g)
    ctr = vis.get_view_control()
    setView(ctr)
    vis.run()
    vis.destroy_window() # close the window when finished


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

def createPlane(r=10, dr=0.1):
    """
    return a plane located at (0,0,0),and with plane normal = (0,0,1)
    r: radius of the plane
    dr:discretization radius of the grid
    """
    bounds = np.array([[-r, -r, 0],[r, r, 0]])/dr
    bounds = np.stack((np.floor(bounds[0]), np.ceil(bounds[1])))*dr
    nx, ny, nz = np.ceil((bounds[1]-bounds[0])/dr).astype(int)
#     print(nx,ny)
    xyz = np.reshape([[[[i, j, 0], [i+1, j, 0], [i, j+1, 0],
                       [i, j+1, 0], [i+1, j, 0], [i+1, j+1, 0]] for i in range(nx-1)] for j in range(ny-1)], (-1, 3))
    xyz = (xyz - ((nx-1)/2,(ny-1)/2,0))*dr
#     xyz, bounds, (nx, ny, nz) = create_grid(bounds, dr)
#     print(nx, ny, nz)
    triangles = np.arange(xyz.shape[0]).reshape((-1,3))
    plane = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(
        xyz), o3d.utility.Vector3iVector(triangles))
    # assign checkerboard color pattern
    c0 = (0.729, 0.78, 0.655) # first color
    c1 = (0.533, 0.62, 0.506) # second color
    colors = np.reshape([[np.tile(c0 if (i+j)%2 else c1,(6,1)) for i in range(nx-1)] for j in range(ny-1)],(-1,3))
    plane.vertex_colors = o3d.utility.Vector3dVector(colors)
    plane.compute_triangle_normals()
    return plane

# Step 3 Global variable
g = np.array([0, 0, -9.81]).reshape((3, 1))
ground_k = 10000
dt = 0.0001
t = 0


input_file = 'cube_eval20_pop10.json'
with open(input_file, 'r') as read_file:
    result = json.load(read_file)

solution = []
for mat in result['solution']:
    center = mat[0]
    k = mat[1]
    b = mat[2]
    c = mat[3]
    mat = Material(center, 0.1, k, b, c)
    solution.append(mat)

for mat in solution:
    print(mat.center, mat.k, mat.b, mat.c)

# make a robot and apply the solution to it 
robot = Robot(robot_shape())
robot.apply_material(solution)

# if counter == 1:
#   # initial rest length of each spring
#   # only do once add to the Robot class attribute

#   print('Hi')
offset_ls = []
for spring in robot.spring_ls:
    l_0 = spring.l_0
    offset_ls.append(l_0)
Robot.offset_ls = offset_ls

# # simulate the robot moving for (t_end - t) secs  
# robot_start = robot.robot_center()

# t = 0
# t_end = 2
# while t < t_end:
#   t += dt
#   # zero the force applied to every mass
#   for i in range(len(robot.mass_ls)):
#       robot.mass_ls[i].F = np.zeros((3, 1))

    # for j, spring in enumerate(robot.spring_ls):
    #   m1 = robot.mass_ls[spring.m1_index]
    #   m2 = robot.mass_ls[spring.m2_index]
    #   w = 10
    #   spring.l_0 = (np.sin(w * t + spring.c) * spring.b + 1) * Robot.offset_ls[j]
    #   l = distance(m1, m2)
    #   F = spring.k * (l - spring.l_0) # check the direction
    #   F = direction(m1, m2) * F
    #   F = F.reshape(F.shape[0], -1)
    #   robot.mass_ls[spring.m1_index].F = robot.mass_ls[spring.m1_index].F + F
    #   robot.mass_ls[spring.m2_index].F = robot.mass_ls[spring.m2_index].F - F

    # for i in range(len(robot.mass_ls)):
    #   robot.mass_ls[i].update_pos(dt)
# robot_end = robot.robot_center()
# travel_dis = np.linalg.norm(robot_end[:2] - robot_start[:2])
# # print('robot_start', robot_start)
# # print('robot_end', robot_end)
# print('travel_dis', travel_dis)

plane = createPlane()
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(robot.levels_arr)

# create the line set data
lsd = o3d.geometry.LineSet()
lsd.points = o3d.utility.Vector3dVector(robot.levels_arr) # (n by 3) arry of points xyzs
lsd.lines = o3d.utility.Vector2iVector(robot.edges) # (n by 2) indices of the edges

def DropCubeVisualization():
    """
    example for the non-blocking visualization
    """    
    ended=False
    def signalEnd(vis):
        nonlocal ended
        ended =True
        
    vis = o3d.visualization.VisualizerWithKeyCallback()
    
    # ref:https://www.glfw.org/docs/3.3/group__keys.html
    # press key Q or ESC to close the window
    vis.register_key_callback(81, signalEnd)# key Q 
    vis.register_key_callback(256, signalEnd)# key escape
    vis.create_window()

    # add geometry
    vis.add_geometry(lsd)
    vis.add_geometry(plane)
    vis.add_geometry(pcd)

    # view control

    ctr = vis.get_view_control()
    setView(ctr)
    
    mass_pos_t = np.copy(robot.levels_arr)
    # mass_ls_t = mass_pos.copy()
    spring_ls_t = robot.spring_ls.copy()
  
    t = 0
    t_prev = 0
    while (not ended):
        t += dt
        print('t', t)
        for i in range(len(robot.mass_ls)):
            robot.mass_ls[i].F = np.zeros((3, 1))
        for j, spring in enumerate(robot.spring_ls):
            m1 = robot.mass_ls[spring.m1_index]
            m2 = robot.mass_ls[spring.m2_index]
            w = 50
            spring.l_0 = (np.sin(w * t + spring.c) * spring.b + 1) * Robot.offset_ls[j]
            l = distance(m1, m2)
            F = spring.k * (l - spring.l_0) # check the direction
            F = direction(m1, m2) * F
            F = F.reshape(F.shape[0], -1)
            robot.mass_ls[spring.m1_index].F = robot.mass_ls[spring.m1_index].F + F
            robot.mass_ls[spring.m2_index].F = robot.mass_ls[spring.m2_index].F - F

        for i in range(len(robot.mass_ls)):
            robot.mass_ls[i].update_pos(dt)

        if t - t_prev > 1./60.:# update renderer
            t_prev = t
            mass_pos_ls = []
            for mass in robot.mass_ls:
                mass_pos_ls.append(mass.p)
            mass_pos_t = np.array(mass_pos_ls).reshape((-1, 3))
            lsd.points = o3d.utility.Vector3dVector(mass_pos_t)
            vis.update_geometry(lsd)
            pcd.points = o3d.utility.Vector3dVector(mass_pos_t)
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

    
    vis.destroy_window() # close the window when finished
    
DropCubeVisualization()
