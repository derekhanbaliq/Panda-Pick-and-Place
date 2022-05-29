import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


from lib.calculateFK import FK
import math



fk = FK()

class Node:
    def __init__(self, q0, q1, q2, q3, q4, q5, q6, parent):
        self.q0=q0
        self.q1=q1
        self.q2=q2
        self.q3=q3
        self.q4=q4
        self.q5=q5
        self.q6=q6
        self.parent=parent



def sample(lLim, uLim): # generate a random new point
    return np.array([\
                random.uniform(lLim[0], uLim[0]),\
                random.uniform(lLim[1], uLim[1]),\
                random.uniform(lLim[2], uLim[2]),\
                random.uniform(lLim[3], uLim[3]),\
                random.uniform(lLim[4], uLim[4]),\
                random.uniform(lLim[5], uLim[5]),\
                random.uniform(lLim[6], uLim[6]),\
                ])



def getdist(tree, q_n1, n2):
    q_n2 = [tree[n2].q0, tree[n2].q1, tree[n2].q2, tree[n2].q3, tree[n2].q4, tree[n2].q5, tree[n2].q6]

    n1_jointPositions, n1_T0e = FK.forward(fk, q_n1)
    n2_jointPositions, n2_T0e = FK.forward(fk, q_n2)

    dist = np.linalg.norm(n2_jointPositions[0] - n1_jointPositions[0])\
            + np.linalg.norm(n2_jointPositions[1] - n1_jointPositions[1])\
            + np.linalg.norm(n2_jointPositions[2] - n1_jointPositions[2])\
            + np.linalg.norm(n2_jointPositions[3] - n1_jointPositions[3])\
            + np.linalg.norm(n2_jointPositions[4] - n1_jointPositions[4])\
            + np.linalg.norm(n2_jointPositions[5] - n1_jointPositions[5])\
            + np.linalg.norm(n2_jointPositions[6] - n1_jointPositions[6])
    return dist

def nearest(tree, sampled_point): #function to find nearest node in the tree to the current sampled point
    min_dist = 10000 #initialize with a huge value
    for i in range (0, len(tree)):
        cur_dist = getdist(tree, sampled_point, i)
        if(cur_dist<min_dist):
            min_dist = cur_dist
            nearest_nodeidx =i
    return nearest_nodeidx

def spherepoints(q1,q2):
    phi=0
    theta=0
    r=0.1
    t_lim=int(10)
    omega=np.pi/t_lim
    q1_sphere=np.zeros((int(8*t_lim*t_lim),3))
    q2_sphere=np.zeros((int(8*t_lim*t_lim),3))
    index=0
    for i in range(8):
      x1=q1[i,0]
      y1=q1[i,1]
      z1=q1[i,2]
      x2=q2[i,0]
      y2=q2[i,1]
      z2=q2[i,2]
      for phi in range(t_lim):
        for theta in range(t_lim):
          q1_sphere[index,0]=x1+r*np.sin(phi*omega)*np.cos(theta*omega)
          q1_sphere[index,1]=y1+r*np.sin(phi*omega)*np.sin(theta*omega)
          q1_sphere[index,2]=z1+r*np.cos(phi*omega)
          q2_sphere[index,0]=x2+r*np.sin(phi*omega)*np.cos(theta*omega)
          q2_sphere[index,1]=y2+r*np.sin(phi*omega)*np.sin(theta*omega)
          q2_sphere[index,2]=z2+r*np.cos(phi*omega)
          index=index+1

    #print(np.shape(q1_sphere))
    return q1_sphere,q2_sphere

def isRobotCollided(q1, q2, map):
    # detect collisions for a robot configuration q
    sampled_configs, number_of_sample = sampleCollisionDetectionPoints(q1, q2, 0.2)

    sampled_jointPositions = []

    for config in sampled_configs:
        current_jointPositions, current_T0e = FK.forward(fk, config)
        #print(np.shape(current_jointPositions))
        sampled_jointPositions.append(current_jointPositions)
        """Sphere to check self collision"""
        for x in range(0, len(current_jointPositions)):
            """use broadcasting to obtain (x-a)+(y-b)+(z-c)"""
            R2= (current_jointPositions - current_jointPositions[x,:])
            #print(R2)
            #print(np.delete(R2,x,axis=0))
            """Remove that row which is being considered"""
            R2 = np.delete(R2,x,axis=0)
            R2 = np.array(R2) #
            #print(np.sum(R2**2, axis=1))
            #print(np.min(np.sum(R2**2, axis=1)-0.06**2))
            """square it and subtract R^2 where R=0.06. Now check if the minimum value is less than 0. If it is, then the point (a,b,c) is inside the 0.06R sphere"""
            if((np.min(np.sum(R2**2, axis=1)-0.06**2))<0):
                print("Self collision between joints")
                return True

        if(current_T0e[2,3]<=0.0):
            """Check z coordinate of end effector"""
            print("End effector too close to ground")
            return True
        #print(sampled_jointPositions)
        #print(np.shape(sampled_jointPositions[0]))

    q1_jointPositions, q1_T0e = FK.forward(fk, q1)
    #print(q1_T0e)
    #print(q1_T0e[2,3])
    q2_jointPositions, q2_T0e = FK.forward(fk, q2)
    for sample_index in range(number_of_sample - 1):
        for obstacle in map.obstacles:
            q1,q2=spherepoints(sampled_jointPositions[sample_index],sampled_jointPositions[sample_index + 1])
            if (True in detectCollision(q1, q2, obstacle)):

                #print("\nRobot collides with obstacles\n")
                #print(obstacle)
                return True
    #print("\nRobot does not collide with obstacles\n")
    return False

def sampleCollisionDetectionPoints(q1, q2, interpolate_percent):
    sampled_configs = []
    increment = (q2 - q1)/(1 / interpolate_percent)
    interpolation_completion = 0
    number_of_sample = 0
    while (interpolation_completion != 1):
        q1 = q1 + increment
        sampled_configs.append(q1)
        interpolation_completion = interpolation_completion + interpolate_percent
        number_of_sample += 1
    sampled_configs.append(q2)
    number_of_sample += 1
    return sampled_configs, number_of_sample


def find_path(tree, latest_added_node, q_target):
    current_node = np.array(latest_added_node)

    print(current_node)
    #print(np.shape(current_node))
    found_path = []
    print("Target", type(q_target))
    found_path.append(np.array(q_target))
    found_path.append(current_node[0:7])
    #print(type(current_node))
    while(1):
        par = int(current_node[-1])
        #print("Parent number:", par)
        if(par==-1): #check if we reached initial point
            break
        current_node = [tree[par].q0, tree[par].q1, tree[par].q2, tree[par].q3, tree[par].q4, tree[par].q5, tree[par].q6, tree[par].parent]
        #print(type(current_node))
        #found_path.append(current_node)
        #print(current_node)
        found_path.append(current_node[0:7])

    return found_path

def is_start_and_goal_valid(start, goal, upper, lower):

    for i in range(7):
        if not start[i] <= upper[i] or not start[i] >= lower[i]:
            print("wrong config for start[{}]!".format(i))
            return False
        if not goal[i] <= upper[i] or not goal[i] >= lower[i]:
            print("wrong config for goal[{}]!".format(i))
            return False

    print("start and goal q are valid!")
    return True

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []
    tree = []
    max_iter = 1000# number of nodes

    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
    if not is_start_and_goal_valid(start, goal, upperLim, lowerLim):
        return path

    tree.append(Node(start[0], start[1], start[2], start[3], start[4], start[5], start[6], -1)) # add start q to tree and set its parent as -1 for detecting path completion in find_path
    q_start = start
    q_target = goal

    for j in range(0, max_iter):
        q_rand = sample(lowerLim, upperLim) #returns 1x3 array
        #print("Sample q: ", q_rand)
        nearestnodeindex = nearest(tree, q_rand) #returns index of nearest node on the tree to the currently sampled point

        #print("Point closer to that point: ", newnode)

        near_pt_on_tree = [tree[nearestnodeindex].q0, tree[nearestnodeindex].q1, tree[nearestnodeindex].q2, tree[nearestnodeindex].q3, tree[nearestnodeindex].q4, tree[nearestnodeindex].q5, tree[nearestnodeindex].q6]
        #print("Node on tree: ", near_pt_on_tree)
        #print(newnode)
        #print(map[0][0])
        #collision = detectCollisionOnce(np.array(near_pt_on_tree), np.array(newnode), map[0][0])
        collision = isRobotCollided(q_rand, near_pt_on_tree, map)

        nnode=[q_rand[0],q_rand[1],q_rand[2],q_rand[3],q_rand[4],q_rand[5],q_rand[6], nearestnodeindex]

        #if(collision==True):
            #print("Robot collides for this random q.")
        if(collision==False):
            #print("Safe configuration found!")
            tree.append(Node(q_rand[0],q_rand[1],q_rand[2],q_rand[3],q_rand[4],q_rand[5],q_rand[6],nearestnodeindex))
            goal_flag = isRobotCollided(q_target, q_rand, map)
            if(goal_flag==False):
                found_path=True
                print("Found path!")
                path = find_path(tree, nnode, q_target)
                print(np.shape(path))
                path = path[::-1]
                # print(path)
                break

    #tree.append(Node(1,2,3,4))
    #tree.append(Node(1,2,3,4))
    # get joint limits
    #print(np.shape(tree)) #returns (2,)
    #print(len(tree)) # returns 2
    #print(tree[0].x)
    return path


if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    #print((map_struct)) #returns map(obstacles=array([[ 0.15, -0.3, 0.496825, 0.45, 0.3, 0.503175]]))
    print("Path: ", path)

    #print(len(path))
    """
    x = []
    y = []
    z = []
    for m in range(0, len(path)):
        x.append(path[m][0])
        y.append(path[m][1])
        z.append(path[m][2])

    fig = plt.figure()
    ax = plt.axes(projection ='3d')
    ax.plot3D(x, y, z, 'green')
    ax.scatter(map_struct[0][0][0],map_struct[0][0][1],map_struct[0][0][2])
    ax.scatter(map_struct[0][0][3],map_struct[0][0][4],map_struct[0][0][5])
    ax.set_title('3D line plot')
    plt.show()
    """
