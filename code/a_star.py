#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import time


def workspace_check(x_check, y_check):
    """
        In this function the given value is checked to see if it lies in the workspace

        Parameters
        ----------
        x_check: X coordinate of given input
        y_check: Y coordinate of given input

        Returns
        -------
        True  : If the input lies inside workspace
        False : If the input lies outside workspace
    """

    if (x_check >= 0) and (y_check >= 0) and (x_check <= 10) and (y_check <= 10):
        return True
    else:
        return False


def obstacle_space(x_check, y_check):
    """
        In this function the given value is checked to see if it lies in the obstacle space

        Parameters
        ----------
        x_check: X coordinate of given input
        y_check: Y coordinate of given input

        Returns
        -------
        True  : If the input lies outside obstacle space
        False : If the input lies inside obstacle space
    """

    flag1, flag2, flag3, flag4, flag5 = (True, True, True, True, True)

    # For Circle 1 with center at (2, 2) and radius 1 unit
    if (x_check - 2) ** 2 + (y_check - 2) ** 2 - (1 + clearance) ** 2 < 0:
        flag1 = False

    # For Circle 2 with center at (2, 8) and radius 1 unit
    if (x_check - 2) ** 2 + (y_check - 8) ** 2 - (1 + clearance) ** 2 < 0:
        flag2 = False

    # For Square with center at (1, 5) and dimension 1.5x1.5
    if 0.25 - clearance < x_check < 1.75 + clearance and 4.25 - clearance < y_check < 5.75 + clearance:

        if 0.25 - clearance < x_check < 0.25 and 4.25 - clearance < y_check < 4.25 and \
                (x_check - 0.25) ** 2 + (y_check - 4.25) ** 2 - clearance ** 2 >= 0:
            flag3 = True

        elif 0.25 - clearance < x_check < 0.25 and 5.75 < y_check < 5.75 + clearance and \
                (x_check - 0.25) ** 2 + (y_check - 5.75) ** 2 - clearance ** 2 >= 0:
            flag3 = True

        elif 1.75 < x_check < 1.75 + clearance and 5.75 < y_check < 5.75 + clearance and \
                (x_check - 1.75) ** 2 + (y_check - 5.75) ** 2 - clearance ** 2 >= 0:
            flag3 = True

        elif 1.75 < x_check < 1.75 + clearance and 4.25 - clearance < y_check < 4.25 and \
                (x_check - 1.75) ** 2 + (y_check - 4.25) ** 2 - clearance ** 2 >= 0:
            flag3 = True

        else:
            flag3 = False

    # For Rectangle 1 with center at (5, 5) and dimension 2.5x1.5
    if 3.75 - clearance < x_check < 6.25 + clearance and 4.25 - clearance < y_check < 5.75 + clearance:

        if 3.75 - clearance < x_check < 3.75 and 4.25 - clearance < y_check < 4.25 and \
                (x_check - 3.75) ** 2 + (y_check - 4.25) ** 2 - clearance ** 2 >= 0:
            flag4 = True

        elif 3.75 - clearance < x_check < 3.75 and 5.75 < y_check < 5.75 + clearance and \
                (x_check - 3.75) ** 2 + (y_check - 5.75) ** 2 - clearance ** 2 >= 0:
            flag4 = True

        elif 6.25 < x_check < 6.25 + clearance and 5.75 < y_check < 5.75 + clearance and \
                (x_check - 6.25) ** 2 + (y_check - 5.75) ** 2 - clearance ** 2 >= 0:
            flag4 = True

        elif 6.25 < x_check < 6.25 + clearance and 4.25 - clearance < y_check < 4.25 and \
                (x_check - 6.25) ** 2 + (y_check - 4.25) ** 2 - clearance ** 2 >= 0:
            flag4 = True

        else:
            flag4 = False

    # For Rectangle 2 with center at (8, 3) and dimension 1.5x2.0
    if 7.25 - clearance < x_check < 8.75 + clearance and 2.0 - clearance < y_check < 4.0 + clearance:

        if 7.25 - clearance < x_check < 7.25 and 2.0 - clearance < y_check < 2.0 and \
                (x_check - 7.25) ** 2 + (y_check - 2.0) ** 2 - clearance ** 2 >= 0:
            flag5 = True

        elif 7.25 - clearance < x_check < 7.25 and 4.0 < y_check < 4.0 + clearance and \
                (x_check - 7.25) ** 2 + (y_check - 4.0) ** 2 - clearance ** 2 >= 0:
            flag5 = True

        elif 8.75 < x_check < 8.75 + clearance and 4.0 < y_check < 4.0 + clearance and \
                (x_check - 8.75) ** 2 + (y_check - 4.0) ** 2 - clearance ** 2 >= 0:
            flag5 = True

        elif 8.75 < x_check < 8.75 + clearance and 2.0 - clearance < y_check < 2.0 and \
                (x_check - 8.75) ** 2 + (y_check - 2.0) ** 2 - clearance ** 2 >= 0:
            flag5 = True

        else:
            flag5 = False

    flag = flag1 and flag2 and flag3 and flag4 and flag5

    return flag


def find_index(node_point):
    x_pt, y_pt = node_point

    x_pt_mid = threshold_distance * (0.5 + int(x_pt / threshold_distance))
    y_pt_mid = threshold_distance * (0.5 + int(y_pt / threshold_distance))

    # ----------------------- For x_pt -----------------------

    if x_pt <= x_pt_mid:
        x_temp_ = threshold_distance * int(x_pt / threshold_distance)

    else:
        x_temp_ = threshold_distance * int(1 + x_pt / threshold_distance)

    # ----------------------- For y_pt ----------------

    if y_pt <= y_pt_mid:
        y_temp_ = threshold_distance * int(y_pt / threshold_distance)

    else:
        y_temp_ = threshold_distance * int(1 + y_pt / threshold_distance)

    node_index = np.intersect1d(np.where(np.array(X) == x_temp_), np.where(np.array(Y) == y_temp_))[0]

    return node_index


def a_star(parent_index):
    x_parent, y_parent = node_pt[parent_index]
    parent_angle = node_angle[parent_index]
    parent_cost = node_cost[parent_index]
    iter_bin.append([])  # appending an empty list for each iteration

    # ------------------------------------------ Performing action sets ------------------------------------------------

    actions = [[5, 5], [10, 10], [5, 0], [0, 5], [5, 10], [10, 5]]  # mentioned in class

    for action in actions:
        t = 0
        x_temp = x_parent
        y_temp = y_parent
        theta_temp = parent_angle
        temp_distance = 0

        omega_left, omega_right = action
        temp_x = [x_temp]
        temp_y = [y_temp]

        while t < 1:
            t = t + dt
            x_temp0 = x_temp
            y_temp0 = y_temp
            theta_temp0 = theta_temp % (2 * math.pi)

            dx_temp = 0.5 * r_wheel * (omega_left + omega_right) * math.cos(theta_temp) * dt
            dy_temp = 0.5 * r_wheel * (omega_left + omega_right) * math.sin(theta_temp) * dt
            dtheta_temp = (r_wheel / L_wheel) * (omega_right - omega_left) * dt

            x_temp = x_temp0 + dx_temp
            y_temp = y_temp0 + dy_temp
            theta_temp = (theta_temp0 + dtheta_temp) % (2 * math.pi)

            temp_x.append(x_temp)
            temp_y.append(y_temp)
            temp_distance += math.sqrt(dx_temp ** 2 + dy_temp ** 2)

        is_valid = workspace_check(temp_x[-1], temp_y[-1]) and obstacle_space(temp_x[-1], temp_y[-1])

        if is_valid:

            neighbor_pt = np.array([temp_x[-1], temp_y[-1]])
            neighbor_index = find_index(neighbor_pt)

            if neighbor_index not in visited_nodes_index:

                if node_cost[neighbor_index] > parent_cost + temp_distance:
                    node_cost[neighbor_index] = parent_cost + temp_distance
                    goal_cost[neighbor_index] = math.sqrt(np.sum((neighbor_pt - goal_pt) ** 2))
                    node_pt[neighbor_index] = neighbor_pt
                    node_angle[neighbor_index] = theta_temp
                    track.update({neighbor_index: parent_index})
                    path.update({neighbor_index: {'x': temp_x, 'y': temp_y, 'UL': omega_left, 'UR': omega_right}})

                    if neighbor_index not in unvisited_nodes_index:
                        unvisited_nodes_index.append(neighbor_index)

                    iter_bin[-1].append({})  # updating an empty list for each action as an empty dictionary
                    iter_bin[-1][-1].update({'x': temp_x, 'y': temp_y})

    node_flag[parent_index] = 1
    unvisited_nodes_index.remove(parent_index)
    visited_nodes_index.append(parent_index)
    return 0


# -------------------------------------------------Parameters-----------------------------------------------------------
r_wheel = 0.038  # in m, radius of wheels
L_wheel = 0.354  # in m, distance between two wheels
# r_robot = 0.177  # in m, radius of robot
dt = 0.1   # in s, differential time needed for integration
threshold_distance = 0.1  # in m, threshold distance between each node
goal_threshold_radius = 0.2  # in m, threshold radius for goal node
# clearance_obstacle = 0.123  # in m, clearance for obstacles
clearance = 0.25  # clearance_obstacle + r_robot  # in m, effective clearance

# -----------------------------------------Discretizing the workspace---------------------------------------------------
X_ = np.linspace(0, 10, int(10 / threshold_distance) + 1)
Y_ = np.linspace(0, 10, int(10 / threshold_distance) + 1)
X = []
Y = []
for ix in range(len(X_)):
    for iy in range(len(Y_)):
        X.append(X_[ix])
        Y.append(Y_[iy])

# --------------------------------------------------Variables-----------------------------------------------------------
node_pt = np.zeros((len(X), 2))  # X & Y coordinates for each node
node_angle = np.zeros(len(X))  # orientation (in radians) of robot at each node
node_flag = np.zeros(len(X))  # will take either 0 (if unvisited) or 1 (if visited) for each node
node_cost = np.zeros(len(X)) + np.inf  # cost to go from start pt for each node
goal_cost = np.zeros(len(X)) + np.inf  # cost to go from each node to goal pt
visited_nodes_index = []  # keeps a list of visited nodes
track = {}  # tracks parent node from child node
path = {}  # stores coordinates and angular velocities of both the wheels for every action with child index as key
iter_bin = []  # stores coordinates for every action with iterator as key

X_start = 1.0
Y_start = 1.0
angle_start = 0.0  # in radian
start_pt = np.array([X_start, Y_start])
start_index = find_index(start_pt)

X_goal = 9.0
Y_goal = 9.0
goal_pt = np.array([X_goal, Y_goal])
goal_index = find_index(goal_pt)

# ----------------------------------------------------------------------------------------------------------------------
node_pt[start_index] = start_pt
node_cost[start_index] = 0.0  # assigning cost of start node to zero
goal_cost[start_index] = math.sqrt(np.sum((start_pt - goal_pt) ** 2))
node_angle[start_index] = angle_start
unvisited_nodes_index = [start_index]

# ----------------------------------------------------------------------------------------------------------------------
start_time = time.time()
iterator = 0
goal_flag = 0

print('\n\nSolving.........')
print('\nIteration # \t Time (mins.)\n')

while goal_flag == 0:

    if iterator % 100 == 0 and iterator != 0:
        mid_time = (time.time() - start_time) / 60
        print(' {0} \t\t {1:1.3f}'.format(iterator, mid_time))

    temp_cost = node_cost[unvisited_nodes_index] + goal_cost[unvisited_nodes_index]
    temp = np.argmin(temp_cost)
    next_node_index = unvisited_nodes_index[temp]

    if goal_cost[next_node_index] <= goal_threshold_radius:
        goal_flag = 1
        iterator += 1
        end_time = time.time()
        total_time = (end_time - start_time) / 60
        print('\n\nNumber of iterations taken to reach goal state: {}'.format(iterator))
        print('\nTime taken to find optimal (shortest) path: {0:1.3f} min'.format(total_time))

        node_flag[next_node_index] = 1
        unvisited_nodes_index.remove(next_node_index)
        visited_nodes_index.append(next_node_index)
        print('\n\nRobot reached within the threshold of goal node ...!')
        print('\nCurrent node number for robot:', next_node_index)
        print('Location (x, y):', node_pt[next_node_index])
        print('Cost:', node_cost[next_node_index])
        break

    goal_flag = a_star(next_node_index)
    iterator += 1

# -----------------------------------------------Visited Node Exploration-----------------------------------------------
x_explore = []
y_explore = []

for i in range(len(iter_bin)):  # for each iteration
    for j in range(len(iter_bin[i])):  # for each action
        x_explore.append(iter_bin[i][j]['x'])
        y_explore.append(iter_bin[i][j]['y'])


# -----------------------------------------------Optimal solution trajectory--------------------------------------------
back_track = []
x_solution = []
y_solution = []


def traj(child):
    if child != start_index:
        back_track.append(child)
        parent = track[child]
        return traj(parent)

    else:
        back_track.append(start_index)
        return back_track[::-1]


trajectory = traj(visited_nodes_index[-1])

for i in range(1, len(trajectory)):
    child_node = trajectory[i]
    x_solution.append(path[child_node]['x'])
    y_solution.append(path[child_node]['y'])

# --------------------------------------- Visualization starts from here -----------------------------------------------
print('\n\n### Creating Visualization ###')
start_time_plot = time.time()

plt.style.use('seaborn-pastel')
fig = plt.figure()
ax = plt.axes(xlim=(0, 10), ylim=(0, 10))  # Defining Workspace limits
ax.set_aspect('equal')

# For Plotting Circle 1 with center at (2, 2) and radius 1 unit
x_circle1 = np.linspace(1, 3, 2000)
y_circle1a = 2 + (1 ** 2 - (x_circle1 - 2) ** 2) ** 0.5
y_circle1b = 2 - (1 ** 2 - (x_circle1 - 2) ** 2) ** 0.5
ax.plot(x_circle1, y_circle1a, 'b.', markersize=0.15)
ax.plot(x_circle1, y_circle1b, 'b.', markersize=0.15)

# For Plotting Circle 2 with center at (2, 8) and radius 1 unit
x_circle2 = np.linspace(1, 3, 2000)
y_circle2a = 8 + (1 ** 2 - (x_circle1 - 2) ** 2) ** 0.5
y_circle2b = 8 - (1 ** 2 - (x_circle1 - 2) ** 2) ** 0.5
ax.plot(x_circle2, y_circle2a, 'b.', markersize=0.15)
ax.plot(x_circle2, y_circle2b, 'b.', markersize=0.15)

# For Plotting Square with center at (1, 5) and dimension 1.5x1.5
x1, y1 = (0.25, 4.25)
x2, y2 = (0.25, 5.75)
x3, y3 = (1.75, 5.75)
x4, y4 = (1.75, 4.25)
ax.plot([x1, x2], [y1, y2], 'b-')
ax.plot([x2, x3], [y2, y3], 'b-')
ax.plot([x3, x4], [y3, y4], 'b-')
ax.plot([x4, x1], [y4, y1], 'b-')

# For Plotting Rectangle 1 with center at (5, 5) and dimension 2.5x1.5
x5, y5 = (3.75, 4.25)
x6, y6 = (3.75, 5.75)
x7, y7 = (6.25, 5.75)
x8, y8 = (6.25, 4.25)
ax.plot([x5, x6], [y5, y6], 'b-')
ax.plot([x6, x7], [y6, y7], 'b-')
ax.plot([x7, x8], [y7, y8], 'b-')
ax.plot([x8, x5], [y8, y5], 'b-')

# For Plotting Rectangle 2 with center at (8, 3) and dimension 1.5x2.0
x9, y9 = (7.25, 2.0)
x10, y10 = (7.25, 4.0)
x11, y11 = (8.75, 4.0)
x12, y12 = (8.75, 2.0)
ax.plot([x9, x10], [y9, y10], 'b-')
ax.plot([x10, x11], [y10, y11], 'b-')
ax.plot([x11, x12], [y11, y12], 'b-')
ax.plot([x12, x9], [y12, y9], 'b-')

# For Plotting Circle threshold for goal node
x_goal_circle = np.linspace(X_goal - goal_threshold_radius, X_goal + goal_threshold_radius, 2000)
y_goal_circle1 = Y_goal + (goal_threshold_radius ** 2 - (x_goal_circle - X_goal) ** 2) ** 0.5
y_goal_circle2 = Y_goal - (goal_threshold_radius ** 2 - (x_goal_circle - X_goal) ** 2) ** 0.5
ax.plot(x_goal_circle, y_goal_circle1, 'y.', markersize=20)
ax.plot(x_goal_circle, y_goal_circle2, 'y.', markersize=20)

new_node, = ax.plot([], [], 'g.')
solution_trajectory, = ax.plot([], [], 'r.')


def animate(frame_number):
    """
        In this function, animation is carried out.

        Parameters
        ----------
        frame_number : int type, here frame number serves as an index for the images

        Returns
        -------
        None
    """

    frame_diff = total_frames - frame_number

    if frame_diff > 51:  # will run for frame_number = [0, 148]
        first = 0
        last = step1 * (frame_number + 1)
        x = x_explore[first:last]
        y = y_explore[first:last]
        new_node.set_data(x, y)
        new_node.set_markersize(1)
        return new_node,

    elif frame_diff == 51:  # will run for frame_number = 149 only
        x = x_explore
        y = y_explore
        new_node.set_data(x, y)
        new_node.set_markersize(1)
        return new_node,

    elif 51 > frame_diff > 1:  # will run for frame_number = [150, 198]
        first = 0
        last = step2 * (frame_number - 149)
        x = x_solution[first:last]
        y = y_solution[first:last]
        solution_trajectory.set_data(x, y)
        solution_trajectory.set_markersize(1.5)
        return solution_trajectory,

    else:  # will run for frame_number = 199 only
        x = x_solution
        y = y_solution
        solution_trajectory.set_data(x, y)
        solution_trajectory.set_markersize(1.5)
        return solution_trajectory,


node_explore_frames = 150
solution_traj_frames = 50
total_frames = node_explore_frames + solution_traj_frames

step1 = int(len(x_explore) / node_explore_frames)
step2 = int(len(x_solution) / solution_traj_frames)

# animation = FuncAnimation(fig, animate, frames=total_frames, interval=30, blit=True, repeat=False)

# animation.save('Differential Drive Visualization (A-star) for test case (1 0.5 0).mp4', dpi=300)

# plt.close()

ax.plot(x_explore, y_explore, 'g.', markersize=1.0)
ax.plot(x_solution, y_solution, 'r.', markersize=1.0)
plt.show()

end_time_plot = time.time()
total_time_plot = (end_time_plot - start_time_plot)
print('\n\nTime taken for making visualization: {0:1.3f} s'.format(total_time_plot))

# --------------------------------------- Writing coordinates and velocities for Gazebo---------------------------------
print('\n\n\nWriting coordinates and velocities for Gazebo')
n_action = len(trajectory)
control_data = np.zeros((n_action, 4))

for control in range(0, n_action - 1):
    control_index = trajectory[control]
    x_old, y_old = node_pt[control_index]
    theta_old = node_angle[control_index]
    UL = path[trajectory[control + 1]]['UL']
    UR = path[trajectory[control + 1]]['UR']

    #shifting origin to Gazebo world coordinates
    x_new, y_new = (x_old - 5, y_old - 5)
    v_x1 = (0.5 * r_wheel) * (UL + UR) * math.cos(theta_old)
    v_x2 = (0.5 * r_wheel) * (UL + UR) * math.sin(theta_old)
    v_x = math.sqrt(v_x1**2 + v_x2**2)
    w_z = (r_wheel / L_wheel) * (UR - UL)

    control_data[control] = np.array([x_new, y_new, v_x, w_z])

control_data[-1][:2] = np.array(node_pt[trajectory[-1]])

np.savetxt('a_star_controls.txt', control_data)

print('\nCompleted !!!')
