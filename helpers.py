from robot_class import robot
from math import *
import random
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


# --------
# this helper function displays the world that a robot is in
# it assumes the world is a square grid of some given size
# and that landmarks is a list of landmark positions(an optional argument)
def display_world(world_size, position, landmarks=None):
    
    # using seaborn, set background grid to gray
    sns.set_style("dark")

    # Plot grid of values
    world_grid = np.zeros((world_size+1, world_size+1))

    # Set minor axes in between the labels
    ax=plt.gca()
    cols = world_size+1
    rows = world_size+1

    ax.set_xticks([x for x in range(1,cols)],minor=True )
    ax.set_yticks([y for y in range(1,rows)],minor=True)
    
    # Plot grid on minor axes in gray (width = 1)
    plt.grid(which='minor',ls='-',lw=1, color='white')
    
    # Plot grid on major axes in larger width
    plt.grid(which='major',ls='-',lw=2, color='white')
    
    # Create an 'o' character that represents the robot
    # ha = horizontal alignment, va = vertical
    ax.text(position[0], position[1], 'o', ha='center', va='center', color='r', fontsize=30)
    
    # Draw landmarks if they exists
    if(landmarks is not None):
        # loop through all path indices and draw a dot (unless it's at the car's location)
        for pos in landmarks:
            if(pos != position):
                ax.text(pos[0], pos[1], 'x', ha='center', va='center', color='purple', fontsize=20)
    
    # Display final result
    plt.show()

    
# --------
# this routine makes the robot data
# the data is a list of measurements and movements: [measurements, [dx, dy]]
# collected over a specified number of time steps, N
# 先sense、再move，因此：
#       t0            t1            t2            ...        tN-1
#      sense  move   sense  move   sense   move         move         (注：tN-1时没有sense和move了)
def make_data(N, num_landmarks, world_size, measurement_range, motion_noise, 
              measurement_noise, distance):

    # check that data has been made 这里是为了确认robot的sense()方法已经实现了
    try:
        check_for_data(num_landmarks, world_size, measurement_range, motion_noise, measurement_noise)
    except ValueError:
        print('Error: You must implement the sense function in robot_class.py.')
        return []
    
    complete = False
    
    r = robot(world_size, measurement_range, motion_noise, measurement_noise) #实例化一个robor，初始位置为world的中心
    r.make_landmarks(num_landmarks)#在world中随机建立num_landmarks个landmark，每个landmark的[x，y]都是整数(注：机器人坐标可为小数)，存储在r.landmarks中

    while not complete:

        data = []

        seen = [False for row in range(num_landmarks)] #先初始化seen为含num_landmarks个False的列表
    
        # guess an initial motion
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance
            
        for k in range(N-1): #因为tN-1时不sense和move了，tN-1是最后一步有sense和move的，所以这里N要减掉1
    
            # collect sensor measurements in a list, Z (注：都是在感知范围之内的landmark，而感知范围之外的landmark不会出现在Z中)
            Z = r.sense() #Z为[index, dx, dy]的列表，index为landmark在r.landmarks中的下标，dx、dy为该landmark与当前机器人所在位置x、y方向的距离。

            # check off all landmarks that were observed 
            for i in range(len(Z)):
                seen[Z[i][0]] = True #被感知的landmark在seen中对应设为True，剩下的num_landmarks-len(Z)个landmark对应位置依然保持False
    
            # move 这里while循环的作用是：如果机器人移动超出了world，就重来一次；如果机器人移动没超出world，就直接跳出while循环。总而言之，就是
            #                    保证机器人做且仅做一次有效的移动。
            while not r.move(dx, dy): #r.move(dx, dy)：当机器人移动dx、dy不超出world时，更新坐标，并返回True；当超出时，不更新坐标，且返回False。
                # if we'd be leaving the robot world, pick instead a new direction
                orientation = random.random() * 2.0 * pi
                dx = cos(orientation) * distance
                dy = sin(orientation) * distance

            # collect/memorize all sensor and motion data
            data.append([Z, [dx, dy]])

        # we are done when all landmarks were observed; otherwise re-run
        complete = (sum(seen) == num_landmarks) #在N个时间步中完成观测到所有的landmark才算完成任务，否则重新一遍N个时间步。

    print(' ')
    print('Landmarks: ', r.landmarks)
    print(r)


    return data #data中总共有N-1个[Z, [dx, dy]]


def check_for_data(num_landmarks, world_size, measurement_range, motion_noise, measurement_noise):
    # make robot and landmarks
    r = robot(world_size, measurement_range, motion_noise, measurement_noise)
    r.make_landmarks(num_landmarks)
    
    
    # check that sense has been implemented/data has been made
    test_Z = r.sense()
    if(test_Z is None):
        raise ValueError
