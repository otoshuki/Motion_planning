#Robotics Club - MoPAT

#Import required libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib import colors

cmap = colors.ListedColormap(['Blue','red', 'black'])

#CV2 font for text
font = cv2.FONT_HERSHEY_COMPLEX

########################################################
#OpenCV Visualization

#Function to draw obstacle in white
def draw_obstacles(corner_obs, frame):
    if np.all(corner_obs != None):
        for i in range(len(corner_obs)):
            obs = np.int32(corner_obs[i][0])
            cv2.polylines(frame, [obs], True, (0,0,0), 5)
            cv2.fillConvexPoly(frame, obs, (0,0,0))

#Function to draw robot in red
def draw_robot(corner_robot, frame):
    corner_robot = np.int32(corner_robot[0])
    #Get center of robot
    center = [int(corner_robot[0][0]/2 + corner_robot[2][0]/2),
                int(corner_robot[0][1]/2 + corner_robot[2][1]/2)]
    front = [int(corner_robot[0][0]/2 + corner_robot[1][0]/2),
                int(corner_robot[0][1]/2 + corner_robot[1][1]/2)]
    #Fill with red
    cv2.fillConvexPoly(frame, corner_robot, (0,0,255))
    cv2.polylines(frame, [corner_robot], True, (0,0,255), 5)
    #Get angle wrt origin
    theta = np.arctan2((corner_robot[0][0]-corner_robot[3][0]),
                       (corner_robot[0][1]-corner_robot[3][1]))
    #Front extension
    extension = [front[0]+int(20*np.sin(theta)), front[1]+int(20*np.cos(theta))]
    #Draw line towards front face
    cv2.arrowedLine(frame, tuple(center), tuple(extension), (0,255,255), 6)
    return center, front, theta

#Function to draw connecting lines to nodes
def draw_nodes(frame, nodes):
    cv2.circle(frame, tuple(nodes[len(nodes)-1]), 3, (255,0,0), -1)
    for i in range(len(nodes)):
        cv2.putText(frame, chr(65+i), tuple(nodes[i]), font,
                    0.5, (255,0,0), 2, cv2.LINE_AA)
    cv2.polylines(frame, np.array([nodes]), False, (255,0,0), 2)

#Draw basic graph on mask
def draw_grid(x_segment, y_segment, mask):
    x_segment = x_segment
    y_segment = y_segment
    y_max, x_max, _ = np.shape(mask)
    for i in range(1, np.shape(x_segment)[0]-1):
        x_grid = x_segment[i]
        cv2.line(mask, (x_grid, 0), (x_grid, y_max), (0,0,0), 1)
    # print(x_segment)
    for i in range(1, np.shape(y_segment)[0]-1):
        y_grid = y_segment[i]
        cv2.line(mask, (0, y_grid), (x_max, y_grid), (0,0,0), 1)

########################################################
#Matplotlib Visualization

#Create grid for motion planning
def make_grid(x_segment, y_segment, r_center):
    x_axis = np.shape(x_segment)[0]
    y_axis = np.shape(y_segment)[0] - 2
    grid = np.ones([y_axis, x_axis])
    r_x = np.where(x_segment > r_center[0])[0][0] - 1
    r_y = np.where(y_segment > r_center[1])[0][0] - 1
    grid[r_y][r_x] = 0
    grid[r_y][r_x+1] = 2
    plt.pcolor(grid, cmap = cmap, edgecolors='k', linewidths=3)
    plt.gca().invert_yaxis()
    # plt.show()
