#Robotics Club - MoPAT
#Motion Planning Algorithm implementations

#Import required libraries
import numpy as np

#Create initial map
def init_map(x_seg, y_seg):
    map = np.zeros((np.shape(y_seg)[0]-2,np.shape(x_seg)[0]))
    return map

#Djakstra's algorithm for static environment
def djakstras(init_map, star_coor, dest_coor):
    #Set start coor

    #Set end coor

    #Initializer distance array
    dist_from_start = np.full(init_map.shape, np.inf)

#A* algorithm for static environment
