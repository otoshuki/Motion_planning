#Robotics Club - MoPAT

#Import required libaries
import numpy as np
import cv2
import cv2.aruco as aruco
import struct
import time
import draw_tools

#CV2 font for text
font = cv2.FONT_HERSHEY_COMPLEX

#Global mouse position
clickX = 0
clickY = 0
change = False
#Node selection
click_node = False
move = False

#Main function
def main():
    #Open the camera
    # cap = cv2.VideoCapture(1)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters_create()
    nodes = []
    global click_node
    global move
    f = open('data.txt', 'w+')
    while(1):
        #Detect marker
        # _, frame = cap.read()
        frame = cv2.imread('frame.png')
        # frame = np.rot90(frame, 2)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict,
                                                    parameters = params)
        detected = aruco.drawDetectedMarkers(frame, corners)
        robot_index = None
        if np.all(ids != None):
            #print('Detected :',corners[0][0])
            for i in range(len(ids)):
                cv2.putText(detected, str(ids[i][0]),
                                    tuple(corners[i][0][2]),
                                    font, 0.5, (0, 0, 255), 1, 4)
                #0th marker as robot location
                if ids[i][0] == 0: robot_index = i
        #Create mask
        mask = np.ones(frame.shape)*255
        #Draw robot orientation and marker
        if robot_index != None:
            robot_corners = corners[robot_index]
            rcenter, rfront, rtheta = draw_tools.draw_robot(robot_corners, mask)
            corners = np.delete(corners, robot_index, 0)
        #Draw obstacles
        draw_tools.draw_obstacles(corners, mask)
        #Clickable node selection with 'C'
        k = cv2.waitKey(1)
        if k == ord('c'):
            click_node = not click_node
            nodes = []
            nodes.append(rcenter)
        if k == ord('w'):
            if click_node == True:
                move = not move
        if click_node == True:
            nodes[0] = rcenter
            nodes = click_nodes(nodes)
            if len(nodes) > 0:
                draw_tools.draw_nodes(mask, nodes)
                if len(nodes) > 1 and move == True:
                    err_dist, err_dir, err_angle = calc_error(nodes, rtheta)
                    if (err_dist < 30 and err_angle < 20): nodes.pop(1)
                    f.seek(0)
                    f.write(str(int(err_dist)) + '+' + str(int(err_angle)) + '-' + err_dir + '*')
                    f.truncate()
                else:
                    f.truncate(0)
        else:
            f.truncate(0)
            move = False
        #Create initial map
        x_seg, y_seg = create_map(mask, robot_corners)    
        draw_tools.draw_grid(x_seg, y_seg, mask)
        draw_tools.make_grid(x_seg, y_seg, rcenter)
        cv2.imshow("Detection", frame)
        cv2.imshow("Mask", mask)
        #End work
        if k  == ord('q'):
            f.truncate(0)
            break
    # cap.release()
    cv2.destroyAllWindows()

#Function to find mouse click location
def get_location(event, x, y, flags, param):
    global clickX, clickY, change
    #If left clicked, set click location
    if event == cv2.EVENT_LBUTTONDOWN:
        clickX = x
        clickY = y
        change = True

#Function to select clickable nodes
def click_nodes(nodes):
    #Global variable to detect mouse click
    global change
    #Node indexing
    #Mouse click detection
    cv2.setMouseCallback("Mask", get_location)
    #Clickable path selection
    if change == True:
        change = not change
        loc = [clickX, clickY]
        nodes.append(loc)
    return nodes

#Function to calculate current error
def calc_error(nodes, rtheta):
    err_dist = np.linalg.norm(np.array(nodes[0])-np.array(nodes[1]))
    err_angle = rtheta - np.arctan2((nodes[1][0] - nodes[0][0]),(nodes[1][1] - nodes[0][1]))
    err_angle = err_angle*180/np.pi
    if (err_angle > 0 or (err_angle > -360 and err_angle < -180)): err_dir = 'r'
    else: err_dir ='l'
    if (err_angle > -360 and err_angle < -180): err_angle = 360 + err_angle
    #print(err_dir)
    return err_dist, err_dir, abs(err_angle)

#Basic graph for mask guides
def create_map(mask, corners):
    width = np.linalg.norm(corners[0][0]-corners[0][1])+10
    x_segment = np.arange(0, np.shape(mask)[1], width)
    y_segment = np.arange(0, np.shape(mask)[0], width)
    x_segment = np.append(x_segment, np.shape(mask)[1])
    y_segment = np.append(y_segment, np.shape(mask)[0])
    return x_segment.astype(int), y_segment.astype(int)

if __name__ == '__main__':
    main()
