#Robotics Club - Motion Planning Set-Up

#Import required libaries
import numpy as np
import cv2
import cv2.aruco as aruco
import serial
import struct
import time

#CV2 font for text
font = cv2.FONT_HERSHEY_COMPLEX
#Global mouse position
clickX = 0
clickY = 0
change = False
#Node selection
click_node = False
move = False
letters = ['Center','A','B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
            'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
#Main function
def main():
    #Open the camera
    cap = cv2.VideoCapture(1)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters_create()
    #Set up serial port
    try:
        arduino = serial.Serial('/dev/ttyACM1', 9600)
        connected = 1
        print("Arduino connected")
        time.sleep(2)
    except:
        print("Arduino not connected")
        connected = 0
    nodes = []
    global click_node
    global move
    while(1):
        #Detect marker
        _, frame = cap.read()
        frame = np.rot90(frame, 2)
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
        mask = np.zeros(frame.shape)
        #Draw robot orientation and marker
        if robot_index != None:
            rcenter, rfront, rtheta = draw_robot(corners[robot_index], mask)
            corners = np.delete(corners, robot_index, 0)
        #Draw obstacles
        draw_obstacles(corners, mask)
        #Clickable node selection with 'C'
        k = cv2.waitKey(1)
        if k == ord('c'):
            if connected == 1:
                click_node = not click_node
                nodes = []
                nodes.append(rcenter)
                if click_node == False: move = not move
            else:
                print("Arduino not connected")
        if k == ord('w'):
            if click_node == True:
                move = not move
        if click_node == True:
            nodes[0] = rcenter
            nodes = click_nodes(nodes)
            if len(nodes) > 0:
                draw_nodes(mask, nodes)
                if len(nodes) > 1 and move == True:
                    err_dist, err_dir, err_angle = calc_error(nodes, rtheta)
                    if (err_dist < 30 and err_angle < 20): nodes.pop(1)
                    arduino.flush()
                    arduino.write((str(int(err_dist)) + ',' + err_dir + ',' + str(int(err_angle)) + '\n').encode())
        #cv2.imshow("Detection", detected)
        cv2.imshow("Mask", mask)
        #End work
        if k  == ord('q'): break
    cap.release()
    cv2.destroyAllWindows()

#Function to draw obstacle in white
def draw_obstacles(corner_obs, frame):
    if np.all(corner_obs != None):
        for i in range(len(corner_obs)):
            obs = np.int32(corner_obs[i][0])
            cv2.polylines(frame, [obs], True, (255,255,255), 5)
            cv2.fillConvexPoly(frame, obs, (255,255,255))

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
    theta = np.arctan2((corner_robot[0][0]-corner_robot[3][0]),(corner_robot[0][1]-corner_robot[3][1]))
    #Front extension
    extension = [front[0]+int(20*np.sin(theta)), front[1]+int(20*np.cos(theta))]
    #Draw line towards front face
    cv2.arrowedLine(frame, tuple(center), tuple(extension), (0,255,255), 6)
    return center, front, theta

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

#Function to draw connecting lines to nodes
def draw_nodes(frame, nodes):
    cv2.circle(frame, tuple(nodes[len(nodes)-1]), 3, (255,0,0), -1)
    for i in range(len(nodes)):
        cv2.putText(frame, letters[i], tuple(nodes[i]), font,
                    0.5, (255,0,0), 2, cv2.LINE_AA)
    cv2.polylines(frame, np.array([nodes]), False, (255,0,0), 2)

#Function to calculate current error
def calc_error(nodes, rtheta):
    #Dist between center and nearest node
    err_dist = np.sqrt((nodes[0][0] - nodes[1][0])**2 + (nodes[0][1] - nodes[1][1])**2)
    err_angle = rtheta - np.arctan2((nodes[1][0] - nodes[0][0]),(nodes[1][1] - nodes[0][1]))
    err_angle = err_angle*180/np.pi
    if (err_angle > 0 or (err_angle > -360 and err_angle < -180)): err_dir = 'r'
    else: err_dir ='l'
    if (err_angle > -360 and err_angle < -180): err_angle = 360 + err_angle
    #print(err_dir)
    return err_dist, err_dir, abs(err_angle)

if __name__ == '__main__':
    main()
