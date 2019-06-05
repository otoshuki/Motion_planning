#Robotics Club - Motion Planning Set-Up

#Import required libaries
import numpy as np
import cv2
import cv2.aruco as aruco

#CV2 font for text
font = cv2.FONT_HERSHEY_COMPLEX

#Main function
def main():
    #Open the camera
    cap = cv2.VideoCapture(1)
    while(1):
        #Detect marker
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        params = aruco.DetectorParameters_create()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict,
                                                    parameters = params)
        detected = aruco.drawDetectedMarkers(frame, corners)
        if np.all(ids != None):
            #print('Detected :',corners[0][0])
            for i in range(len(ids)):
                cv2.putText(detected, str(ids[i][0]),
                                    tuple(corners[i][0][2]),
                                    font, 0.5, (0, 0, 255), 1, 4)
                #0th marker as robot location
                if ids[i][0] == 0: robot_index = i
        #Draw robot orientation and marker
        rcenter, rfront, rtheta = draw_robot(corners[robot_index][0], detected)
        cv2.draw_obstacles
        if cv2.waitKey(1) == ord('q'):
            break
        cv2.imshow('Detection', detected)
    cap.release()
    cv2.destroyAllWindows()

def draw_obstacles(corner_obs, id, frame):
    corner_obs = corner_obs.reshape((-1,1,2))
    cv2.polylines(frame, [pts], True, (255,255,255))

def draw_robot(corner_robot, frame):
    #Get center of robot
    center = [int(corner_robot[0][0]/2 + corner_robot[2][0]/2),
                int(corner_robot[0][1]/2 + corner_robot[2][1]/2)]
    front = [int(corner_robot[0][0]/2 + corner_robot[1][0]/2),
                int(corner_robot[0][1]/2 + corner_robot[1][1]/2)]
    cv2.circle(frame, tuple(center), 2, (0,255,255), -1)
    #Get angle wrt origin
    theta = np.arctan((corner_robot[0][0]-corner_robot[3][0])/(corner_robot[0][1]-corner_robot[3][1]))
    #Front extension
    extension = [front[0]+int(20*np.sin(theta)), front[1]+int(20*np.cos(theta))]
    #Draw line towards front face
    cv2.line(frame, tuple(center), tuple(extension), (255,0,0), 2)
    return center, front, theta

if __name__ == '__main__':
    main()
