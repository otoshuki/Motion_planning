#Robotics Club - IITG

import numpy as np
import cv2
import cv2.aruco as aruco

font = cv2.FONT_HERSHEY_COMPLEX

def detect_marker(show):
    '''
    Funtion to perform marker detection
    Input   : show - 0  : return values without display
                   - 1  : return values after displaying
    Return  : corners   : numpy array of corners of markers
              ids       : ids of markers
              rejected  : corners of rejected points
    '''
    cap = cv2.VideoCapture(1)
    while(1):
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        #Get the params for the dictionary
        params = aruco.DetectorParameters_create()
        #Dectection
        #detectMarkers(inp, dict, corners, ids, params, rejetcs)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict,
                                                    parameters = params)
        if show == 0:
            cap.release()
            return corners, ids, rejected, frame
        detected = aruco.drawDetectedMarkers(frame, corners)
        if np.all(ids != None):
            #print('Detected :',len(ids))
            for i in range(len(ids)):
                cv2.putText(detected, str(ids[i][0]),
                                    tuple(corners[i][0][2]),
                                    font, 0.5, (0, 0, 255), 1, 4)
        cv2.imshow('Detection', detected)
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    return corners, ids, rejected

if __name__ == '__main__':
    detect_marker(1)
