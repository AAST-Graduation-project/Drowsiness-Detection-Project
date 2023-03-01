from numpy import array
from math import dist
from cv2 import (COLOR_BGR2GRAY, FONT_HERSHEY_COMPLEX, cvtColor, VideoCapture, resize, INTER_CUBIC,
                 destroyAllWindows, imshow, polylines, putText, waitKey)
from dlib import get_frontal_face_detector, shape_predictor

# import the file.dat 
dlib_face = shape_predictor("/home/mario/Drowsiness-Detection-Project/ROS_Workspace/src/drowsiness_detection_pkg/scripts/Lib/shape_predictor_68_face_landmarks.dat")
# face detector model
face_detector = get_frontal_face_detector()

#global constant values
Drowsiness_Factor = 0.7
Holding_Frames = 20
Ignored_Initial_EAR_Values = 0.1
N_Initial_Frames = 20

# global initial values
class static:
    flag=0
    initial_EAR=0.0
    state= "NO state"
    initial_EAR_counter=0
    sum_EAR=0.0
    counter=0


"""
Function_name: Get_Initial_EAR
Input: current EAR & Intia
job: takes the average of the curren EAR 
output: Initiail EAR
"""
def Get_Initial_EAR(C_EAR):
    
    if static.initial_EAR_counter == N_Initial_Frames:
        static.initial_EAR = round((static.sum_EAR/N_Initial_Frames), 2)
        static.sum_EAR=0
        static.initial_flag=0
    elif (static.initial_EAR_counter<N_Initial_Frames) and (C_EAR > Ignored_Initial_EAR_Values):
        static.sum_EAR += C_EAR
        static.initial_EAR_counter += 1
        
    return static.initial_EAR

"""
Function_name: Get_Eye
Input: all 68 landmark coordinates 
job: determine the eye coordinates (x,y) & save it in array
output: array of left & right eye coordinates
"""
def Get_Eye(face_landmark_part):
    right_eye = array([(face_landmark_part(36).x, face_landmark_part(36).y),
                        (face_landmark_part(37).x, face_landmark_part(37).y),
                        (face_landmark_part(38).x, face_landmark_part(38).y),
                        (face_landmark_part(39).x, face_landmark_part(39).y),
                        (face_landmark_part(40).x, face_landmark_part(40).y),
                        (face_landmark_part(41).x, face_landmark_part(41).y)])
    left_eye = array([(face_landmark_part(42).x, face_landmark_part(42).y),
                        (face_landmark_part(43).x, face_landmark_part(43).y),
                        (face_landmark_part(44).x, face_landmark_part(44).y),
                        (face_landmark_part(45).x, face_landmark_part(45).y),
                        (face_landmark_part(46).x, face_landmark_part(46).y),
                        (face_landmark_part(47).x, face_landmark_part(47).y)])

    return right_eye, left_eye


"""
Function_name: EAR_Calc
Input: array of six coordinates of the Eye 
job: calculate the Eye Aspect Ratio
output: Eye Aspect Ratio For a single Eye
"""
def EAR_Calc(eye):
    A = dist((eye[1]), (eye[5]))
    B = dist((eye[2]), (eye[4]))
    C = dist((eye[0]), (eye[3]))
    EAR = (A+B)/(2.0*C)
    return EAR


"""
Function_name: Get_Eye_State
Input: Current_EAR & initial_EAR 
job: compare Current_EAR with initial_EAR
output: Eye_state
"""
def Get_Eye_State(C_Current_EAR, C_initial_EAR):
    
    
    if C_Current_EAR < (C_initial_EAR*Drowsiness_Factor):
        static.counter+=1
        if static.counter > Holding_Frames:
            static.flag = 1
            static.state = "sleep"
        
    elif C_Current_EAR >= (C_initial_EAR*Drowsiness_Factor) :
        static. counter = 0
        static.flag = 0
        static.state = "awake"
    return static.state


"""
Function_name: Get_Face
Input: Image 
job: main function to get the EAR and put it on the Image
output: EAR_Image
"""
def Get_Face(frame):
    #change Colored image to gray scale
    gray = cvtColor(frame, COLOR_BGR2GRAY)
    #detect the Driver's face
    faces = face_detector(gray)
    
    #loop for each face detect
    for face in faces:
        face_landmark = dlib_face(gray, face)

        right_eye, left_eye = Get_Eye(face_landmark.part)

        #Right eye aspect ratio
        right_EAR = EAR_Calc(right_eye) 
        #Left eye aspect ratio
        left_EAR = EAR_Calc(left_eye)

        #average of Left & Right EAR
        avg_EAR = round((left_EAR+right_EAR)/2.0, 2) 
        
        # polylines(frame, [right_eye], 1, (0, 255, 0), 1) 
        # polylines(frame, [left_eye], 1, (0, 255, 0), 1)

        #Initial EAR for first run only
        if static.initial_EAR == 0:
            static.initial_EAR =Get_Initial_EAR(avg_EAR)

        #state of the Driver based on EAR
        static.state=Get_Eye_State(avg_EAR, static.initial_EAR)

        #print state of eye on screen
        putText(frame, str(static.state), (0, 75), 
            FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        #print current EAR on screen
        putText(frame, ("EAR="+(str(avg_EAR))), (0, 15),
            FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
    return frame
    
    
    
    #7amoooo
