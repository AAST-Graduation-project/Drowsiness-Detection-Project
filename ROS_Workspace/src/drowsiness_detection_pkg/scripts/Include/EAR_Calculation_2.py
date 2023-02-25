import cv2
import mediapipe as mp
import numpy as np
from math import dist

# constant values
'''
N_Initial_Frames: Number of Frames to Calculate inital EAR (20 ... 40)
Holding_Frames: Number of Frames to wait when detecting Drowsiness ((20 ... 40) depends on FPS)  
Drowsiness_Factor: % of Initial EAR to be Compared with the Current EAR to detect Drowsiness (0.5 ... 0.8)
Ignored_Initial_EAR_Values: Any Value of EAR Less then That number will be Ignored (0.2 ... 0.1)
'''
N_Initial_Frames = 25
Holding_Frames = 20
Drowsiness_Factor = 0.7
Ignored_Initial_EAR_Values = 0.2

# global static variables 
class static:
    INITIAL_EAR = 0
    INITIAL_COUNTER=0
    D_COUNTER = 0
    SUM_EAR= 0
    STATE = "NO STATE"

# mediapipe face mesh detector
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.7, min_tracking_confidence=0.7,
                                  refine_landmarks=True, max_num_faces= 1, static_image_mode=False)

"""
name:   F_Get_Face
Input:  cv2_Frame 
job:    Adding State, Cuurnt Eye Aspect Ratio & Initail Eye Apect Ratio to the cv2_Frame   
output: cv2_Frame
"""
def F_Get_Face(C_Frame):
    # change encoding image from BGR to RGB
    RGB_Frame = cv2.cvtColor(C_Frame,cv2.COLOR_BGR2RGB)
    # improve speed
    RGB_Frame.flags.writeable= False
    # start detection
    results = face_mesh.process(RGB_Frame)
    # when the face is found
    if results.multi_face_landmarks:
        # get eyes coordinates in two lists from 486 landmarks 
        left_eye,right_eye = F_LandmarksDetection(C_Frame, results)
        # get eye aspect ratio 
        avg_EAR = F_EAR_Calc(left_eye,right_eye)
        # 
        if static.INITIAL_EAR == 0:
             F_Initial_EAR_Calc(avg_EAR)

        F_Get_State(avg_EAR, static.INITIAL_EAR)
        # put the values on the screen
        # state
        cv2.putText(C_Frame, ((str(static.STATE))), (10, 120),
                cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        # initial EAR
        cv2.putText(C_Frame, ("init_EAR="+(str(static.INITIAL_EAR))), (10, 90),
                cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        # current EAR
        cv2.putText(C_Frame, ("EAR="+(str(avg_EAR))), (10, 60),
                cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        
        # for drawing the coordinates of the eyes uncomment the following lines

        # cv2.circle(C_Frame, left_eye[8], 2, (0,255,0),-1)
        # cv2.circle(C_Frame, left_eye[0], 2, (0,255,0),-1)
        # cv2.circle(C_Frame, left_eye[4], 2, (0,255,0),-1)
        # cv2.circle(C_Frame, left_eye[12], 2, (0,255,0),-1)

        # cv2.circle(C_Frame, right_eye[8], 2, (0,255,0),-1)
        # cv2.circle(C_Frame, right_eye[0], 2, (0,255,0),-1)
        # cv2.circle(C_Frame, right_eye[4], 2, (0,255,0),-1)
        # cv2.circle(C_Frame, right_eye[12], 2, (0,255,0),-1)
    # return the frame    
    return C_Frame    


"""
name:   F_LandmarksDetection
Input:  Image & all coordinates 
job:    save the eyes coordinates in two list (left_coord[] & right_coord[])
output: left_coord[] & right_coord[]
"""
def F_LandmarksDetection(C_Frame, C_results):
    #left_eye = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
    #right_eye = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]

    img_hieght, img_wedth= C_Frame.shape[:2]
    left_coord = [(int(C_results.multi_face_landmarks[0].landmark[362].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[362].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[382].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[382].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[381].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[381].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[380].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[380].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[374].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[374].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[373].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[373].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[390].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[390].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[249].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[249].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[263].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[263].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[466].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[466].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[388].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[388].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[387].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[387].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[386].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[386].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[385].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[385].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[384].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[384].y*img_hieght)),
                  (int(C_results.multi_face_landmarks[0].landmark[398].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[398].y*img_hieght))]
    
    right_coord = [(int(C_results.multi_face_landmarks[0].landmark[33].x*img_wedth),  int(C_results.multi_face_landmarks[0].landmark[33].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[7].x*img_wedth),   int(C_results.multi_face_landmarks[0].landmark[7].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[163].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[163].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[144].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[144].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[145].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[145].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[153].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[153].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[154].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[154].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[155].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[155].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[133].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[133].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[173].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[173].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[157].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[157].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[158].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[158].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[159].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[159].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[160].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[160].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[161].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[161].y*img_hieght)),
                   (int(C_results.multi_face_landmarks[0].landmark[246].x*img_wedth), int(C_results.multi_face_landmarks[0].landmark[246].y*img_hieght))]
    

    # for facial_landmarks in C_results.multi_face_landmarks:
    #     for i in [362,  374,  263,  386]:
    #         point =facial_landmarks.landmark[i]
    #         left_coord.append((int(point.x*img_wedth), int(point.y*img_hieght)))
    #     for i in [33,  145, 133,  159]:
    #         point = facial_landmarks.landmark[i]
    #         right_coord.append((int(point.x*img_wedth), int(point.y*img_hieght)))
    
    return left_coord,  right_coord

"""
name:   F_EAR_Calc
Input:  Left_eye[(x, y)] & Right_eye[(x, y)]
job:    calculate Eye Aspect Ratio 
output: the average EAR of both eyes
"""
def F_EAR_Calc(C_left_eye, C_right_eye):
    left_EAR = dist(C_left_eye[12],C_left_eye[4])/dist(C_left_eye[0],C_left_eye[8])
    right_EAR = dist(C_right_eye[12],C_right_eye[4])/dist(C_right_eye[0],C_right_eye[8])
    AVG_EAR = round((right_EAR+left_EAR)/2.0, 2)
    return AVG_EAR

"""
name:   F_Initial_EAR_Calc
Input:  Current Aye Aspect Ratio
job:    Calculates the The Intial EAR from the current EAR  
output: None
"""
def F_Initial_EAR_Calc(C_avg_EAR):
    if static.INITIAL_COUNTER == N_Initial_Frames:
        static.INITIAL_EAR = round((static.SUM_EAR/N_Initial_Frames), 2)
        static.SUM_EAR=0
        static.INITIAL_COUNTER=0
    elif (static.INITIAL_COUNTER<N_Initial_Frames) and (C_avg_EAR > Ignored_Initial_EAR_Values):
        static.SUM_EAR += C_avg_EAR
        static.INITIAL_COUNTER += 1
        

"""
name:   F_Get_State
Input:  Current Eye Aspect Ratio & Initial Eye Aspect Ratio
job:    Compere The current EAR with Initial EAR  
output: None
"""
def F_Get_State(C_Current_EAR, C_INITIAL_EAR):

    if C_Current_EAR < (C_INITIAL_EAR*Drowsiness_Factor):
        static.D_COUNTER += 1
        if static.D_COUNTER > Holding_Frames:
            #static.flag = 1
            static.STATE = "sleep"
        
    elif C_Current_EAR >= (C_INITIAL_EAR*Drowsiness_Factor):
        static.D_COUNTER = 0
        #static.flag = 0
        static.STATE = "awake"
    
