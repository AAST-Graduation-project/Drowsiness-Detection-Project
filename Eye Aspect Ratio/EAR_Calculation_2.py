import cv2
import mediapipe as mp
import numpy as np
from scipy.spatial.distance import euclidean as dist
import time

# constant values
'''
N_Initial_Frames: Number of Frames to Calculate initial EAR (20 ... 40)
Holding_Frames: Number of Frames to wait when detecting Drowsiness ((20 ... 40) depends on FPS)  
Drowsiness_Factor: % of Initial EAR to be Compared with the Current EAR to detect Drowsiness (0.5 ... 0.8)
Ignored_Initial_EAR_Values: Any Value of EAR Less then That number will be Ignored (0.2 ... 0.1)
'''
N_Initial_Frames = 25
Holding_Frames = 20
Drowsiness_Factor = 0.75
Ignored_Initial_EAR_Values = 0.2

# Global Static Variables


class static:
    INITIAL_EAR = 0
    INITIAL_COUNTER = 0
    D_COUNTER = 0
    SUM_EAR = 0
    STATE = "NO STATE"


# mediapipe face mesh detector
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.7, min_tracking_confidence=0.7,
                                  refine_landmarks=True, max_num_faces=1, static_image_mode=False)

"""
name:   F_Get_Face
Input:  cv2_Frame 
job:    Adding State, Currant Eye Aspect Ratio & Initial Eye Aspect Ratio to the cv2_Frame   
output: cv2_Frame
"""


def F_Get_Face(C_Frame):
    # Change Encoding Frame from BGR to RGB
    RGB_Frame = cv2.cvtColor(C_Frame, cv2.COLOR_BGR2RGB)

    # Improve Speed
    RGB_Frame.flags.writeable = False

    # start detection
    results = face_mesh.process(RGB_Frame)

    # When the Face is Found
    if results.multi_face_landmarks:
        # get eyes coordinates in two lists from 486 landmarks
        left_eye, right_eye = F_LandmarksDetection(C_Frame, results)

        # get eye aspect ratio
        avg_EAR = F_EAR_Calc(left_eye, right_eye)

        # Calculate Initial EAR at the Start Only
        if static.INITIAL_EAR == 0:
            F_Initial_EAR_Calc(avg_EAR)

        # Find the Driver's State
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

    # Frame Dimensions
    img_hight, img_width = C_Frame.shape[:2]

    # The (468) Landmarks of the Face
    Landmark = C_results.multi_face_landmarks[0].landmark

    # Left Eye Coordinates
    left_coord = np.array([(int(Landmark[i].x * img_width), int(Landmark[i].y * img_hight))
                          for i in [362,  374,  263,  386]])

    # Right Eye Coordinates
    right_coord = np.array([(int(Landmark[i].x * img_width), int(Landmark[i].y * img_hight))
                           for i in [33,  145,  133, 159]])

    # Returned Coordinates
    return left_coord,  right_coord


"""
name:   F_EAR_Calc
Input:  Left_eye[(x, y)] & Right_eye[(x, y)]
job:    calculate Eye Aspect Ratio 
output: the average EAR of both eyes
"""


def F_EAR_Calc(C_left_eye, C_right_eye):
    # Calculate the Eye Width for Both Eyes
    left_eye_width = dist(C_left_eye[0], C_left_eye[2])
    right_eye_width = dist(C_right_eye[0], C_right_eye[2])

    # Eye Aspect Ratio = Eye Height/Eye Width
    left_EAR = dist(C_left_eye[3], C_left_eye[1]) / left_eye_width
    right_EAR = dist(C_right_eye[3], C_right_eye[1]) / right_eye_width

    # Calculate the Average EAR
    AVG_EAR = (right_EAR + left_EAR) / 2.0

    # Return the Average
    return AVG_EAR


"""
name:   F_Initial_EAR_Calc
Input:  Current Aye Aspect Ratio
job:    Calculates the The Initial EAR from the current EAR  
output: None
"""


def F_Initial_EAR_Calc(C_avg_EAR):
    # When the Counter is reached the Threshold
    if static.INITIAL_COUNTER == N_Initial_Frames:
        # Calculate the Average Initial EAR from the Sum of the Current EAR
        static.INITIAL_EAR = round((static.SUM_EAR/N_Initial_Frames), 2)
        # Clear the Sum
        static.SUM_EAR = 0
        # Reset the Counter
        static.INITIAL_COUNTER = 0

    # When the Counter is Less Than the Threshold with Ignoring the Blinking Frames
    elif (static.INITIAL_COUNTER < N_Initial_Frames) and (C_avg_EAR > Ignored_Initial_EAR_Values):
        # Add the Current EAR Value to the Sum
        static.SUM_EAR += C_avg_EAR
        # Increment the Counter
        static.INITIAL_COUNTER += 1


"""
name:   F_Get_State
Input:  Current Eye Aspect Ratio & Initial Eye Aspect Ratio
job:    Compere The Current EAR with Initial EAR  
output: None
"""


def F_Get_State(C_Current_EAR, C_INITIAL_EAR):
    # When the Eye is About to be Closed
    if C_Current_EAR < (C_INITIAL_EAR*Drowsiness_Factor):
        # Increment the Drowsiness Counter
        static.D_COUNTER += 1
        # When the Drowsiness Counter Reaches the Threshold (Not Blinking)
        if static.D_COUNTER > Holding_Frames:
            # The Driver is Drowsy
            static.STATE = "Drowsy"

    # When the Eye is Opened
    elif C_Current_EAR >= (C_INITIAL_EAR*Drowsiness_Factor):
        # Reset the Drowsiness Counter
        static.D_COUNTER = 0
        # The Driver is Awake
        static.STATE = "awake"


cap = cv2.VideoCapture(0)

while True:

    _, Frame = cap.read()
    start = time.time()
    New = F_Get_Face(Frame)
    End = time.time()-start
    End = 1/End
    print(f"{End: .2f}")
    cv2.imshow("Frame", New)
    if cv2.waitKey(1) == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
