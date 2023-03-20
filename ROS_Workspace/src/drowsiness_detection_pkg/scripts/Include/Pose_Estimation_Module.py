import cv2
import mediapipe as mp
import numpy as np
import time

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.7, min_tracking_confidence=0.7,
                                  max_num_faces= 1, static_image_mode=False)


cap = cv2.VideoCapture(0)
global Driver_Pose
class static:
    COUNTER = 0
    ESTIMATE_Drowsiness = 0

def F_Get_Face(C_Frame,C_result):
    img_h, img_w = C_Frame.shape[:2]
    face_3d =np.array([ (C_result.multi_face_landmarks[0].landmark[1].x * img_w, C_result.multi_face_landmarks[0].landmark[1].y * img_h, C_result.multi_face_landmarks[0].landmark[1].z),
                            (C_result.multi_face_landmarks[0].landmark[33].x * img_w, C_result.multi_face_landmarks[0].landmark[33].y * img_h, C_result.multi_face_landmarks[0].landmark[33].z),
                            (C_result.multi_face_landmarks[0].landmark[61].x * img_w, C_result.multi_face_landmarks[0].landmark[61].y * img_h, C_result.multi_face_landmarks[0].landmark[61].z),
                            (C_result.multi_face_landmarks[0].landmark[199].x * img_w, C_result.multi_face_landmarks[0].landmark[199].y * img_h, C_result.multi_face_landmarks[0].landmark[199].z),
                            (C_result.multi_face_landmarks[0].landmark[263].x * img_w, C_result.multi_face_landmarks[0].landmark[263].y * img_h, C_result.multi_face_landmarks[0].landmark[263].z),
                            (C_result.multi_face_landmarks[0].landmark[291].x * img_w, C_result.multi_face_landmarks[0].landmark[291].y * img_h, C_result.multi_face_landmarks[0].landmark[291].z)])
   
    face_2d =np.array([ (C_result.multi_face_landmarks[0].landmark[1].x * img_w, C_result.multi_face_landmarks[0].landmark[1].y * img_h),
                            (C_result.multi_face_landmarks[0].landmark[33].x * img_w, C_result.multi_face_landmarks[0].landmark[33].y * img_h),
                            (C_result.multi_face_landmarks[0].landmark[61].x * img_w, C_result.multi_face_landmarks[0].landmark[61].y * img_h),
                            (C_result.multi_face_landmarks[0].landmark[199].x * img_w, C_result.multi_face_landmarks[0].landmark[199].y * img_h),
                            (C_result.multi_face_landmarks[0].landmark[263].x * img_w, C_result.multi_face_landmarks[0].landmark[263].y * img_h),
                            (C_result.multi_face_landmarks[0].landmark[291].x * img_w, C_result.multi_face_landmarks[0].landmark[291].y * img_h)])
    return face_2d, face_3d

def F_Get_Angles(C_Frame, C_Face_2d, C_Face_3d):
    img_w, img_h = C_Frame.shape[:2]
        
    # The camera matrix
    cam_matrix = np.array([[img_w, 0, img_h / 2], [0, img_w, img_w / 2], [0, 0, 1]])

    # The distortion parameters
    dist_matrix = np.zeros((4, 1), dtype=np.float64)

    # Solve PnP
    success, rot_vec, trans_vec = cv2.solvePnP(C_Face_3d, C_Face_2d, cam_matrix, dist_matrix)

    # Get rotational matrix
    rmat, jac = cv2.Rodrigues(rot_vec)

    # Get angles
    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
    return angles

def F_Pose_State(C_Angles):
    # Get the y rotation degree
    x = C_Angles[0] * 360
    y = C_Angles[1] * -360

    # See where the user's head tilting
    if y < -15:
        text = "Looking Left"
        static.COUNTER += 1
    elif y > 15:
        #state = "Looking Right"
        static.COUNTER += 1
    elif x < -15:
        #state = "Looking Down"
        static.ESTIMATE_Drowsiness += 1
        static.COUNTER += 1
    elif x > 20:
        #state = "Looking Up"
        static.COUNTER += 1
    else:
        #state = "Forward"
        static.COUNTER = 0 
        static.ESTIMATE_Drowsiness = 0

    if static.COUNTER >= 40:
        if static.ESTIMATE_Drowsiness >= 40:
            state = "Suspected Drowsiness"
        else:
            state = "NOT FOCUSED"
    else:
        state = "FOCUSED"   
    return state    
        

def F_Get_Pose(Frame):
    global Driver_Pose
    Driver_Pose = "No Driver"
    start = time.time()

    # convert the color space from BGR to RGB
    RGB_image = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)

    # To improve performance
    RGB_image.flags.writeable = False

    # Get the result
    results = face_mesh.process(RGB_image)

    if results.multi_face_landmarks:
        
        Face_2d, Face_3d = F_Get_Face(Frame,results)
       
        Angles = F_Get_Angles(Frame, Face_2d, Face_3d)

        Driver_Pose = F_Pose_State(Angles)
         # Add the text on the image
        cv2.putText(Frame, Driver_Pose, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)    

        end = time.time()
        totalTime = end - start

        fps = 1.0 / totalTime

        cv2.putText(Frame, f'FPS: {int(fps)}', (20, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        
    return Frame , Driver_Pose
   


