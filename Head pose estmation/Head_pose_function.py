import cv2
import mediapipe as mp
import numpy as np


#mediapipe face mesh detector
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.7, min_tracking_confidence=0.7,
                                  max_num_faces=1, static_image_mode=False)
mp_drawing = mp.solutions.drawing_utils


##########################

def GetNose(image):
    image = cv2.resize(image, None, fx=0.5, fy=0.5)
    # Flip the image horizontally for a later selfie-view display
    image = cv2.flip(image, 1)
    # change encoding image from BGR to RGB
    RGB_Frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # improve speed
    RGB_Frame.flags.writeable = False
    # start detection
    results = face_mesh.process(RGB_Frame)
    img_h, img_w, img_c = image.shape
    # when the face is found
    if results.multi_face_landmarks:
        #Get the 2D coordinates
        face_2d = np.array([((results.multi_face_landmarks[0].landmark[1].x * img_w),
                         (results.multi_face_landmarks[0].landmark[1].y * img_h)),
                        ((results.multi_face_landmarks[0].landmark[33].x * img_w),
                         (results.multi_face_landmarks[0].landmark[33].y * img_h)),
                        ((results.multi_face_landmarks[0].landmark[61].x * img_w),
                         (results.multi_face_landmarks[0].landmark[61].y * img_h)),
                        ((results.multi_face_landmarks[0].landmark[199].x * img_w),
                         (results.multi_face_landmarks[0].landmark[199].y * img_h)),
                        ((results.multi_face_landmarks[0].landmark[263].x * img_w),
                         (results.multi_face_landmarks[0].landmark[263].y * img_h)),
                        ((results.multi_face_landmarks[0].landmark[291].x * img_w),
                         (results.multi_face_landmarks[0].landmark[291].y * img_h))])

        # Get the 3D Coordinates

        face_3d = np.array([((results.multi_face_landmarks[0].landmark[1].x * img_w),
                         (results.multi_face_landmarks[0].landmark[1].y * img_h),
                         (results.multi_face_landmarks[0].landmark[1].z)),
                        ((results.multi_face_landmarks[0].landmark[33].x * img_w),
                         (results.multi_face_landmarks[0].landmark[33].y * img_h),
                         (results.multi_face_landmarks[0].landmark[33].z)),
                        ((results.multi_face_landmarks[0].landmark[61].x * img_w),
                         (results.multi_face_landmarks[0].landmark[61].y * img_h),
                         (results.multi_face_landmarks[0].landmark[61].z)),
                        ((results.multi_face_landmarks[0].landmark[199].x * img_w),
                         (results.multi_face_landmarks[0].landmark[199].y * img_h),
                         (results.multi_face_landmarks[0].landmark[199].z)),
                        ((results.multi_face_landmarks[0].landmark[263].x * img_w),
                         (results.multi_face_landmarks[0].landmark[263].y * img_h),
                         (results.multi_face_landmarks[0].landmark[263].z)),
                        ((results.multi_face_landmarks[0].landmark[291].x * img_w),
                         (results.multi_face_landmarks[0].landmark[291].y * img_h),
                         (results.multi_face_landmarks[0].landmark[291].z))])
        # Get the nose 2D
        nose_2d = face_2d[0]

    return face_2d,face_3d,nose_2d

def GetStateOfNose(image,face_2d,face_3d):
    img_h, img_w, img_c = image.shape
    # The camera matrix
    cam_matrix = np.array([[img_w, 0, img_h / 2],
                           [0, img_w, img_w / 2],
                           [0, 0, 1]])

    # The distortion parameters
    dist_matrix = np.zeros((4, 1), dtype=np.float64)

    # Solve PnP
    success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

    # Get rotational matrix
    rmat, jac = cv2.Rodrigues(rot_vec)

    # Get angles
    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

    # Get the y rotation degree
    x = angles[0] * 360
    y = angles[1] * 360
    z = angles[2] * 360

    # See where the user's head tilting
    if y < -10:
        text = "Looking Left"
    elif y > 10:
        text = "Looking Right"
    elif x < -10:
        text = "Looking Down"
    elif x > 10:
        text = "Looking Up"
    else:
        text = "Forward"

    return x,y,text



def displayNose(nose_2d,image,x,y,text) :
    # Display the nose direction
    #p1 = (int(nose_2d[0]), int(nose_2d[1]))
    #p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))
    #cv2.line(image, p1, p2, (0, 0, 0), 3)

    #display the image
    cv2.imshow('Head Pose Estimation', image)
    #display the text
    cv2.putText(image, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)



