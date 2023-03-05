import cv2
import mediapipe as mp
import numpy as np
import time

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.7, min_tracking_confidence=0.7,
                                   max_num_faces= 1, static_image_mode=False)
mp_drawing = mp.solutions.drawing_utils

#drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    image= cv2.resize(image,None,fx=0.5,fy=0.5)
    start = time.time()

    # Flip the image horizontally for a later selfie-view display
    # Also convert the color space from BGR to RGB
    image = cv2.flip(image, 1)
    rgp_image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

    # To improve performance
    rgp_image.flags.writeable = False

    # Get the result
    results = face_mesh.process(rgp_image)

    img_h, img_w, img_c = image.shape
    face_3d = []
    face_2d = []
    
    if results.multi_face_landmarks:
           
        # Get the 2D coordinates
        
        face_2d = np.array([((results.multi_face_landmarks[0].landmark[1].x*img_w), (results.multi_face_landmarks[0].landmark[1].y*img_h)),
                    ((results.multi_face_landmarks[0].landmark[33].x*img_w), (results.multi_face_landmarks[0].landmark[33].y*img_h)),
                    ((results.multi_face_landmarks[0].landmark[61].x*img_w), (results.multi_face_landmarks[0].landmark[61].y*img_h)),
                    ((results.multi_face_landmarks[0].landmark[199].x*img_w), (results.multi_face_landmarks[0].landmark[199].y*img_h)),
                    ((results.multi_face_landmarks[0].landmark[263].x*img_w), (results.multi_face_landmarks[0].landmark[263].y*img_h)),
                    ((results.multi_face_landmarks[0].landmark[291].x*img_w), (results.multi_face_landmarks[0].landmark[291].y*img_h))])
                
        # Get the 3D Coordinates
    
        face_3d =np.array([((results.multi_face_landmarks[0].landmark[1].x*img_w), (results.multi_face_landmarks[0].landmark[1].y*img_h), (results.multi_face_landmarks[0].landmark[1].z)),
                    ((results.multi_face_landmarks[0].landmark[33].x*img_w), (results.multi_face_landmarks[0].landmark[33].y*img_h), (results.multi_face_landmarks[0].landmark[33].z)),
                    ((results.multi_face_landmarks[0].landmark[61].x*img_w), (results.multi_face_landmarks[0].landmark[61].y*img_h), (results.multi_face_landmarks[0].landmark[61].z)),
                    ((results.multi_face_landmarks[0].landmark[199].x*img_w), (results.multi_face_landmarks[0].landmark[199].y*img_h), (results.multi_face_landmarks[0].landmark[199].z)),
                    ((results.multi_face_landmarks[0].landmark[263].x*img_w), (results.multi_face_landmarks[0].landmark[263].y*img_h), (results.multi_face_landmarks[0].landmark[263].z)),
                    ((results.multi_face_landmarks[0].landmark[291].x*img_w), (results.multi_face_landmarks[0].landmark[291].y*img_h), (results.multi_face_landmarks[0].landmark[291].z))])
         # Get the nose 2D
        nose_2d =face_2d[0]
        # Get the nose 3D
        nose_3d =np.array( [((results.multi_face_landmarks[0].landmark[1].x*img_w), (results.multi_face_landmarks[0].landmark[1].y*img_h), (results.multi_face_landmarks[0].landmark[1].z*3000))])


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

        # Display the nose direction
        #nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)

       # p1 = (int(nose_2d[0]), int(nose_2d[1]))
        #p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))

       # cv2.line(image, p1, p2, (255, 0, 0), 3)

        # Add the text on the image
        cv2.putText(image, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        #cv2.putText(image, "x: " + str(np.round(x, 2)), (300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        #cv2.putText(image, "y: " + str(np.round(y, 2)), (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        #cv2.putText(image, "z: " + str(np.round(z, 2)), (300, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        end = time.time()
        totalTime = end - start

        fps = 1 / totalTime
        # print("FPS: ", fps)

        cv2.putText(image, f'FPS: {int(fps)}', (20, 230), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

       

    cv2.imshow('Head Pose Estimation', image)

    if cv2.waitKey(1) ==ord('q'):
        break

cap.release()