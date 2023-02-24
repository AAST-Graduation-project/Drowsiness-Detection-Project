
from argparse import ArgumentParser
from math import dist
from cv2 import (COLOR_BGR2GRAY, FONT_HERSHEY_COMPLEX, cvtColor, VideoCapture, resize, INTER_CUBIC,
                 destroyAllWindows, imshow, polylines, putText, waitKey)
from dlib import get_frontal_face_detector, shape_predictor

from numpy import array
import time


counter, flag, avg_EAR, initial_EAR = 0, 0, 0, 0


dlib_face = shape_predictor("/home/mario/Downloads/shape_predictor_68_face_landmarks.dat")
face_detector = get_frontal_face_detector()


# ap = ArgumentParser()
# ap.add_argument("-w", "--webcam", type=int, default=0)
# args = vars(ap.parse_args())
# cap = VideoStream(src=args["webcam"]).start()

# cap = VideoStream(usePiCamera=True).start()  # For Raspberry Pi

cap = VideoCapture(0) # opencv


def Get_Eye(face_landmark_part):
    right_eye = array([(face_landmark_part(36).x, face_landmark_part(36).y),
                          (face_landmark_part(37).x, face_landmark_part(37).y),
                          (face_landmark_part(38).x, face_landmark_part(38).y),
                          (face_landmark_part(39).x, face_landmark_part(39).y),
                          (face_landmark_part(40).x, face_landmark.part(40).y),
                          (face_landmark.part(41).x, face_landmark.part(41).y)])
    left_eye = array([(face_landmark_part(42).x, face_landmark_part(42).y),
                        (face_landmark_part(43).x, face_landmark_part(43).y),
                        (face_landmark_part(44).x, face_landmark_part(44).y),
                        (face_landmark_part(45).x, face_landmark_part(45).y),
                        (face_landmark_part(46).x, face_landmark_part(46).y),
                        (face_landmark_part(47).x, face_landmark_part(47).y)])

    return right_eye, left_eye

def EAR_Calc(eye):
    A = dist((eye[1]), (eye[5]))
    B = dist((eye[2]), (eye[4]))
    C = dist((eye[0]), (eye[3]))
    EAR = (A+B)/(2.0*C)
    return EAR


while True:
    start = time.time()
    frame = cap.read()
    frame = resize(frame,None,fx=0.7,fy=0.7)
    gray = cvtColor(frame, COLOR_BGR2GRAY)
    faces = face_detector(gray)
    if initial_EAR == 0:
        initial_counter = 0
        while initial_counter < 40:
            initial_counter += 1
            faces = face_detector(gray)
            for face in faces:
                face_landmark = dlib_face(gray, face)
                right_eye, left_eye = Get_Eye(face_landmark.part)
                right_EAR = EAR_Calc(right_eye)
                left_EAR = EAR_Calc(left_eye)
                avg_EAR = round((left_EAR+right_EAR)/2.0, 2)
                if (initial_EAR < avg_EAR):
                    initial_EAR = avg_EAR

    for face in faces:
        face_landmark = dlib_face(gray, face)
        right_eye, left_eye = Get_Eye(face_landmark.part)
        right_EAR = EAR_Calc(right_eye)
        left_EAR = EAR_Calc(left_eye)
        avg_EAR = round((left_EAR+right_EAR)/2.0, 2)
        polylines(frame, [right_eye], 1, (0, 255, 0), 1)
        polylines(frame, [left_eye], 1, (0, 255, 0), 1)
        avg_EAR = round((left_EAR+right_EAR)/2.0, 2)
        putText(frame, ("EAR="+(str(avg_EAR))), (10, 60),
            FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        putText(frame, 'initial_EAR='+(str(initial_EAR)), (10, 20),
            FONT_HERSHEY_COMPLEX, 0.7, (100, 0, 100), 2)
    if avg_EAR < (initial_EAR*0.6):
        counter += 1
        if counter > 45:
            putText(frame, "SLEEP", (10, 100),
                    FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
            flag = 1
    elif (avg_EAR >= (initial_EAR*0.6) and flag == 1):
        counter = 0
        flag = 0

    imshow('Result', frame)
    
    print(time.time()-start)
    if waitKey(1) == ord('q'):
        destroyAllWindows()
        #cap.stop()
        break

