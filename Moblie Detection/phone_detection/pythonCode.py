import cv2
from ultralytics import YOLO
import numpy as np


cap= cv2.VideoCapture(0)

model=YOLO("yolov8m_custom.pt")

while True:
    ret, frame= cap.read()
    if not ret:
        break

    results=model(frame,device="0")
    result=results[0]
    bboxes=np.array(result.boxes.xyxy.cpu(),dtype="int")
    classes=np.array(result.boxes.cls.cpu(),dtype="int")
    if classes == 0:
        print("phone")
    for cls, bbox in zip(classes,bboxes):
        (x,y,x2,y2)=bbox

        cv2.rectangle(frame,(x,y),(x2,y2),(0,0,255),2)
        cv2.putText(frame,str("phone"),(x,y-5), cv2.FONT_HERSHEY_PLAIN,1,(0,0,255),2)
        



    cv2.imshow("Img",frame)
    Key=cv2.waitKey(1)
    if Key==27:
        break

cap.relaese()
cv2.destroyAllWindow()