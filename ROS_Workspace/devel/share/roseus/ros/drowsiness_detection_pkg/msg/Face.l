;; Auto-generated. Do not edit!


(when (boundp 'drowsiness_detection_pkg::Face)
  (if (not (find-package "DROWSINESS_DETECTION_PKG"))
    (make-package "DROWSINESS_DETECTION_PKG"))
  (shadow 'Face (find-package "DROWSINESS_DETECTION_PKG")))
(unless (find-package "DROWSINESS_DETECTION_PKG::FACE")
  (make-package "DROWSINESS_DETECTION_PKG::FACE"))

(in-package "ROS")
;;//! \htmlinclude Face.msg.html


(defclass drowsiness_detection_pkg::Face
  :super ros::object
  :slots (_Word ))

(defmethod drowsiness_detection_pkg::Face
  (:init
   (&key
    ((:Word __Word) "")
    )
   (send-super :init)
   (setq _Word (string __Word))
   self)
  (:Word
   (&optional __Word)
   (if __Word (setq _Word __Word)) _Word)
  (:serialization-length
   ()
   (+
    ;; string _Word
    4 (length _Word)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Word
       (write-long (length _Word) s) (princ _Word s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Word
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Word (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get drowsiness_detection_pkg::Face :md5sum-) "65ac8484bf24836096969362e94897e2")
(setf (get drowsiness_detection_pkg::Face :datatype-) "drowsiness_detection_pkg/Face")
(setf (get drowsiness_detection_pkg::Face :definition-)
      "string Word

")



(provide :drowsiness_detection_pkg/Face "65ac8484bf24836096969362e94897e2")


