; Auto-generated. Do not edit!


(cl:in-package drowsiness_detection_pkg-msg)


;//! \htmlinclude Face.msg.html

(cl:defclass <Face> (roslisp-msg-protocol:ros-message)
  ((Word
    :reader Word
    :initarg :Word
    :type cl:string
    :initform ""))
)

(cl:defclass Face (<Face>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Face>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Face)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drowsiness_detection_pkg-msg:<Face> is deprecated: use drowsiness_detection_pkg-msg:Face instead.")))

(cl:ensure-generic-function 'Word-val :lambda-list '(m))
(cl:defmethod Word-val ((m <Face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drowsiness_detection_pkg-msg:Word-val is deprecated.  Use drowsiness_detection_pkg-msg:Word instead.")
  (Word m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Face>) ostream)
  "Serializes a message object of type '<Face>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Word))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Word))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Face>) istream)
  "Deserializes a message object of type '<Face>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Word) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Word) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Face>)))
  "Returns string type for a message object of type '<Face>"
  "drowsiness_detection_pkg/Face")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Face)))
  "Returns string type for a message object of type 'Face"
  "drowsiness_detection_pkg/Face")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Face>)))
  "Returns md5sum for a message object of type '<Face>"
  "65ac8484bf24836096969362e94897e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Face)))
  "Returns md5sum for a message object of type 'Face"
  "65ac8484bf24836096969362e94897e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Face>)))
  "Returns full string definition for message of type '<Face>"
  (cl:format cl:nil "string Word~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Face)))
  "Returns full string definition for message of type 'Face"
  (cl:format cl:nil "string Word~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Face>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Word))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Face>))
  "Converts a ROS message object to a list"
  (cl:list 'Face
    (cl:cons ':Word (Word msg))
))
