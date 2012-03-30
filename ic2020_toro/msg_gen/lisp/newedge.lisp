; Auto-generated. Do not edit!


(cl:in-package ic2020_toro-msg)


;//! \htmlinclude newedge.msg.html

(cl:defclass <newedge> (roslisp-msg-protocol:ros-message)
  ((prime_keyframe
    :reader prime_keyframe
    :initarg :prime_keyframe
    :type cl:integer
    :initform 0)
   (obs_keyframe
    :reader obs_keyframe
    :initarg :obs_keyframe
    :type cl:integer
    :initform 0)
   (rot
    :reader rot
    :initarg :rot
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (trans
    :reader trans
    :initarg :trans
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass newedge (<newedge>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <newedge>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'newedge)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ic2020_toro-msg:<newedge> is deprecated: use ic2020_toro-msg:newedge instead.")))

(cl:ensure-generic-function 'prime_keyframe-val :lambda-list '(m))
(cl:defmethod prime_keyframe-val ((m <newedge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_toro-msg:prime_keyframe-val is deprecated.  Use ic2020_toro-msg:prime_keyframe instead.")
  (prime_keyframe m))

(cl:ensure-generic-function 'obs_keyframe-val :lambda-list '(m))
(cl:defmethod obs_keyframe-val ((m <newedge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_toro-msg:obs_keyframe-val is deprecated.  Use ic2020_toro-msg:obs_keyframe instead.")
  (obs_keyframe m))

(cl:ensure-generic-function 'rot-val :lambda-list '(m))
(cl:defmethod rot-val ((m <newedge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_toro-msg:rot-val is deprecated.  Use ic2020_toro-msg:rot instead.")
  (rot m))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <newedge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_toro-msg:trans-val is deprecated.  Use ic2020_toro-msg:trans instead.")
  (trans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <newedge>) ostream)
  "Serializes a message object of type '<newedge>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'prime_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'prime_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'prime_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'prime_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obs_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obs_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'obs_keyframe)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'obs_keyframe)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'rot))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'trans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <newedge>) istream)
  "Deserializes a message object of type '<newedge>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'prime_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'prime_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'prime_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'prime_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obs_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obs_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'obs_keyframe)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'obs_keyframe)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rot) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rot)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trans)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<newedge>)))
  "Returns string type for a message object of type '<newedge>"
  "ic2020_toro/newedge")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'newedge)))
  "Returns string type for a message object of type 'newedge"
  "ic2020_toro/newedge")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<newedge>)))
  "Returns md5sum for a message object of type '<newedge>"
  "4dc59ec3b0c54addb13a6d290b0c5b3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'newedge)))
  "Returns md5sum for a message object of type 'newedge"
  "4dc59ec3b0c54addb13a6d290b0c5b3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<newedge>)))
  "Returns full string definition for message of type '<newedge>"
  (cl:format cl:nil "# Index of Keyframes~%uint32 prime_keyframe~%uint32 obs_keyframe~%~%# Rotation and Translation~%float32[] rot~%float32[] trans~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'newedge)))
  "Returns full string definition for message of type 'newedge"
  (cl:format cl:nil "# Index of Keyframes~%uint32 prime_keyframe~%uint32 obs_keyframe~%~%# Rotation and Translation~%float32[] rot~%float32[] trans~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <newedge>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <newedge>))
  "Converts a ROS message object to a list"
  (cl:list 'newedge
    (cl:cons ':prime_keyframe (prime_keyframe msg))
    (cl:cons ':obs_keyframe (obs_keyframe msg))
    (cl:cons ':rot (rot msg))
    (cl:cons ':trans (trans msg))
))
