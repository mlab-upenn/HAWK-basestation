; Auto-generated. Do not edit!


(cl:in-package ic2020_render-msg)


;//! \htmlinclude rendupdate.msg.html

(cl:defclass <rendupdate> (roslisp-msg-protocol:ros-message)
  ((numOfUpdates
    :reader numOfUpdates
    :initarg :numOfUpdates
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass rendupdate (<rendupdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rendupdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rendupdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ic2020_render-msg:<rendupdate> is deprecated: use ic2020_render-msg:rendupdate instead.")))

(cl:ensure-generic-function 'numOfUpdates-val :lambda-list '(m))
(cl:defmethod numOfUpdates-val ((m <rendupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_render-msg:numOfUpdates-val is deprecated.  Use ic2020_render-msg:numOfUpdates instead.")
  (numOfUpdates m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <rendupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_render-msg:data-val is deprecated.  Use ic2020_render-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rendupdate>) ostream)
  "Serializes a message object of type '<rendupdate>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numOfUpdates)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numOfUpdates)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numOfUpdates)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numOfUpdates)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rendupdate>) istream)
  "Deserializes a message object of type '<rendupdate>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numOfUpdates)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numOfUpdates)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numOfUpdates)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numOfUpdates)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rendupdate>)))
  "Returns string type for a message object of type '<rendupdate>"
  "ic2020_render/rendupdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rendupdate)))
  "Returns string type for a message object of type 'rendupdate"
  "ic2020_render/rendupdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rendupdate>)))
  "Returns md5sum for a message object of type '<rendupdate>"
  "04c332cac37137a642a8f44f295c4870")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rendupdate)))
  "Returns md5sum for a message object of type 'rendupdate"
  "04c332cac37137a642a8f44f295c4870")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rendupdate>)))
  "Returns full string definition for message of type '<rendupdate>"
  (cl:format cl:nil "# Array of update structures~%uint32 numOfUpdates~%uint8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rendupdate)))
  "Returns full string definition for message of type 'rendupdate"
  (cl:format cl:nil "# Array of update structures~%uint32 numOfUpdates~%uint8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rendupdate>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rendupdate>))
  "Converts a ROS message object to a list"
  (cl:list 'rendupdate
    (cl:cons ':numOfUpdates (numOfUpdates msg))
    (cl:cons ':data (data msg))
))
