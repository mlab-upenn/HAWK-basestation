; Auto-generated. Do not edit!


(cl:in-package ic2020_toro-msg)


;//! \htmlinclude loopnotice.msg.html

(cl:defclass <loopnotice> (roslisp-msg-protocol:ros-message)
  ((close
    :reader close
    :initarg :close
    :type cl:fixnum
    :initform 0))
)

(cl:defclass loopnotice (<loopnotice>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <loopnotice>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'loopnotice)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ic2020_toro-msg:<loopnotice> is deprecated: use ic2020_toro-msg:loopnotice instead.")))

(cl:ensure-generic-function 'close-val :lambda-list '(m))
(cl:defmethod close-val ((m <loopnotice>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ic2020_toro-msg:close-val is deprecated.  Use ic2020_toro-msg:close instead.")
  (close m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <loopnotice>) ostream)
  "Serializes a message object of type '<loopnotice>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'close)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <loopnotice>) istream)
  "Deserializes a message object of type '<loopnotice>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'close)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<loopnotice>)))
  "Returns string type for a message object of type '<loopnotice>"
  "ic2020_toro/loopnotice")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'loopnotice)))
  "Returns string type for a message object of type 'loopnotice"
  "ic2020_toro/loopnotice")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<loopnotice>)))
  "Returns md5sum for a message object of type '<loopnotice>"
  "3db2583c538d71b20052e0f0e775249f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'loopnotice)))
  "Returns md5sum for a message object of type 'loopnotice"
  "3db2583c538d71b20052e0f0e775249f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<loopnotice>)))
  "Returns full string definition for message of type '<loopnotice>"
  (cl:format cl:nil "# char indicating we need to loop close~%uint8 close~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'loopnotice)))
  "Returns full string definition for message of type 'loopnotice"
  (cl:format cl:nil "# char indicating we need to loop close~%uint8 close~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <loopnotice>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <loopnotice>))
  "Converts a ROS message object to a list"
  (cl:list 'loopnotice
    (cl:cons ':close (close msg))
))
