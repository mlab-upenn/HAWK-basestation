; Auto-generated. Do not edit!


(in-package ic2020_vodom-msg)


;//! \htmlinclude keyframe.msg.html

(defclass <keyframe> (ros-message)
  ((keyframe_num
    :reader keyframe_num-val
    :initarg :keyframe_num
    :type integer
    :initform 0)
   (rotation
    :reader rotation-val
    :initarg :rotation
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (translation
    :reader translation-val
    :initarg :translation
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (height
    :reader height-val
    :initarg :height
    :type integer
    :initform 0)
   (width
    :reader width-val
    :initarg :width
    :type integer
    :initform 0)
   (im
    :reader im-val
    :initarg :im
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0))
   (numberOf3DPoints
    :reader numberOf3DPoints-val
    :initarg :numberOf3DPoints
    :type integer
    :initform 0)
   (point_step
    :reader point_step-val
    :initarg :point_step
    :type integer
    :initform 0)
   (points
    :reader points-val
    :initarg :points
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0))
   (numCorn1
    :reader numCorn1-val
    :initarg :numCorn1
    :type integer
    :initform 0)
   (corn1
    :reader corn1-val
    :initarg :corn1
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (status
    :reader status-val
    :initarg :status
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0))
   (numCorn2
    :reader numCorn2-val
    :initarg :numCorn2
    :type integer
    :initform 0)
   (corn2
    :reader corn2-val
    :initarg :corn2
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (numSURF
    :reader numSURF-val
    :initarg :numSURF
    :type integer
    :initform 0)
   (features
    :reader features-val
    :initarg :features
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (descBuffer
    :reader descBuffer-val
    :initarg :descBuffer
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (surfMatches
    :reader surfMatches-val
    :initarg :surfMatches
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (imux
    :reader imux-val
    :initarg :imux
    :type float
    :initform 0.0)
   (imuy
    :reader imuy-val
    :initarg :imuy
    :type float
    :initform 0.0)
   (imuz
    :reader imuz-val
    :initarg :imuz
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <keyframe>) ostream)
  "Serializes a message object of type '<keyframe>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'keyframe_num)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'keyframe_num)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'keyframe_num)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'keyframe_num)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'rotation))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'rotation))
  (let ((__ros_arr_len (length (slot-value msg 'translation))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'translation))
    (write-byte (ldb (byte 8 0) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'height)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'width)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'im))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'im))
    (write-byte (ldb (byte 8 0) (slot-value msg 'numberOf3DPoints)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'numberOf3DPoints)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'numberOf3DPoints)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'numberOf3DPoints)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'point_step)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'point_step)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'point_step)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'point_step)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'points))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'points))
    (write-byte (ldb (byte 8 0) (slot-value msg 'numCorn1)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'numCorn1)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'numCorn1)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'numCorn1)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'corn1))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'corn1))
  (let ((__ros_arr_len (length (slot-value msg 'status))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'status))
    (write-byte (ldb (byte 8 0) (slot-value msg 'numCorn2)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'numCorn2)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'numCorn2)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'numCorn2)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'corn2))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'corn2))
    (write-byte (ldb (byte 8 0) (slot-value msg 'numSURF)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'numSURF)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'numSURF)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'numSURF)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'features))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'features))
  (let ((__ros_arr_len (length (slot-value msg 'descBuffer))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'descBuffer))
  (let ((__ros_arr_len (length (slot-value msg 'surfMatches))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'surfMatches))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'imux))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'imuy))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'imuz))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <keyframe>) istream)
  "Deserializes a message object of type '<keyframe>"
  (setf (ldb (byte 8 0) (slot-value msg 'keyframe_num)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'keyframe_num)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'keyframe_num)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'keyframe_num)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'rotation) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'rotation)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'translation) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'translation)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (setf (ldb (byte 8 0) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'width)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'im) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'im)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'numberOf3DPoints)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'numberOf3DPoints)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'numberOf3DPoints)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'numberOf3DPoints)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'point_step)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'point_step)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'point_step)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'point_step)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'points) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'points)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'numCorn1)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'numCorn1)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'numCorn1)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'numCorn1)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'corn1) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'corn1)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'status) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'status)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'numCorn2)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'numCorn2)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'numCorn2)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'numCorn2)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'corn2) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'corn2)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (setf (ldb (byte 8 0) (slot-value msg 'numSURF)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'numSURF)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'numSURF)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'numSURF)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'features) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'features)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'descBuffer) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'descBuffer)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'surfMatches) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'surfMatches)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'imux) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'imuy) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'imuz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<keyframe>)))
  "Returns string type for a message object of type '<keyframe>"
  "ic2020_vodom/keyframe")
(defmethod md5sum ((type (eql '<keyframe>)))
  "Returns md5sum for a message object of type '<keyframe>"
  "0ffd2f798c68a348fd55754d5e039856")
(defmethod message-definition ((type (eql '<keyframe>)))
  "Returns full string definition for message of type '<keyframe>"
  (format nil "uint32 keyframe_num~%~%# Rotation and Translation~%float32[] rotation~%float32[] translation~%~%# Image~%uint32 height~%uint32 width~%uint8[] im~%~%# Depth Points~%uint32 numberOf3DPoints~%uint32 point_step #used for getting point in single row based off x and y coords~%uint8[] points~%~%# Shi Tomasi Corners~%uint32 numCorn1~%float32[] corn1 # relates this keyframe to last~%uint8[] status # which ones are good~%~%uint32 numCorn2~%float32[] corn2 # corners found in this keyframe to relate to next~%~%# SURF~%uint32 numSURF~%float32[] features~%float32[] descBuffer~%int32[] surfMatches # index of matches in previous keyframe, -1 means no match~%~%# IMU~%float32 imux~%float32 imuy~%float32 imuz~%~%~%"))
(defmethod serialization-length ((msg <keyframe>))
  (+ 0
     4
     4 (reduce #'+ (slot-value msg 'rotation) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'translation) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4 (reduce #'+ (slot-value msg 'im) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
     4
     4
     4 (reduce #'+ (slot-value msg 'points) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
     4
     4 (reduce #'+ (slot-value msg 'corn1) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'status) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
     4
     4 (reduce #'+ (slot-value msg 'corn2) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4 (reduce #'+ (slot-value msg 'features) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'descBuffer) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'surfMatches) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <keyframe>))
  "Converts a ROS message object to a list"
  (list '<keyframe>
    (cons ':keyframe_num (keyframe_num-val msg))
    (cons ':rotation (rotation-val msg))
    (cons ':translation (translation-val msg))
    (cons ':height (height-val msg))
    (cons ':width (width-val msg))
    (cons ':im (im-val msg))
    (cons ':numberOf3DPoints (numberOf3DPoints-val msg))
    (cons ':point_step (point_step-val msg))
    (cons ':points (points-val msg))
    (cons ':numCorn1 (numCorn1-val msg))
    (cons ':corn1 (corn1-val msg))
    (cons ':status (status-val msg))
    (cons ':numCorn2 (numCorn2-val msg))
    (cons ':corn2 (corn2-val msg))
    (cons ':numSURF (numSURF-val msg))
    (cons ':features (features-val msg))
    (cons ':descBuffer (descBuffer-val msg))
    (cons ':surfMatches (surfMatches-val msg))
    (cons ':imux (imux-val msg))
    (cons ':imuy (imuy-val msg))
    (cons ':imuz (imuz-val msg))
))
