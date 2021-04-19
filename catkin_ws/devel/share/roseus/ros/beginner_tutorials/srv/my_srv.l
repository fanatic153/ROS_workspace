;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::my_srv)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'my_srv (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::MY_SRV")
  (make-package "BEGINNER_TUTORIALS::MY_SRV"))
(unless (find-package "BEGINNER_TUTORIALS::MY_SRVREQUEST")
  (make-package "BEGINNER_TUTORIALS::MY_SRVREQUEST"))
(unless (find-package "BEGINNER_TUTORIALS::MY_SRVRESPONSE")
  (make-package "BEGINNER_TUTORIALS::MY_SRVRESPONSE"))

(in-package "ROS")





(defclass beginner_tutorials::my_srvRequest
  :super ros::object
  :slots (_id ))

(defmethod beginner_tutorials::my_srvRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; int64 _id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass beginner_tutorials::my_srvResponse
  :super ros::object
  :slots (_name _gender _age ))

(defmethod beginner_tutorials::my_srvResponse
  (:init
   (&key
    ((:name __name) "")
    ((:gender __gender) "")
    ((:age __age) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _gender (string __gender))
   (setq _age (round __age))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:gender
   (&optional __gender)
   (if __gender (setq _gender __gender)) _gender)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _gender
    4 (length _gender)
    ;; int64 _age
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _gender
       (write-long (length _gender) s) (princ _gender s)
     ;; int64 _age
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _age (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _age) (= (length (_age . bv)) 2)) ;; bignum
              (write-long (ash (elt (_age . bv) 0) 0) s)
              (write-long (ash (elt (_age . bv) 1) -1) s))
             ((and (class _age) (= (length (_age . bv)) 1)) ;; big1
              (write-long (elt (_age . bv) 0) s)
              (write-long (if (>= _age 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _age s)(write-long (if (>= _age 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _gender
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gender (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _age
#+(or :alpha :irix6 :x86_64)
      (setf _age (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _age (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass beginner_tutorials::my_srv
  :super ros::object
  :slots ())

(setf (get beginner_tutorials::my_srv :md5sum-) "b8a2aab5099fa54e0eb1247d552c20ed")
(setf (get beginner_tutorials::my_srv :datatype-) "beginner_tutorials/my_srv")
(setf (get beginner_tutorials::my_srv :request) beginner_tutorials::my_srvRequest)
(setf (get beginner_tutorials::my_srv :response) beginner_tutorials::my_srvResponse)

(defmethod beginner_tutorials::my_srvRequest
  (:response () (instance beginner_tutorials::my_srvResponse :init)))

(setf (get beginner_tutorials::my_srvRequest :md5sum-) "b8a2aab5099fa54e0eb1247d552c20ed")
(setf (get beginner_tutorials::my_srvRequest :datatype-) "beginner_tutorials/my_srvRequest")
(setf (get beginner_tutorials::my_srvRequest :definition-)
      "int64 id
---
string name
string gender
int64 age

")

(setf (get beginner_tutorials::my_srvResponse :md5sum-) "b8a2aab5099fa54e0eb1247d552c20ed")
(setf (get beginner_tutorials::my_srvResponse :datatype-) "beginner_tutorials/my_srvResponse")
(setf (get beginner_tutorials::my_srvResponse :definition-)
      "int64 id
---
string name
string gender
int64 age

")



(provide :beginner_tutorials/my_srv "b8a2aab5099fa54e0eb1247d552c20ed")


