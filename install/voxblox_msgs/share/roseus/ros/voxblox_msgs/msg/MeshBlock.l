;; Auto-generated. Do not edit!


(when (boundp 'voxblox_msgs::MeshBlock)
  (if (not (find-package "VOXBLOX_MSGS"))
    (make-package "VOXBLOX_MSGS"))
  (shadow 'MeshBlock (find-package "VOXBLOX_MSGS")))
(unless (find-package "VOXBLOX_MSGS::MESHBLOCK")
  (make-package "VOXBLOX_MSGS::MESHBLOCK"))

(in-package "ROS")
;;//! \htmlinclude MeshBlock.msg.html


(defclass voxblox_msgs::MeshBlock
  :super ros::object
  :slots (_index _x _y _z _r _g _b ))

(defmethod voxblox_msgs::MeshBlock
  (:init
   (&key
    ((:index __index) (make-array 3 :initial-element 0 :element-type :integer))
    ((:x __x) (make-array 0 :initial-element 0 :element-type :integer))
    ((:y __y) (make-array 0 :initial-element 0 :element-type :integer))
    ((:z __z) (make-array 0 :initial-element 0 :element-type :integer))
    ((:r __r) (make-array 0 :initial-element 0 :element-type :char))
    ((:g __g) (make-array 0 :initial-element 0 :element-type :char))
    ((:b __b) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _index __index)
   (setq _x __x)
   (setq _y __y)
   (setq _z __z)
   (setq _r __r)
   (setq _g __g)
   (setq _b __b)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:g
   (&optional __g)
   (if __g (setq _g __g)) _g)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; int64[3] _index
    (* 8    3)
    ;; uint16[] _x
    (* 2    (length _x)) 4
    ;; uint16[] _y
    (* 2    (length _y)) 4
    ;; uint16[] _z
    (* 2    (length _z)) 4
    ;; uint8[] _r
    (* 1    (length _r)) 4
    ;; uint8[] _g
    (* 1    (length _g)) 4
    ;; uint8[] _b
    (* 1    (length _b)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64[3] _index
     (dotimes (i 3)
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _index i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _index i)) (= (length ((elt _index i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _index i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _index i) . bv) 1) -1) s))
             ((and (class (elt _index i)) (= (length ((elt _index i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _index i) . bv) 0) s)
              (write-long (if (>= (elt _index i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _index i) s)(write-long (if (>= (elt _index i) 0) 0 #xffffffff) s)))
       )
     ;; uint16[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (write-word (elt _x i) s)
       )
     ;; uint16[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (write-word (elt _y i) s)
       )
     ;; uint16[] _z
     (write-long (length _z) s)
     (dotimes (i (length _z))
       (write-word (elt _z i) s)
       )
     ;; uint8[] _r
     (write-long (length _r) s)
     (princ _r s)
     ;; uint8[] _g
     (write-long (length _g) s)
     (princ _g s)
     ;; uint8[] _b
     (write-long (length _b) s)
     (princ _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64[3] _index
   (dotimes (i (length _index))
#+(or :alpha :irix6 :x86_64)
      (setf (elt _index i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _index i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     )
   ;; uint16[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _z i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint8[] _r
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _r (make-array n :element-type :char))
     (replace _r buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _g
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _g (make-array n :element-type :char))
     (replace _g buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _b
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _b (make-array n :element-type :char))
     (replace _b buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get voxblox_msgs::MeshBlock :md5sum-) "a2921da2fac1fa36ed1e9d85b1e1b29c")
(setf (get voxblox_msgs::MeshBlock :datatype-) "voxblox_msgs/MeshBlock")
(setf (get voxblox_msgs::MeshBlock :definition-)
      "# vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 
# no alpha information

# Index of meshed points in block map
int64[3] index

# Triangle information (always in groups of 3)
uint16[] x
uint16[] y
uint16[] z

# Color information may be missing
uint8[] r
uint8[] g
uint8[] b
")



(provide :voxblox_msgs/MeshBlock "a2921da2fac1fa36ed1e9d85b1e1b29c")


