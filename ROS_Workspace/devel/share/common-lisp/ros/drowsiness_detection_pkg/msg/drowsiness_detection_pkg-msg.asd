
(cl:in-package :asdf)

(defsystem "drowsiness_detection_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Face" :depends-on ("_package_Face"))
    (:file "_package_Face" :depends-on ("_package"))
  ))