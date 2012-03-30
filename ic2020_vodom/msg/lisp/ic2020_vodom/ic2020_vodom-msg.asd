
(in-package :asdf)

(defsystem "ic2020_vodom-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "keyframe" :depends-on ("_package"))
    (:file "_package_keyframe" :depends-on ("_package"))
    ))
