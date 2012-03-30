
(cl:in-package :asdf)

(defsystem "ic2020_toro-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "newedge" :depends-on ("_package_newedge"))
    (:file "_package_newedge" :depends-on ("_package"))
    (:file "loopnotice" :depends-on ("_package_loopnotice"))
    (:file "_package_loopnotice" :depends-on ("_package"))
  ))