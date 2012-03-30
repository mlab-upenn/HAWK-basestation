
(cl:in-package :asdf)

(defsystem "ic2020_render-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rendupdate" :depends-on ("_package_rendupdate"))
    (:file "_package_rendupdate" :depends-on ("_package"))
  ))