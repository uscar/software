
(cl:in-package :asdf)

(defsystem "art_laser-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Lines" :depends-on ("_package_Lines"))
    (:file "_package_Lines" :depends-on ("_package"))
  ))