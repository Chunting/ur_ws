
(cl:in-package :asdf)

(defsystem "lesson_move_group-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sensor_accessor" :depends-on ("_package_sensor_accessor"))
    (:file "_package_sensor_accessor" :depends-on ("_package"))
  ))