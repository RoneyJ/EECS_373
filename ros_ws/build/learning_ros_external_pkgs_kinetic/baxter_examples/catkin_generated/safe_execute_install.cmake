execute_process(COMMAND "/home/jproney/ros_ws/build/learning_ros_external_pkgs_kinetic/baxter_examples/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jproney/ros_ws/build/learning_ros_external_pkgs_kinetic/baxter_examples/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
