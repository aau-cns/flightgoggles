# Always compile these dirs
add_subdirectory(Common)
add_subdirectory(GeneralClient)
add_subdirectory(CameraModel)

# Only compile ROS client if ROS is installed.
if(COMPILE_ROSCLIENT)
  add_subdirectory(ROSClient)
endif()
