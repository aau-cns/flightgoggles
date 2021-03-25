
# Include ROS dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compile ROS node
add_executable(ROSClient ROSClient.cpp)
add_dependencies(ROSClient ${PROJECT_NAME}_gencfg)
target_link_libraries(ROSClient FlightGogglesClientLib CameraModelLib pthread ${catkin_LIBRARIES} ${OpenCV_LIBS})





