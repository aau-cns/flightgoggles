# Set library files
set(LIB_FILES
  CameraModel.cpp
  CameraModel.hpp
  FisheyeModel.cpp
  FisheyeModel.hpp
)


# Add CameraModels as library
add_library(CameraModelLib SHARED ${LIB_FILES})

# Link in needed libraries
target_link_libraries(CameraModelLib ${OpenCV_LIBS})

# Expose as library
target_include_directories(CameraModelLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

