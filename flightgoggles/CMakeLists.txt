cmake_minimum_required(VERSION 2.8.3)
project(flightgoggles)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        sensor_msgs
        message_generation)



add_message_files(
        FILES
        IRMarker.msg
        IRMarkerArray.msg
        FGParam.msg
)

# Generate services in the 'srv' folder
add_service_files(
        FILES
        SetTransformStamped.srv
        GetImage.srv
        SetTransformStampedArr.srv
)


generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)

# CMAKE Variable that defines if binary renderer is downloaded during install
if(NOT DEFINED FLIGHTGOGGLES_DOWNLOAD_BINARY)
  set(FLIGHTGOGGLES_DOWNLOAD_BINARY ON)
endif(NOT DEFINED FLIGHTGOGGLES_DOWNLOAD_BINARY)

# DOWNLOAD AND INSTALL FLIGHTGOGGLES RENDERER
include(ExternalData)

# Locations to download from
set(ExternalData_URL_TEMPLATES
        "http://d34kgw45d3q5oc.cloudfront.net/Public/FlightGoggles/HashedReleases/%(algo)/%(hash)"
        "ftp://fast-nas.mit.edu/Public/FlightGoggles/HashedReleases/%(algo)/%(hash)"
        )

# Will be automatically cleaned on restart.
set(ExternalData_OBJECT_STORES
        "/tmp/flightgoggles_binary_cache"
        )

# Allow for extended timeout for downloads
set(ExternalData_TIMEOUT_ABSOLUTE 3600)
set(ExternalData_TIMEOUT_INACTIVITY 0)

# Define FG renderer version required
# Get location of .tar.gz file
ExternalData_Expand_Arguments(flightgogglesBinaryDownload
        flightgogglesBinaryData
        DATA{renderer_version/FlightGoggles_Data.tar.gz} # Input arguments, DATA{} allowed
)
ExternalData_Expand_Arguments(flightgogglesBinaryDownload
        flightgogglesBinary
        DATA{renderer_version/FlightGoggles.x86_64} # Input arguments, DATA{} allowed
        )
if(FLIGHTGOGGLES_DOWNLOAD_BINARY)
ExternalData_Add_Target(flightgogglesBinaryDownload)
endif(FLIGHTGOGGLES_DOWNLOAD_BINARY)

# get hash of data file for use in .stamp
file(READ renderer_version/FlightGoggles_Data.tar.gz.md5 flightgogglesVersionHash)

# Should only be run if output does not exist.
add_custom_command(
        OUTPUT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${flightgogglesVersionHash}.stamp"
        DEPENDS flightgogglesBinaryDownload
        WORKING_DIRECTORY "${CATKIN_DEVEL_PREFIX}"
        # Make install directory
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}"
        # Mark as executable
        COMMAND chmod +x "${flightgogglesBinary}"
        # Make directory for data
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/FlightGoggles_Data"
        COMMAND cd "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/FlightGoggles_Data" && ${CMAKE_COMMAND} -E tar xfz "${flightgogglesBinaryData}"
        # Copy executable
        COMMAND ${CMAKE_COMMAND} -E copy "${flightgogglesBinary}" "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}"
        # Update output timestamp file
        COMMAND ${CMAKE_COMMAND} -E touch "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${flightgogglesVersionHash}.stamp"
        )

if(FLIGHTGOGGLES_DOWNLOAD_BINARY)
  add_custom_target(flightgogglesBinaryUnpack ALL DEPENDS "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${flightgogglesVersionHash}.stamp")
endif(FLIGHTGOGGLES_DOWNLOAD_BINARY)

set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
        "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/."
        )


# Download FG executable to: ${CATKIN_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}

include_directories()


## Mark files for installation
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/FlightGoggles.x86_64
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
