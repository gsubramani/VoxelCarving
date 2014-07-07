cmake_minimum_required(VERSION 0.1)
project( CameraCalibration )
find_package( OpenCV REQUIRED )
find_package( VTK REQUIRED )
include(${VTK_USE_FILE})
IF(VTK_FOUND)
  message("found VTK. Version:" ${VTK_VERSION}. VTK_DIR: ${VTK_DIR})
ELSE(VTK_FOUND)
  MESSAGE(FATAL_ERROR "Cannot build the executable without VTK. Please set the VTK_DIR")
ENDIF(VTK_FOUND)

add_executable( CameraCalibration HelloWorld2.cpp )



target_link_libraries( CameraCalibration ${OpenCV_LIBS} )

