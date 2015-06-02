cmake_minimum_required(VERSION 2.8.3)
project(kinfu_ros)

#compute flags macros
MACRO(CUDA_COMPUTE_TARGET_FLAGS arch_bin arch_ptx cuda_nvcc_target_flags)
    string(REGEX REPLACE "\\." "" ARCH_BIN_WITHOUT_DOTS "${${arch_bin}}")
    string(REGEX REPLACE "\\." "" ARCH_PTX_WITHOUT_DOTS "${${arch_ptx}}")
                                
    set(cuda_computer_target_flags_temp "") 
    
    # Tell NVCC to add binaries for the specified GPUs
    string(REGEX MATCHALL "[0-9()]+" ARCH_LIST "${ARCH_BIN_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
        if (ARCH MATCHES "([0-9]+)\\(([0-9]+)\\)")
            # User explicitly specified PTX for the concrete BIN                    
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${CMAKE_MATCH_2},code=sm_${CMAKE_MATCH_1})                 
        else()                  
            # User didn't explicitly specify PTX for the concrete BIN, we assume PTX=BIN                                
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=sm_${ARCH})                   
        endif()
    endforeach()
                
    # Tell NVCC to add PTX intermediate code for the specified architectures
    string(REGEX MATCHALL "[0-9]+" ARCH_LIST "${ARCH_PTX_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)                
        set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=compute_${ARCH})              
    endforeach()    
                            
    set(${cuda_nvcc_target_flags} ${cuda_computer_target_flags_temp})       
ENDMACRO()

MACRO(APPEND_TARGET_ARCH_FLAGS)
    set(cuda_nvcc_target_flags "")
    CUDA_COMPUTE_TARGET_FLAGS(CUDA_ARCH_BIN CUDA_ARCH_PTX cuda_nvcc_target_flags)       
    if (cuda_nvcc_target_flags)
        message(STATUS "CUDA NVCC target flags: ${cuda_nvcc_target_flags}")
        list(APPEND CUDA_NVCC_FLAGS ${cuda_nvcc_target_flags})
    endif()
ENDMACRO()


list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE MODULE PATH IS ${CMAKE_MODULE_PATH}")
#include(cmake/Utils.cmake)
#include(cmake/Targets.cmake)

## System dependencies are found with CMake's conventions
find_package(OpenCV 2.4.9 REQUIRED COMPONENTS core highgui)
message(STATUS "Found OpenCV At ${OpenCV_INCLUDE_DIRS}")

find_package(CUDA REQUIRED)
if(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
        set(CUDA_ARCH_BIN "2.0 2.1(2.0) 3.0 5.0 5.2" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
    else()
        set(CUDA_ARCH_BIN "2.0 2.1(2.0)" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
    endif()
APPEND_TARGET_ARCH_FLAGS()
message(STATUS "CUDA_BIN_ARCH ${CUDA_ARCH_BIN}")
find_package(OpenNI REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rosconsole
  roscpp
  sensor_msgs
  tf
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinfu_ros
  CATKIN_DEPENDS cv_bridge image_transport rosconsole roscpp sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIR}
)

set(HAVE_CUDA 1)
#list(APPEND CUDA_NVCC_FLAGS "-arch;compute_20")
list(APPEND CUDA_NVCC_FLAGS "-gencode;arch=compute_20,code=sm_20;-gencode;arch=compute_20,code=sm_21;-gencode;arch=compute_30,code=sm_30;-gencode;arch=compute_35,code=sm_35")
  list(APPEND CUDA_NVCC_FLAGS "-Xcompiler;-fPIC;")

include_directories(${PROJECT_SOURCE_DIR}/include/kinfu_ros)
include_directories(${PROJECT_SOURCE_DIR}/src)

## Declare a cpp library
 cuda_add_library(kinfu_ros
	src/capture.cpp
	src/core.cpp
	src/device_memory.cpp
	src/imgproc.cpp
	src/kinfu.cpp
	src/precomp.cpp
	src/projective_icp.cpp
	src/tsdf_volume.cpp
	src/cuda/imgproc.cu
	src/cuda/proj_icp.cu
	src/cuda/tsdf_volume.cu
	src/ros/ros_rgbd_camera.cpp
 )

 target_link_libraries(kinfu_ros
   ${catkin_LIBRARIES}
   ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${OpenCV_LIBS} 
   ${OPENNI_LIBRARIES}
 )

## Declare a cpp executable
add_executable(kinfu_node src/kinfu_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(kinfu_ros_node kinfu_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(kinfu_node
   kinfu_ros
   ${catkin_LIBRARIES}
   ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${OpenCV_LIBS} 
   ${OPENNI_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kinfu_ros kinfu_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinfu_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
