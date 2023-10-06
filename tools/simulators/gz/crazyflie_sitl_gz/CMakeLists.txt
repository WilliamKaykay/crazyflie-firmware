cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

project(crazyflie_gazebo_sitl VERSION 1.0.0)
include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS system thread timer)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(OGRE OGRE)
find_package(gz-sim7 REQUIRED)
find_package(gz-transport12 REQUIRED OPTIONAL_COMPONENTS log)
find_package(gz-msgs9 REQUIRED)

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER}
                           PROPERTIES GENERATED TRUE)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost 1.40 COMPONENTS system thread timer REQUIRED )

# =============================================================================================== #
# ====================================== CF COMMUNICATION LIB =================================== #
# =============================================================================================== #
# set (CRAZYFLIE_COMM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/crazyflie_comm/include)
# set (CRAZYFLIE_COMM_LIBRARY crazyflie_comm)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/crazyflie_comm)
# include_directories(
#   ${CRAZYFLIE_COMM_INCLUDE_DIR}
# )
# list(APPEND targets_to_install crazyflie_comm)
#================================================================================================ #


include_directories(
  include
  ${GZ-SIM_INCLUDE_DIRS}
  ${GZ-MSGS_INCLUDE_DIRS}
  ${GZ-TRANSPORT_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Boost_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}/eigen3
  ${EIGEN3_INCLUDE_DIRS}/eigen3
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging
  /usr/local/include/OGRE
  /usr/local/include/OGRE/Paging
)


link_libraries(
  ${GZ-SIM_LIBRARIES}
  ${GZ-MSGS_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
)


link_directories(
    ${GZ-SIM_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OGRE_LIBRARY_DIRS}
)

#======================================= gz_CrazyflieInterface PLUGIN ==========================================//
add_library(gz_CrazyflieInterface_plugin SHARED src/gz_CrazyflieInterface_plugin.cpp)
set_property(TARGET gz_CrazyflieInterface_plugin PROPERTY ENABLE_EXPORTS ON)
target_link_libraries(gz_CrazyflieInterface_plugin 
  # ${CRAZYFLIE_COMM_LIBRARY} 
  ${GZ-SIM_LIBRARIES} 
  ${GZ-TRANSPORT_LIBRARIES} 
  ${GZ-MSGS_LIBRARIES}
)

list(APPEND targets_to_install gz_CrazyflieInterface_plugin)
#==============================================================================================//

set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)

install(TARGETS ${plugins} DESTINATION ${PLUGIN_PATH})
install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)