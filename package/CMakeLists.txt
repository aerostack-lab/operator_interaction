##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME operator_interaction)
project(${PROJECT_NAME} C CXX)

add_definitions(-std=c++11)
##############################################################################
# Catkin
##############################################################################
set(OPERATOR_INTERACTION_INCLUDE_DIR
        src/controller/include)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS droneMsgsROS roscpp std_msgs behavior_execution_manager behavior_execution_manager_msgs aerostack_msgs rqt_gui_cpp rqt_gui )
catkin_package(INCLUDE_DIRS
  ${OPERATOR_INTERACTION_INCLUDE_DIR}
  ${OPERATOR_INTERACTION_INCLUDE_DIR}
  CATKIN_DEPENDS droneMsgsROS roscpp std_msgs behavior_execution_manager behavior_execution_manager_msgs aerostack_msgs
  DEPENDS yaml-cpp)

include_directories(${OPERATOR_INTERACTION_INCLUDE_DIR})
include_directories(${OPERATOR_INTERACTION_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# check required dependencies
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

##############################################################################
# Qwt library
##############################################################################

FIND_PATH(QWT_INCLUDE_DIR NAMES qwt.h PATHS
  /usr/include
  PATH_SUFFIXES qwt
)

MESSAGE(STATUS "qwt header: ${QWT_INCLUDE_DIR}")

find_library(QWT_LIB NAMES qwt-qt5)

MESSAGE(STATUS "Found Qwt: ${QWT_LIB}")

#set(QWT_LIBRARY "/usr/lib/libqwt-qt5.so.6.1.2")


include_directories(${QWT_INCLUDE_DIR})
link_directories(${QWT_LIBRARY})

##############################################################################
# Qt Environment
##############################################################################

# included via the dependency call in package.xml
find_package(Qt5 COMPONENTS Core Gui Svg OpenGL Widgets PrintSupport REQUIRED)
#include(${QT_USE_FILE})

## to avoid conflict with boost signals it is needed to define QT_NO_KEYWORDS.
add_definitions(-DQT_NO_KEYWORDS)
ADD_DEFINITIONS(${QT_DEFINITIONS})

##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/resources/*.qrc)
file(GLOB_RECURSE QT_MOC_INFORM RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controller/include/behavior_inform_operator.h src/controller/include/inform_operator_widget.h)
file(GLOB_RECURSE QT_MOC_REQUEST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controller/include/behavior_request_operator_assistance.h src/controller/include/operator_assistance_widget.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP_INFORM ${QT_MOC_INFORM})
QT5_WRAP_CPP(QT_MOC_HPP_REQUEST ${QT_MOC_REQUEST})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES_INFORM RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/controller/source/behavior_inform_operator.cpp src/controller/source/inform_operator_widget.cpp)
file(GLOB_RECURSE QT_SOURCES_REQUEST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/controller/source/behavior_request_operator_assistance.cpp src/controller/source/operator_assistance_widget.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")

##############################################################################
# Binaries
##############################################################################

add_library(operator_interaction_lib ${QT_SOURCES_INFORM} ${QT_RESOURCES_CPP} ${QT_MOC_HPP_INFORM} ${QT_MOC_HPP_REQUEST} ${QT_FORMS_HPP})
add_dependencies(operator_interaction_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(operator_interaction_lib ${QWT_LIB} ${catkin_LIBRARIES} yaml-cpp)

qt5_use_modules(operator_interaction_lib Core Gui Svg OpenGL Widgets PrintSupport)

add_executable(behavior_inform_operator ${QT_SOURCES_INFORM} ${QT_RESOURCES_CPP} ${QT_MOC_HPP_INFORM} ${QT_FORMS_HPP})
target_link_libraries(behavior_inform_operator operator_interaction_lib)

add_executable(behavior_request_operator_assistance ${QT_SOURCES_REQUEST} ${QT_RESOURCES_CPP} ${QT_MOC_HPP_REQUEST} ${QT_FORMS_HPP})
target_link_libraries(behavior_request_operator_assistance operator_interaction_lib)







