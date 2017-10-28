cmake_minimum_required(VERSION 2.8.3)
project(a_start)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  rospy
)


find_package(OpenCV REQUIRED)

catkin_package(
)

set (untitled_HDRS
)

set (untitled_SRCS
    src/Path_planner/main.cpp
    src/Visualisation/mainwindow.cpp
    src/Path_planner/path.cpp
)

set (untitled_UIS
    ui/mainwindow.ui
)


set (untitled_MOCS
    include/a_start/mainwindow.h
    include/a_start/path.h
)

find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
add_definitions( ${QT_DEFINITIONS} )
include_directories (
    ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}
)


QT4_WRAP_UI(UIS ${untitled_UIS})
QT4_WRAP_CPP(MOCS ${untitled_MOCS})
QT4_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(a_star
  src/Path_planner/main.cpp
  src/Path_planner/path.cpp

  include/a_start/path.h

  include/a_start/nodegen.h



)

add_executable(node_generation
  src/Node_gen/main_node.cpp
  src/Node_gen/nodegen.cpp

  include/a_start/nodegen.h

)

add_executable(visualiser

  src/Visualisation/visual.cpp

  include/a_start/mainwindow.h

  ${SOURCES} ${MOC_SRCS} ${RES_SOURCES} ${UI_HEADERS}

)

add_executable(main_path

  src/Main_path/main_path_in.cpp
  src/Main_path/main_path.cpp

  include/a_start/main_path.h

)

target_link_libraries(a_star ${OpenCV_LIBS}  ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(a_star ${catkin_EXPORTED_TARGETS})


target_link_libraries(node_generation ${OpenCV_LIBS})


add_library( visualiser_ SHARED ${untitled_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} ${RES_SOURCES})
target_link_libraries(visualiser_ ${QT_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries(visualiser visualiser_)

target_link_libraries(main_path ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_definitions(-std=c++11)

