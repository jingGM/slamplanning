cmake_minimum_required(VERSION 2.8.3)
project(stage_ros)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  message_generation
)


find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(Protobuf REQUIRED)

add_message_files(
    FILES
    RobotStatus.msg
    Transition.msg
    Transitions.msg
    State.msg
    States.msg
    Goal.msg
    GoalObs.msg
    Scan.msg
    ScanObs.msg
    Action.msg
    ActionObs.msg
    Actions.msg
    Velocity.msg
    Velocities.msg
    VelocityObs.msg
    Reward.msg
)

add_service_files(
    FILES
    UpdateStage.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /opt/ros/melodic/include/Stage-4.3
  #/usr/local/include/Stage-4.1
#  ${PROTOBUF_INCLUDE_DIRS}
)

catkin_package()

# Declare a cpp executable
add_executable(stageros src/stageros.cpp)

add_executable(drl_stageros src/drl_stageros.cpp)

target_link_libraries(stageros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  -L/usr/local/lib64 -lstage  -lfltk_images -lfltk_forms -lfltk_gl -lGL -lfltk -lSM -lICE -lX11 -lXext -lm -lGLU -lGL -lGLU -lGL
  ${${PROJECT_NAME}_extra_libs}
)

target_link_libraries(drl_stageros
 ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
#  ${PROTOBUF_LIBRARIES}
  -L/usr/local/lib64 -lstage  -lfltk_images -lfltk_forms -lfltk_gl -lGL -lfltk -lSM -lICE -lX11 -lXext -lm -lGLU -lGL -lGLU -lGL
  ${${PROJECT_NAME}_extra_libs}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(stageros ${catkin_EXPORTED_TARGETS})
endif()

if(catkin_EXPORTED_TARGETS)
  add_dependencies(drl_stageros ${catkin_EXPORTED_TARGETS})
endif()

## Install

install(PROGRAMS scripts/upgrade-world.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS stageros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS drl_stageros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/hztest.xml)
  add_rostest(test/intensity_test.xml)
endif()
