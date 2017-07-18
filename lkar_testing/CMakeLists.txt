cmake_minimum_required(VERSION 2.8.3)
project(lkar_testing)



set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
        rospy
        std_msgs
        move_base_msgs
        actionlib
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface

        cv_bridge
        pcl_conversions
        pcl_ros
        tf
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
)

#find_package(motion_analysis_msgs)

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy std_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIR} ${Boost_INCLUDE_DIR} src include)

add_executable(move_goal src/compiler_test.cpp)
target_link_libraries(move_goal ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(add_object src/add_object.cpp)
target_link_libraries(add_object ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(remove_object src/remove_object.cpp)
target_link_libraries(remove_object ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(check_collision src/check_collision.cpp)
target_link_libraries(check_collision ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(move_position_goal src/move_position_goal.cpp)
target_link_libraries(move_position_goal ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall" )