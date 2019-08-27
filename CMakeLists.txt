cmake_minimum_required(VERSION 2.8.3)

project(turtlebot_6wd)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf
    nav_msgs 
    lino_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

#catkin_package()
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES linorobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs lino_msgs geometry_msgs sensor_msgs std_msgs 
  #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(base_node src/base_node.cpp src/PID.cpp)
target_link_libraries(base_node ${catkin_LIBRARIES})

#add_executable(imu_vel src/imu_vel.cpp)
#target_link_libraries(imu_vel ${catkin_LIBRARIES})
add_executable(raw_vel src/raw_vel.cpp)
target_link_libraries(raw_vel ${catkin_LIBRARIES})

#add_executable(mpu925x_node src/mpu925x_node.cpp)
#target_link_libraries(mpu925x_node ${catkin_LIBRARIES})
#target_link_libraries (mpu925x_node wiringPi)
