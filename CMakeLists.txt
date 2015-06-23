cmake_minimum_required(VERSION 2.8.3)
project(beaglebone)

find_package(catkin REQUIRED COMPONENTS 
	roscpp rospy rostest std_msgs tf tf2
	message_generation geometry_msgs sensor_msgs
)
find_package(Boost REQUIRED)


add_message_files(
  DIRECTORY
  msg
  FILES
  IMU.msg
  LRS.msg
  WheelVelocities.msg
  WheelDistances.msg
)

#add_service_files(
#  FILES
#)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#sensor and motor drivers
add_executable(MotionSensorDriver src/demconrobot_MotionSensorDriver.cpp)
add_library(Driver src/MotionSensorDriver.cpp src/CanDriver.cpp src/MPU6000IMU.cpp)
target_link_libraries(MotionSensorDriver Driver)
target_link_libraries(MotionSensorDriver ${catkin_LIBRARIES})
target_link_libraries(MotionSensorDriver ${Boost_LIBRARIES})
add_dependencies(MotionSensorDriver beaglebone_gencpp)
add_dependencies(Driver beaglebone_gencpp)

#laser scanner
add_library(xv11_laser_driver src/xv11_laser.cpp)
add_executable(neato_laser_publisher src/neato_laser_publisher.cpp)
target_link_libraries(neato_laser_publisher xv11_laser_driver)
target_link_libraries(neato_laser_publisher ${Boost_LIBRARIES})
target_link_libraries(neato_laser_publisher ${catkin_LIBRARIES})

#IMU
add_library(imu_driver src/MPU6000IMU.cpp)
add_executable(imu_publisher src/imu_publisher.cpp)
target_link_libraries(imu_publisher imu_driver)
target_link_libraries(imu_publisher ${catkin_LIBRARIES})
target_link_libraries(imu_publisher ${Boost_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beaglebone.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
