cmake_minimum_required(VERSION 3.0.2)
project(momav)

add_compile_options(-std=c++17)
add_compile_options(-Wno-psabi)

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")


# catkin_simple
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# YAML
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# core (rapsberry pi, arm)
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "armv7l" OR ${ARCHITECTURE} STREQUAL "aarch64") # raspberry pi (ARM 32bit or 64bit)
	
	# commander
	cs_add_executable(commander src/commander.cpp)

	# controller_navigation
	cs_add_executable(controller_navigation src/controller_navigation.cpp)
	target_link_libraries(controller_navigation Eigen3::Eigen)

	# controller_pose
	cs_add_executable(controller_pose src/controller_pose.cpp)
	target_link_libraries(controller_pose Eigen3::Eigen)

	# controller_wrench
	cs_add_executable(controller_wrench src/controller_wrench.cpp)
	target_link_libraries(controller_wrench Eigen3::Eigen ${YAML_CPP_LIBRARIES})

	# driver_actuator
	cs_add_executable(driver_actuator src/driver_actuator.cpp)
	target_link_libraries(driver_actuator i2c)

	# driver_cam
	find_package(realsense2 REQUIRED)
	cs_add_executable(driver_cam src/driver_cam.cpp)
	target_include_directories(driver_cam PUBLIC ${realsense_INCLUDE_DIR})
	target_link_libraries(driver_cam realsense2 Eigen3::Eigen ${YAML_CPP_LIBRARIES})

	# driver_sim
	cs_add_executable(driver_sim src/driver_sim.cpp)
	target_link_libraries(driver_sim Eigen3::Eigen)


# ground (laptop, x86_64)
else()

	# monitor
	cs_install_scripts(src/monitor.py)

	# driver track
	cs_add_executable(driver_track src/driver_track.cpp)
	target_include_directories(driver_track PUBLIC /usr/include/natnet)
	target_link_libraries(driver_track Eigen3::Eigen /usr/lib/libNatNet.so)

	# driver remote
	cs_add_executable(driver_remote src/driver_remote.cpp)


endif()

cs_install()
cs_export()