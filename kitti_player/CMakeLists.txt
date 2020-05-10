cmake_minimum_required(VERSION 2.8.3)
project(kitti_player)

SET (CMAKE_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall")
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    tf
                    tf2
                    std_msgs                    
                    geometry_msgs
                    cv_bridge
                    image_transport
                    dynamic_reconfigure
)

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system program_options)


generate_dynamic_reconfigure_options(cfg/kitti_player.cfg)
catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(
		${}
  		${catkin_INCLUDE_DIRS} 
  		${PCL_INCLUDE_DIRS}
                ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kitti_player src/kitti_player.cpp)

target_link_libraries(kitti_player ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(kitti_player ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
