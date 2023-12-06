cmake_minimum_required(VERSION 3.0.2)

project(eigen_example) #The name of your choice
#for the project comes here
add_compile_options(-std=c++11) #opt. to use c++11

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS  roscpp)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS})

#The name of the cpp file and its path can vary
add_executable(${PROJECT_NAME}_node src/eigen_example_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
${Eigen_LIBRARIES})



