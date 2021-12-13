cmake_minimum_required(VERSION 3.5.1)
project(optiTrack_tf)

find_package(catkin REQUIRED COMPONENTS
 
 roscpp
 rospy
 std_msgs

)


SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SRCS RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

set(PRIVATE_COMPILE_OPTS )

catkin_package(
    LIBRARIES
        optiTrack_tf
    INCLUDE_DIRS
        include 
    CATKIN_DEPENDS roscpp rospy std_msgs  
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 
)

add_executable(optiTrack_tf ${SRCS})

add_dependencies(optiTrack_tf ${catkin_EXPORTED_TARGETS})
 add_dependencies(optiTrack_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_include_directories(
    optiTrack_tf PRIVATE 
    ${PROJECT_SOURCE_DIR/include}
)

 target_link_libraries(optiTrack_tf
   ${catkin_LIBRARIES}
         ${OMPL_LIBRARIES}
         ${PCL_LIBRARIES} 
 )
target_compile_options(optiTrack_tf PRIVATE ${PRIVATE_COMPILE_OPTS} "-fpermissive")