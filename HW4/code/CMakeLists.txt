cmake_minimum_required( VERSION 2.8 )
project( imageBasics )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable( undistorted undistort_image.cpp )
# 链接OpenCV库
target_link_libraries( undistorted ${OpenCV_LIBS} )

add_executable( disparity disparity.cpp )
# 链接库
target_link_libraries( disparity ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable( GN gaussnewton.cpp )
# 链接库
target_link_libraries( GN ${OpenCV_LIBS})