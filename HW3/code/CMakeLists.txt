cmake_minimum_required( VERSION 2.8 )
set(CMAKE_CXX_STANDARD 11)
project( draw_traj )

# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable( draw_traj draw_trajectory.cpp )
target_link_libraries( draw_traj ${Sophus_LIBRARIES} )
target_link_libraries( draw_traj ${Pangolin_LIBRARIES} )

add_executable( rmse rmse.cpp )
target_link_libraries( rmse ${Sophus_LIBRARIES} )
target_link_libraries( rmse ${Pangolin_LIBRARIES} )