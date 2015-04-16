cmake_minimum_required(VERSION 2.8)
project( Example_box )
find_package( OpenCV REQUIRED)
add_executable(sim3 main.cpp Sim3dSolver.cpp )
target_link_libraries(sim3 ${OpenCV_LIBS} )



 
