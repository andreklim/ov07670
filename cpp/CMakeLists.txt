cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
find_package (Threads)
add_executable( DisplayImage DisplayImage.cpp )
add_executable( client client.cpp)
add_executable( server server.cpp)

target_link_libraries( DisplayImage ${OpenCV_LIBS} )
target_link_libraries( client ${OpenCV_LIBS} )
target_link_libraries( server ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} )

