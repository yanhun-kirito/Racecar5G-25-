cmake_minimum_required(VERSION 2.8)
project(total)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(PROTO_PB_DIR ${PROJECT_SOURCE_DIR})
add_library(pid SHARED pid.cpp)
add_library(init SHARED init.cpp)


set(CMAKE_PREFIX_PATH /usr/local/lib/cmake/pigpio)
include_directories( ${PROTO_PB_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(/home/pi/Smartcar_5g-main)

add_executable(main demo.cpp )
target_link_libraries(main ${OpenCV_LIBS} /usr/lib/libpigpio.so pid init Threads::Threads)
set(pigpio_DIR /usr/lib/cmake/pigpio)

