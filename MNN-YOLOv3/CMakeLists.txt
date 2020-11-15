cmake_minimum_required(VERSION 3.10)
FIND_PACKAGE(OpenCV REQUIRED)
project(loongson_cmd)

#set(CMAKE_CXX_STANDARD 17 "-lpthread")
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS} -pthread -std=c++11")

#find_package(OpenCV REQUIRED)
include_directories(
        mnn/include
        src
)

link_directories(mnn/lib)

add_executable(loongson_cmd src/main.cpp src/UltraFace.cpp src/mobilefacenet.cpp)
target_link_libraries(loongson_cmd MNN ${OpenCV_LIBS})
