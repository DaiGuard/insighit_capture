cmake_minimum_required(VERSION 3.22)
project(insight_capture
    VERSION 0.1.0
    DESCRIPTION "insight camera capture library"
    LANGUAGES CXX
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(
    src
)

link_directories(
    build
)

add_library(${PROJECT_NAME} SHARED
    src/insight_capture.cpp
)

add_executable(test_lib
    src/test_lib.cpp
)

target_link_libraries(test_lib 
    ${PROJECT_NAME}
)

