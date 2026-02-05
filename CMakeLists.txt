cmake_minimum_required(VERSION 3.16)
project(apriltag3_cpp_refactor LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_library(apriltag3_cpp
    src/image.cpp
    src/union_find.cpp
    src/quad_detector.cpp
    src/decode_core.c
    src/decoder.cpp
    src/dedup.cpp
    src/apriltag_detector.cpp)

target_include_directories(apriltag3_cpp PUBLIC include)
target_link_libraries(apriltag3_cpp PUBLIC opencv_core opencv_imgcodecs)

add_executable(apriltag3_cpp_demo examples/demo_main.cpp)
target_link_libraries(apriltag3_cpp_demo PRIVATE apriltag3_cpp)

enable_testing()
add_test(NAME demo_no_args COMMAND apriltag3_cpp_demo)
set_tests_properties(demo_no_args PROPERTIES PASS_REGULAR_EXPRESSION "usage")
