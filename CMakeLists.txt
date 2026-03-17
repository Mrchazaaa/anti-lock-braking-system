cmake_minimum_required(VERSION 3.16)

project(ABS LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ABS
    src/abs.cpp
    src/velocity_ekf.cpp
)

target_include_directories(ABS
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/TinyEKF/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
