cmake_minimum_required(VERSION 4.0)
project(Application LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE APPLICATION_SOURCE_FILES src/*.cpp src/*.h)
add_executable(Application ${APPLICATION_SOURCE_FILES})

target_link_libraries(Application PRIVATE CommonCore)
target_include_directories(Application PRIVATE src)