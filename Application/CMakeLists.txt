cmake_minimum_required(VERSION 4.0)
project(Application LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE APPLICATION_SOURCE_FILES src/*.cpp src/*.h)
add_executable(Application ${APPLICATION_SOURCE_FILES}
        src/Events.cpp
        src/Events.h
        src/Physics/BroadPhase/BroadPhase.cpp
        src/Physics/BroadPhase/BroadPhase.h
        src/Physics/Shapes/AABB.cpp
        src/Physics/Shapes/AABB.h
        src/Physics/CollisionSystem.cpp
        src/Physics/CollisionSystem.h
        src/Colors.h)

target_link_libraries(Application PRIVATE CommonCore)
target_include_directories(Application PRIVATE src)

set(CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config/")
target_compile_definitions(Application PRIVATE CONFIG_PATH="${CONFIG_PATH}" NOMINMAX)