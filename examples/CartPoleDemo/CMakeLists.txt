cmake_minimum_required(VERSION 3.10)

project(CartPoleDemo)

include_directories(
        ${CMAKE_SOURCE_DIR}/../../src)

link_directories(
        ${CMAKE_SOURCE_DIR}/../../build_cmake/Extras/BulletRobotics
)
add_executable(
        ${PROJECT_NAME}
        main.cpp
)
target_link_libraries(
        ${PROJECT_NAME}
        BulletRobotics
)