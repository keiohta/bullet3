include_directories(
        ../src)

link_directories(
        ../../build_cmake/Extras/BulletRobotics)

link_libraries(
        BulletDynamics)

add_executable(
        CartPoleDemo
        main.cpp)