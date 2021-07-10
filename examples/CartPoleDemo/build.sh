#!/bin/sh

g++ main.cpp -I ../../src -L ../../build_cmake/Extras/BulletRobotics -l BulletRobotics

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../../build_cmake/Extras/BulletRobotics

./a.out