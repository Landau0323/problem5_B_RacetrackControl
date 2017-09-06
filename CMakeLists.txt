cmake_minimum_required(VERSION 3.8)
project(problem5_B_RacetrackControl)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE_FILES main.cpp parameter.h Particle.cpp Particle.h)
add_executable(problem5_B_RacetrackControl ${SOURCE_FILES})