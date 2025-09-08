cmake_minimum_required(VERSION 3.20)

project(IPhysicsEngine)

include_directories(IphysicsEngine PRIVATE include)

add_executable(IPhysicsEngine ../source/main.cpp ../source/core.cpp ../source/particle.cpp)