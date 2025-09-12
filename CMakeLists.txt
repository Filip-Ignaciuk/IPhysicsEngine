cmake_minimum_required(VERSION 3.20)

project(IPhysicsEngine)

add_executable(IPhysicsEngine ../source/main.cpp ../source/core.cpp ../source/particle.cpp ../source/ballistic.cpp ../source/fireworks.cpp)

include_directories(IPhysicsEngine PRIVATE include)

find_package(raylib REQUIRED)

target_link_libraries(IPhysicsEngine
    PRIVATE
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreFoundation"
        "-framework CoreGraphics"
        "-framework OpenGL"
        "-framework Carbon"
        "-framework Foundation"
        "-framework AppKit"
        raylib
)