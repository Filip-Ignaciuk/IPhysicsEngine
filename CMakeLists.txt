cmake_minimum_required(VERSION 3.20)

project(IPhysicsEngine)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(IPhysicsEngine 
    ../source/main.cpp 
    ../source/core.cpp 
    ../source/components/component.cpp
    ../source/object.cpp
    ../source/components/geometry.cpp
    ../source/meshmanager.cpp
    ../source/languagemanager.cpp
    ../source/collidebroad.cpp
    ../source/collidenarrow.cpp
    ../source/components/primitive.cpp
    ../source/contacts.cpp
    ../source/collidenarrow.cpp
    ../source/world.cpp
    ../source/components/rigidbody.cpp
    ../source/forcegenerator.cpp
    )

include_directories(IPhysicsEngine PRIVATE include)
include_directories(IPhysicsEngine PRIVATE components)

find_package(raylib REQUIRED)

if(UNIX AND NOT APPLE)
target_link_libraries(IPhysicsEngine
    PRIVATE
        raylib
)
endif()

if(APPLE)
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
endif()

