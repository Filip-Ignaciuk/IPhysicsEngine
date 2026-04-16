cmake_minimum_required(VERSION 3.20)

project(IPhysicsEngineCore)
project(IPlanetSimulator)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(IPhysics STATIC
    core/source/core.cpp
    core/source/object.cpp  
    core/source/world.cpp
    core/source/forcegenerator.cpp
    core/source/contacts.cpp
    core/source/collidebroad.cpp
    core/source/collidenarrow.cpp
    core/source/components/component.cpp
    core/source/components/geometry.cpp
    core/source/components/primitive.cpp
    core/source/components/rigidbody.cpp
)

add_executable(IPlanetSimulator 
    apps/IPlanetSimulator/main.cpp
    apps/IPlanetSimulator/meshmanager.cpp
    apps/IPlanetSimulator/languagemanager.cpp
    
)

include_directories(IPhysics PRIVATE core/include)
include_directories(IPhysics PRIVATE core/include/components)

find_package(raylib REQUIRED)

if(UNIX AND NOT APPLE)
target_link_libraries(IPlanetSimulator
    PRIVATE
        raylib
        IPhysics
)
endif()

if(APPLE)
target_link_libraries(IPlanetSimulator
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
        IPhysics
)
endif()

