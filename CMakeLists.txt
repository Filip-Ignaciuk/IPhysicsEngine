cmake_minimum_required(VERSION 3.20)

project(IPhysicsEngineCore)

set(CMAKE_BUILD_TYPE Release)
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
    core/source/components/information.cpp
)

add_library(IApp STATIC
    apps/appcore/source/meshmanager.cpp
    apps/appcore/source/languagemanager.cpp
    apps/appcore/source/errormanager.cpp
)

add_executable(IPlanetSimulator 
    apps/IPlanetSimulator/main.cpp
)

include_directories(IPhysics PRIVATE core/include)
include_directories(IPhysics PRIVATE core/include/components)
include_directories(IApp PRIVATE apps/appcore/include)


find_package(raylib REQUIRED)

if(UNIX AND NOT APPLE)
target_link_libraries(IPlanetSimulator
    PRIVATE
        raylib
        IPhysics
        IApp
)
endif()

if(APPLE)
target_link_libraries(IPlanetSimulator
    PRIVATE
        raylib
        IPhysics
        IApp
)
endif()

if(EMSCRIPTEN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
        -s USE_GLFW=3 \
        -s ASYNCIFY \
        -s ALLOW_MEMORY_GROWTH=1 \
        --preload-file resources \
        --shell-file shell.html")
    
    set_target_properties(IPlanetSimulator PROPERTIES 
        SUFFIX ".html"
    )
    
    target_link_libraries(IPlanetSimulator
        PRIVATE
            IPhysics
            IApp
            raylib
    )
endif()