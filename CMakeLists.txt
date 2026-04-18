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
target_include_directories(IPhysics PUBLIC 
    core/include
    core/include/components
)
target_link_libraries(IApp PUBLIC IPhysics)
target_include_directories(IApp PUBLIC
    apps/appcore/include
)
if(NOT EMSCRIPTEN)
    find_package(raylib REQUIRED)
endif()
target_link_libraries(IPlanetSimulator
    PRIVATE
        IPhysics
        IApp
)
if(NOT EMSCRIPTEN)
    target_link_libraries(IPlanetSimulator PRIVATE raylib)
endif()
if(EMSCRIPTEN)
    target_link_libraries(IPlanetSimulator PRIVATE
        /Users/cold/Libraries/raylib/build-web/raylib/libraylib.a
    )
    target_compile_options(IPlanetSimulator PRIVATE -w)
    target_include_directories(IPlanetSimulator PRIVATE
        /Users/cold/Libraries/emsdk/upstream/emscripten/cache/sysroot/include
        /Users/cold/Libraries/raylib/src
    )
    set_target_properties(IPlanetSimulator PROPERTIES
        SUFFIX ".html"
        LINK_FLAGS "-sUSE_GLFW=3 -sASYNCIFY -sALLOW_MEMORY_GROWTH=1 -sASSERTIONS=1 --preload-file ${CMAKE_SOURCE_DIR}/resources@resources --shell-file ${CMAKE_SOURCE_DIR}/apps/IPlanetSimulator/shell.html"
    )
endif()