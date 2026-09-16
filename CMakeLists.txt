cmake_minimum_required(VERSION 3.26)

option(IPHYSICS_USE_CUDA "Enable CUDA gravity backends" ON)

if(IPHYSICS_USE_CUDA)
        project(IPhysics LANGUAGES CXX CUDA)
else()
        project(IPhysics LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(IPHYSICS_SOURCES
        core/source/core.cpp
        core/source/object.cpp
        core/source/world.cpp
        core/source/forcegenerator.cpp
        core/source/contacts.cpp
        core/source/collidebroad.cpp
        core/source/collidenarrow.cpp

        core/source/components/geometry.cpp
        core/source/components/primitive.cpp
        core/source/components/rigidbody.cpp
        core/source/components/information.cpp

        core/source/forcegenerators/spring.cpp
        core/source/forcegenerators/downwardgravity.cpp
        core/source/forcegenerators/gravity.cpp
        core/source/forcegenerators/barneshutgravity.cpp
)

if(IPHYSICS_USE_CUDA)
    list(APPEND IPHYSICS_SOURCES
        core/source/forcegenerators/cudagravity.cu
        core/source/forcegenerators/cudabarneshutgravity.cu
    )
endif()

add_library(IPhysics STATIC ${IPHYSICS_SOURCES})

target_include_directories(IPhysics PUBLIC
        core/include
        core/include/components
        core/include/forcegenerators
)

set(IAPP_SOURCES
        apps/AppCore/source/meshmanager.cpp
        apps/AppCore/source/errormanager.cpp
        apps/AppCore/source/rayguihelper.cpp

)

if(IPHYSICS_USE_CUDA)
    list(APPEND IAPP_SOURCES
        apps/AppCore/source/gpuinformation.cu
    )
endif()

add_library(IApp STATIC ${IAPP_SOURCES})

target_link_libraries(IApp PUBLIC IPhysics)

target_include_directories(IApp PUBLIC
        apps/AppCore/include
)

add_executable(IPlanetSimulator
       apps/IPlanetSimulator/main.cpp
)

target_link_libraries(IPlanetSimulator
        PRIVATE
        IPhysics
        IApp
)

set(IGRAVITY_SOURCES
        apps/IGravity/source/main.cpp
        apps/IGravity/source/igravity.cpp
        apps/IGravity/source/igravitymodel.cpp
        apps/IGravity/source/igravityview.cpp
        apps/IGravity/source/igravitycontroller.cpp
)

add_executable(IGravity ${IGRAVITY_SOURCES})

target_include_directories(IGravity PUBLIC
        apps/IGravity/include
)

target_link_libraries(IGravity
        PRIVATE
        IPhysics
        IApp
)

if(NOT EMSCRIPTEN)
        find_package(raylib REQUIRED)

        target_link_libraries(IPlanetSimulator PRIVATE raylib)
        target_link_libraries(IGravity PRIVATE raylib)
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