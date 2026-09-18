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
else()
    list(APPEND IAPP_SOURCES
        apps/AppCore/source/gpuinformation.cpp
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

if(IPHYSICS_USE_CUDA)
    target_compile_definitions(IGravity PRIVATE IPHYSICS_USE_CUDA)
endif()

target_include_directories(IGravity PUBLIC
        apps/IGravity/include
)

target_link_libraries(IGravity
        PRIVATE
        IPhysics
        IApp
)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 6.0
)
FetchContent_MakeAvailable(raylib)

target_link_libraries(IPhysics PRIVATE raylib)
target_link_libraries(IApp PRIVATE raylib)
target_link_libraries(IPlanetSimulator PRIVATE raylib)
target_link_libraries(IGravity PRIVATE raylib)

if(MSVC)
    target_compile_options(IPhysics PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>
        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>
    )
endif()