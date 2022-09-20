cmake_minimum_required(VERSION 3.22)
project(caramel)

set(CMAKE_CXX_STANDARD 20)

# Extern libs
add_subdirectory(ext/peanut)
include_directories(ext)
include_directories(ext/peanut/include)

include_directories(include)

set(SOURCES
        main.cpp
        src/aabb.cpp
        src/acceleration.cpp
        src/camera.cpp
        src/image.cpp
        src/integrators.cpp
        src/light.cpp
        src/scene.cpp
        src/shape.cpp)

set(HEADERS
        include/aabb.h
        include/acceleration.h
        include/bsdf.h
        include/camera.h
        include/common.h
        include/coordinate.h
        include/distrib1D.h
        include/image.h
        include/integrators.h
        include/light.h
        include/logger.h
        include/ray.h
        include/rayintersectinfo.h
        include/scene.h
        include/shape.h
        include/transform.h
        include/warp_sample.h)

add_executable(caramel
        ${SOURCES}
        ${HEADERS})

