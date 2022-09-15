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
        src/image.cpp
        src/shape.cpp
        src/acceleration.cpp
        src/aabb.cpp
        src/integrators.cpp
        src/camera.cpp)

set(HEADERS
        include/aabb.h
        include/acceleration.h
        include/camera.h
        include/common.h
        include/image.h
        include/integrators.h
        include/logger.h
        include/ray.h
        include/rayintersectinfo.h
        include/scene.h
        include/shape.h
        include/transform.h)

add_executable(caramel
        ${SOURCES}
        ${HEADERS})

