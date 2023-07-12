cmake_minimum_required(VERSION 3.22)
project(caramel)

set(CMAKE_CXX_STANDARD 20)

# Extern libs
add_subdirectory(ext/peanut)
include_directories(ext)
include_directories(ext/peanut/include)

include_directories(include)

set(SOURCES
        src/aabb.cpp
        src/bsdfs/diffuse.cpp
        src/bsdfs/mirror.cpp
        src/bsdfs/dielectric.cpp
        src/bsdfs/microfacet.cpp
        src/bsdfs/bsdf.cpp
        src/bsdfs/orennayar.cpp
        src/camera.cpp
        src/image.cpp
        src/integrators/depth.cpp
        src/integrators/direct.cpp
        src/integrators/hitpos.cpp
        src/integrators/integrators.cpp
        src/integrators/normal.cpp
        src/integrators/uv.cpp
        src/integrators/path.cpp
        src/lights/area.cpp
        src/mesh_accel/naive.cpp
        src/mesh_accel/octree.cpp
        src/samplers/uniformstd.cpp
        src/scene.cpp
        src/shapes/objmesh.cpp
        src/shapes/shape.cpp
        src/shapes/triangle.cpp
        src/scene_parser.cpp
        src/rayintersectinfo.cpp
        )

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
        include/warp_sample.h
        include/scene_parser.h)

if(NOT EXCLUDE_TEST)
    add_executable(caramel_test
            ${SOURCES}
            ${HEADERS}

            test/catch_amalgamated.hpp
            test/catch_amalgamated.cpp

            test/test_ray_intersection.cpp
            )

    target_include_directories(caramel_test PUBLIC ../include)
endif()

add_executable(caramel
        main.cpp
        ${SOURCES}
        ${HEADERS})

