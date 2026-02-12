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
        src/bvh_base.cpp
        src/bsdfs/diffuse.cpp
        src/bsdfs/mirror.cpp
        src/bsdfs/conductor.cpp
        src/bsdfs/dielectric.cpp
        src/bsdfs/microfacet.cpp
        src/bsdfs/bsdf.cpp
        src/bsdfs/orennayar.cpp
        src/camera.cpp
        src/image.cpp
        src/progress.cpp
        src/integrators/MCIntegrator.cpp
        src/integrators/depth.cpp
        src/integrators/hitpos.cpp
        src/integrators/normal.cpp
        src/integrators/uv.cpp
        src/integrators/path.cpp
        src/lights/area.cpp
        src/lights/point.cpp
        src/lights/imageEnvLight.cpp
        src/lights/constantEnvLight.cpp
        src/mesh_accel/naive.cpp
        src/mesh_accel/octree.cpp
        src/mesh_accel/bvh_mesh.cpp
        src/samplers/uniformstd.cpp
        src/scene_accel/bvh_scene.cpp
        src/scene.cpp
        src/shapes/objmesh.cpp
        src/shapes/plymesh.cpp
        src/shapes/shape.cpp
        src/shapes/triangle.cpp
        src/scene_parser.cpp
        src/rayintersectinfo.cpp
        src/textures/image_texture.cpp
        src/render.cpp
        )

set(HEADERS
        include/aabb.h
        include/mesh_accel.h
        include/bsdf.h
        include/camera.h
        include/common.h
        include/coordinate.h
        include/distribution.h
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
        include/scene_parser.h
        include/textures.h
        include/render.h)

if(NOT EXCLUDE_TEST)
    include_directories(test)

    add_executable(caramel_test
            ${SOURCES}
            ${HEADERS}

            test/catch_amalgamated.hpp
            test/catch_amalgamated.cpp
            test/complex_render_test.cpp
            test/simple_render_test.cpp
            test/unit_tests.cpp
            test/utils.cpp
    )

    target_compile_definitions(caramel_test PRIVATE
        TEST_SCENE_PATH="${CMAKE_SOURCE_DIR}/caramel-scenes/"
        ENABLE_PROGRESS=0
    )
    target_include_directories(caramel_test PUBLIC ../include)
endif()

add_executable(caramel
        main.cpp
        ${SOURCES}
        ${HEADERS}
        include/bvh_base.h)

target_compile_definitions(caramel PRIVATE ENABLE_PROGRESS=1)
