//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2026 Jino Park
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

// caramel headers
#include <common.h>
#include <ray.h>
#include <shape.h>
#include <image.h>
#include <render.h>
#include <scene.h>
#include <rayintersectinfo.h>

#include <utils.h>

// Dependencies headers
#include "catch_amalgamated.hpp"

#include <cmath>

using namespace Caramel;

TEST_CASE("test1 render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test1/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test1/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);

    if (SAVE_RENDERED_IMAGES) {
        rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test1_rendered.exr").string());
    }

    Image diff_image(1, 1);
    const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

    CHECK(flip_error_value <= Catch::Approx(0.004));
}

TEST_CASE("test2 render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test2/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test2/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);

    if (SAVE_RENDERED_IMAGES) {
        rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test2_rendered.exr").string());
    }

    Image diff_image(1, 1);
    const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

    CHECK(flip_error_value <= Catch::Approx(0.004));
}

TEST_CASE("test3 render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test3/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test3/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);

    if (SAVE_RENDERED_IMAGES) {
        rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test3_rendered.exr").string());
    }

    Image diff_image(1, 1);
    const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

    CHECK(flip_error_value <= Catch::Approx(0.054));
}

TEST_CASE("test4 render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test4/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test4/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);

    if (SAVE_RENDERED_IMAGES) {
        rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test4_rendered.exr").string());
    }

    Image diff_image(1, 1);
    const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

    CHECK(flip_error_value <= Catch::Approx(0.221));
}

TEST_CASE("test5 render test", "[RenderTest]") {
    SECTION("Conductor"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_conductor.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_conductor.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);

        if (SAVE_RENDERED_IMAGES) {
            rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test5_conductor_rendered.exr").string());
        }

        Image diff_image(1, 1);
        const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

        CHECK(flip_error_value <= Catch::Approx(0.058));
    }
    SECTION("Dielectric"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_dielectric.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_dielectric.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);

        if (SAVE_RENDERED_IMAGES) {
            rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test5_dielectric_rendered.exr").string());
        }

        Image diff_image(1, 1);
        const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

        CHECK(flip_error_value <= Catch::Approx(0.247));
    }
    SECTION("Diffuse"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_diffuse.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_diffuse.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);

        if (SAVE_RENDERED_IMAGES) {
            rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test5_diffuse_rendered.exr").string());
        }

        Image diff_image(1, 1);
        const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

        CHECK(flip_error_value <= Catch::Approx(0.046));
    }
    SECTION("Mirror"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_mirror.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_mirror.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);

        if (SAVE_RENDERED_IMAGES) {
            rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test5_mirror_rendered.exr").string());
        }

        Image diff_image(1, 1);
        const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

        CHECK(flip_error_value <= Catch::Approx(0.058));
    }
    SECTION("Microfacet"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_microfacet.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_microfacet.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);
        CHECK(flip_error(ref, rendered) <= Catch::Approx(0.053));
    }
    SECTION("OrenNayar"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_orennayar.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_orennayar.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);
        CHECK(flip_error(ref, rendered) <= Catch::Approx(0.047));
    }
    SECTION("TwoSided"){
        std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test5/scene_twosided.json";
        Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test5/gt_scene_twosided.exr");
        auto [_s, _i] = build_scene(scene_path);
        Image rendered = render(_s, _i);
        CHECK(flip_error(ref, rendered) <= Catch::Approx(0.049));
    }
}

TEST_CASE("test6 render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test6/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test6/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);

    if (SAVE_RENDERED_IMAGES) {
        rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test6_rendered.exr").string());
    }

    Image diff_image(1, 1);
    const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

    CHECK(flip_error_value <= Catch::Approx(0.035));
}

TEST_CASE("test7 render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/test7/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test7/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);

    if (SAVE_RENDERED_IMAGES) {
        rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "test7_rendered.exr").string());
    }

    Image diff_image(1, 1);
    const Float flip_error_value = flip_error(ref, rendered, true, &diff_image);

    CHECK(flip_error_value <= Catch::Approx(0.007));
}

TEST_CASE("furnace render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/furnace/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/furnace/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);
    CHECK(flip_error(ref, rendered) <= Catch::Approx(0.008));
}

TEST_CASE("thinlens render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/thinlens/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/thinlens/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);
    CHECK(flip_error(ref, rendered) <= Catch::Approx(0.038));
}

TEST_CASE("texture render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/texture/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/texture/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);
    CHECK(flip_error(ref, rendered) <= Catch::Approx(0.016));
}

TEST_CASE("ply render test", "[RenderTest]") {
    std::string scene_path = std::string(TEST_SCENE_PATH) + "test_scenes/ply/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/ply/gt.exr");
    auto [_s, _i] = build_scene(scene_path);
    Image rendered = render(_s, _i);
    CHECK(flip_error(ref, rendered) <= Catch::Approx(0.041));
}
