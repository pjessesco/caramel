//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2025 Jino Park
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

#include <utils.h>

// Dependencies headers
#include "catch_amalgamated.hpp"

using namespace Caramel;

#define SAVE_RENDERED_IMAGES

TEST_CASE("ajax render test", "[RenderTest]") {
    std::filesystem::current_path();
    std::string scene_path = std::string(TEST_SCENE_PATH) + "ajax/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "ajax/gt.exr");
    Image rendered = render(scene_path);

#ifdef SAVE_RENDERED_IMAGES
    rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "ajax_rendered.exr").string());
#endif

    CHECK(avg(rendered) / avg(ref) <= Catch::Approx(1.005));
    CHECK(Catch::Approx(0.9991) <= avg(rendered) / avg(ref));
}

TEST_CASE("veach-mis render test", "[RenderTest]"){
    std::string scene_path = std::string(TEST_SCENE_PATH) + "veach_mis/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "veach_mis/gt.exr");
    Image rendered = render(scene_path);
    
#ifdef SAVE_RENDERED_IMAGES
    rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "veach_mis_rendered.exr").string());
#endif

    CHECK(avg(rendered) / avg(ref) <= Catch::Approx(1.005));
    CHECK(Catch::Approx(0.9995) <= avg(rendered) / avg(ref));
}

TEST_CASE("cbox render test", "[RenderTest]"){
    std::string scene_path = std::string(TEST_SCENE_PATH) + "cbox/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "cbox/gt.exr");
    Image rendered = render(scene_path);

#ifdef SAVE_RENDERED_IMAGES
    rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "cbox_rendered.exr").string());
#endif

    CHECK(avg(rendered) / avg(ref) <= Catch::Approx(1.005));
    CHECK(Catch::Approx(0.9995) <= avg(rendered) / avg(ref));
}

TEST_CASE("diamond render test", "[RenderTest]"){
    std::string scene_path = std::string(TEST_SCENE_PATH) + "diamonds/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "diamonds/gt.exr");
    Image rendered = render(scene_path);

#ifdef SAVE_RENDERED_IMAGES
    rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "diamonds_rendered.exr").string());
#endif

    CHECK(avg(rendered) / avg(ref) <= Catch::Approx(1.006));
    CHECK(Catch::Approx(0.994) <= avg(rendered) / avg(ref));
}

TEST_CASE("shaderballs render test", "[RenderTest]"){
    std::string scene_path = std::string(TEST_SCENE_PATH) + "shaderballs/scene.json";
    Image ref(std::string(TEST_SCENE_PATH) + "shaderballs/gt.exr");
    Image rendered = render(scene_path);

#ifdef SAVE_RENDERED_IMAGES
    rendered.write_exr((std::filesystem::path(scene_path).parent_path() / "shaderballs_rendered.exr").string());
#endif

    CHECK(avg(rendered) / avg(ref) <= Catch::Approx(1.05));
    CHECK(Catch::Approx(0.995) <= avg(rendered) / avg(ref));
}






