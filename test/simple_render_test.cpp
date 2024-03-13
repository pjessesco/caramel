//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
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

TEST_CASE("test1 render test") {
    std::filesystem::current_path();
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test1/gt.exr");
    Image rendered = render(std::string(TEST_SCENE_PATH) + "test_scenes/test1/scene.json");

    CHECK(avg(rendered)/avg(ref) <= Catch::Approx(1.0005));
    CHECK(Catch::Approx(0.9995) <= avg(rendered)/avg(ref));
}

TEST_CASE("test2 render test") {
    std::filesystem::current_path();
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test2/gt.exr");
    Image rendered = render(std::string(TEST_SCENE_PATH) + "test_scenes/test2/scene.json");

    CHECK(avg(rendered)/avg(ref) <= Catch::Approx(1.0005));
    CHECK(Catch::Approx(0.9995) <= avg(rendered)/avg(ref));
}

TEST_CASE("test3 render test") {
    std::filesystem::current_path();
    Image ref(std::string(TEST_SCENE_PATH) + "test_scenes/test3/gt.exr");
    Image rendered = render(std::string(TEST_SCENE_PATH) + "test_scenes/test3/scene.json");

    CHECK(avg(rendered)/avg(ref) <= Catch::Approx(1.0005));
    CHECK(Catch::Approx(0.9995) <= avg(rendered)/avg(ref));
}







