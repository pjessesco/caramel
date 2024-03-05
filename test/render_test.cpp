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

#include <utils.h>

// Dependencies headers
#include "catch_amalgamated.hpp"

using namespace Caramel;

TEST_CASE("ajax render test") {
    std::filesystem::current_path();
    Image ref(std::string(TEST_SCENE_PATH) + "ajax/gt.exr");
    Image render = render_for_test(std::string(TEST_SCENE_PATH) + "ajax/scene.json");

    CHECK(mse(ref, render) <= Catch::Approx(0.00003));
    CHECK(avg(diff(ref, render)) <= Catch::Approx(0.00002));
}

TEST_CASE("veach-mis render test"){
    Image ref(std::string(TEST_SCENE_PATH) + "veach_mis/gt.exr");
    Image render = render_for_test(std::string(TEST_SCENE_PATH) + "veach_mis/scene.json", 768,512);
    CHECK(avg(diff(ref, render)) <= Catch::Approx(0.041));
}

TEST_CASE("cbox render test"){
    Image ref(std::string(TEST_SCENE_PATH) + "cbox/gt.exr");
    Image render = render_for_test(std::string(TEST_SCENE_PATH) + "cbox/scene.json", 800, 600);

    CHECK(mse(ref, render) <= Catch::Approx(0.0416));
    CHECK(avg(diff(ref, render)) <= Catch::Approx(0.122));
}

TEST_CASE("shaderballs render test"){
    Image ref(std::string(TEST_SCENE_PATH) + "shaderballs/gt.exr");
    Image render = render_for_test(std::string(TEST_SCENE_PATH) + "shaderballs/scene.json", 1400, 800);
    
    CHECK(mse(ref, render) <= Catch::Approx(0.015));
    CHECK(avg(diff(ref, render)) <= Catch::Approx(0.045));
}



