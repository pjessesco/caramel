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

TEST_CASE("render test"){
    SECTION("ajax"){
        std::filesystem::current_path();
        Image ref(std::string(TEST_SCENE_PATH) + "ajax/gt.exr");
        Image render = render_for_test(std::string(TEST_SCENE_PATH) + "ajax/scene.json");

        REQUIRE(mse(ref, render) <= Catch::Approx(0.00003));
        REQUIRE(avg(diff(ref, render)) <= Catch::Approx(0.00002));
    }

    SECTION("veach-mis"){
        Image ref(std::string(TEST_SCENE_PATH) + "veach_mis/gt_test.exr");
        Image render = render_for_test(std::string(TEST_SCENE_PATH) + "veach_mis/scene.json", 150, 100);

        REQUIRE(avg(diff(ref, render)) <= Catch::Approx(0.042));
    }

    SECTION("cbox"){
        Image ref(std::string(TEST_SCENE_PATH) + "cbox/gt_test.exr");
        Image render = render_for_test(std::string(TEST_SCENE_PATH) + "cbox/scene.json", 100, 75);

        // MSE range is high since we render with low spp number & resolution
        REQUIRE(mse(ref, render) <= Catch::Approx(0.008));
        REQUIRE(avg(diff(ref, render)) <= Catch::Approx(0.023));
    }

    SECTION("shaderballs"){
        Image ref(std::string(TEST_SCENE_PATH) + "shaderballs/gt_test.exr");
        Image render = render_for_test(std::string(TEST_SCENE_PATH) + "shaderballs/scene.json", 175, 100);

        // MSE range is high since we render with low spp number & resolution
        REQUIRE(mse(ref, render) <= Catch::Approx(0.0035));
        REQUIRE(avg(diff(ref, render)) <= Catch::Approx(0.028));
    }
}


