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



TEST_CASE("ajax render test", "[RenderTest]") {
    TEST_BODY(ajax, 0.009)
}

TEST_CASE("veach-mis render test", "[RenderTest]"){
    TEST_BODY(veach_mis, 0.0005)
}

TEST_CASE("cbox render test", "[RenderTest]"){
    TEST_BODY(cbox, 0.0005)
}

TEST_CASE("diamond render test", "[RenderTest]"){
    TEST_BODY(diamonds, 0.006)
}

TEST_CASE("shaderballs render test", "[RenderTest]"){
    TEST_BODY(shaderballs, 0.005)
}

TEST_CASE("attic render test", "[RenderTest]"){
    TEST_BODY(attic, 0.005)
}

TEST_CASE("dragon render test", "[RenderTest]"){
    TEST_BODY(dragon, 0.005)
}

TEST_CASE("house render test", "[RenderTest]"){
    TEST_BODY(house, 0.005)
}

TEST_CASE("lego render test", "[RenderTest]"){
    TEST_BODY(lego, 0.005)
}

TEST_CASE("shaderball render test", "[RenderTest]"){
    TEST_BODY(shaderball, 0.005)
}

TEST_CASE("stormtrooper render test", "[RenderTest]"){
    TEST_BODY(stormtrooper, 0.005)
}

TEST_CASE("veach_door render test", "[RenderTest]"){
    TEST_BODY(veach_door, 0.005)
}






