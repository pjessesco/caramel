//
// This software is released under the MIT license.
//
// Copyright (c) 2022 Jino Park
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

// Standard headers
#include <type_traits>

// caramel headers
#include <common.h>
#include <ray.h>
#include <shape.h>

// Dependencies headers
#include "catch_amalgamated.hpp"

using namespace Caramel;
#define FLT(x) static_cast<Float>(x)

TEST_CASE("moeller_trumbore()"){
    /* Below test is generated using :
     *
     *  std::cout<<"-------------------\n";
     *  std::cout<<"Ray r{"
     *               "{FLT("<<ray.m_o[0]<<"), FLT("<<ray.m_o[1]<<"), FLT("<<ray.m_o[2]<<")}, "
     *               "{FLT("<<ray.m_d[0]<<"), FLT("<<ray.m_d[1]<<"), FLT("<<ray.m_d[2]<<")}};"<<std::endl;
     *  std::cout<<"Vector3f p0 = {FLT("<<p0[0]<<"), FLT("<<p0[1]<<"), FLT("<<p0[2]<<")};"<<std::endl;
     *  std::cout<<"Vector3f p1 = {FLT("<<p1[0]<<"), FLT("<<p1[1]<<"), FLT("<<p1[2]<<")};"<<std::endl;
     *  std::cout<<"Vector3f p2 = {FLT("<<p2[0]<<"), FLT("<<p2[1]<<"), FLT("<<p2[2]<<")};"<<std::endl;
     *  std::cout<<"auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);"<<std::endl;
     *  std::cout<<"CHECK(u == Catch::Approx(FLT("<<u<<")));"<<std::endl;
     *  std::cout<<"CHECK(v == Catch::Approx(FLT("<<v<<")));"<<std::endl;
     *  std::cout<<"CHECK(t == Catch::Approx(FLT("<<t<<")));"<<std::endl;
     */

    SECTION("Success"){
        {
            Ray r{{FLT(-0.868556), FLT(0), FLT(0.134)}, {FLT(0.37142), FLT(0.92271), FLT(-0.103212)}};
            Vector3f p0 = {FLT(-0.24), FLT(1.58), FLT(-0.22)};
            Vector3f p1 = {FLT(0.23), FLT(1.58), FLT(-0.22)};
            Vector3f p2 = {FLT(-0.24), FLT(1.58), FLT(0.16)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == Catch::Approx(FLT(0.015839)).epsilon(1e-4));
            CHECK(v == Catch::Approx(FLT(0.466487)));
            CHECK(t == Catch::Approx(FLT(1.71235)));
        }
        {
            Ray r{{FLT(-1.00725), FLT(0.732564), FLT(-0.692236)}, {FLT(0.699483), FLT(0.586623), FLT(0.408163)}};
            Vector3f p0 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p1 = {FLT(1), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.02), FLT(1.59), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == Catch::Approx(FLT(0.512451)));
            CHECK(v == Catch::Approx(FLT(0.4652)));
            CHECK(t == Catch::Approx(FLT(1.46165)));
        }
        {
            Ray r{{FLT(0), FLT(0.919769), FLT(5.41159)}, {FLT(-0.162241), FLT(0.0839264), FLT(-0.983176)}};
            Vector3f p0 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p1 = {FLT(-1.02), FLT(1.59), FLT(0.99)};
            Vector3f p2 = {FLT(-1.01), FLT(0), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == Catch::Approx(FLT(0.0459288)).epsilon(1e-4));
            CHECK(v == Catch::Approx(FLT(0.0899728)));
            CHECK(t == Catch::Approx(FLT(6.28139)));
        }
        {
            Ray r{{FLT(-0.995326), FLT(0.0600852), FLT(-0.614505)}, {FLT(0.47721), FLT(0.807191), FLT(0.347438)}};
            Vector3f p0 = {FLT(-0.24), FLT(1.58), FLT(-0.22)};
            Vector3f p1 = {FLT(0.23), FLT(1.58), FLT(-0.22)};
            Vector3f p2 = {FLT(-0.24), FLT(1.58), FLT(0.16)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == Catch::Approx(FLT(0.304777)));
            CHECK(v == Catch::Approx(FLT(0.683443)));
            CHECK(t == Catch::Approx(FLT(1.88297)));
        }
        {
            Ray r{{FLT(-1.00174), FLT(0.410517), FLT(-0.634689)}, {FLT(0.661682), FLT(0.658302), FLT(0.358909)}};
            Vector3f p0 = {FLT(0.23), FLT(1.58), FLT(-0.22)};
            Vector3f p1 = {FLT(0.23), FLT(1.58), FLT(0.16)};
            Vector3f p2 = {FLT(-0.24), FLT(1.58), FLT(0.16)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == Catch::Approx(FLT(0.466941)));
            CHECK(v == Catch::Approx(FLT(0.119686)));
            CHECK(t == Catch::Approx(FLT(1.77651)));
        }
    }

    SECTION("Fail : (v < Float0 || Float1 < v)"){
        {
            Ray r{{FLT(-1.00214), FLT(0.184094), FLT(-0.159966)}, {FLT(0.595905), FLT(0.802894), FLT(-0.0160882)}};
            Vector3f p0 = {FLT(-0.561924), FLT(0.601247), FLT(-0.164723)};
            Vector3f p1 = {FLT(-0.593885), FLT(0.582185), FLT(-0.164723)};
            Vector3f p2 = {FLT(-0.581792), FLT(0.582185), FLT(-0.148418)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.00268), FLT(0.219641), FLT(-0.173887)}, {FLT(0.570599), FLT(0.816619), FLT(0.0868872)}};
            Vector3f p0 = {FLT(-0.603045), FLT(0.601247), FLT(-0.279646)};
            Vector3f p1 = {FLT(-0.602172), FLT(0.601247), FLT(-0.297403)};
            Vector3f p2 = {FLT(-0.62775), FLT(0.582185), FLT(-0.299922)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.00312), FLT(0.238767), FLT(-0.165289)}, {FLT(0.66456), FLT(0.742683), FLT(0.0823544)}};
            Vector3f p0 = {FLT(-0.631466), FLT(0.511284), FLT(-0.107652)};
            Vector3f p1 = {FLT(-0.647315), FLT(0.511284), FLT(-0.129022)};
            Vector3f p2 = {FLT(-0.660994), FLT(0.483838), FLT(-0.119882)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.0032), FLT(0.250951), FLT(-0.180868)}, {FLT(0.640879), FLT(0.765098), FLT(0.0624344)}};
            Vector3f p0 = {FLT(-0.631466), FLT(0.536986), FLT(-0.139612)};
            Vector3f p1 = {FLT(-0.660994), FLT(0.511284), FLT(-0.151843)};
            Vector3f p2 = {FLT(-0.647315), FLT(0.511284), FLT(-0.129022)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
    }

    SECTION("Fail : (u < Float0 || Float1 < u + v)"){
        {
            Ray r{{FLT(-1.01782), FLT(1.24386), FLT(0.787702)}, {FLT(0.750732), FLT(0.237113), FLT(-0.616587)}};
            Vector3f p0 = {FLT(-0.99), FLT(0), FLT(-1.04)};
            Vector3f p1 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.01), FLT(0), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.0114), FLT(0.223327), FLT(0.819529)}, {FLT(0.59481), FLT(0.650827), FLT(-0.471832)}};
            Vector3f p0 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p1 = {FLT(1), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.02), FLT(1.59), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.01365), FLT(0.580243), FLT(0.813654)}, {FLT(0.665883), FLT(0.602144), FLT(-0.44048)}};
            Vector3f p0 = {FLT(-0.24), FLT(1.58), FLT(-0.22)};
            Vector3f p1 = {FLT(0.23), FLT(1.58), FLT(-0.22)};
            Vector3f p2 = {FLT(-0.24), FLT(1.58), FLT(0.16)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.01689), FLT(1.09484), FLT(0.79478)}, {FLT(0.650257), FLT(0.344842), FLT(-0.676942)}};
            Vector3f p0 = {FLT(0.23), FLT(1.58), FLT(-0.22)};
            Vector3f p1 = {FLT(0.23), FLT(1.58), FLT(0.16)};
            Vector3f p2 = {FLT(-0.24), FLT(1.58), FLT(0.16)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.01851), FLT(1.35356), FLT(0.792757)}, {FLT(0.672), FLT(0.188647), FLT(-0.71612)}};
            Vector3f p0 = {FLT(1), FLT(1.59), FLT(-1.04)};
            Vector3f p1 = {FLT(1), FLT(1.59), FLT(0.99)};
            Vector3f p2 = {FLT(-1.02), FLT(1.59), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
    }

    SECTION("Fail : (t <= ray.m_min_t)"){
        {
            Ray r{{FLT(-1.00524), FLT(0.2882), FLT(-0.0450096)}, {FLT(0.563286), FLT(0.826047), FLT(-0.0188618)}};
            Vector3f p0 = {FLT(-0.99), FLT(0), FLT(-1.04)};
            Vector3f p1 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.01), FLT(0), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.00814), FLT(0.441475), FLT(-0.0439953)}, {FLT(0.697763), FLT(0.710467), FLT(0.0914531)}};
            Vector3f p0 = {FLT(-0.99), FLT(0), FLT(-1.04)};
            Vector3f p1 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.01), FLT(0), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.00401), FLT(0.195875), FLT(0.00703222)}, {FLT(0.486258), FLT(0.873805), FLT(0.00432434)}};
            Vector3f p0 = {FLT(-0.99), FLT(0), FLT(-1.04)};
            Vector3f p1 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.01), FLT(0), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
        {
            Ray r{{FLT(-1.00853), FLT(0.44075), FLT(-0.00356299)}, {FLT(0.717835), FLT(0.688741), FLT(-0.10173)}};
            Vector3f p0 = {FLT(-0.99), FLT(0), FLT(-1.04)};
            Vector3f p1 = {FLT(-1.02), FLT(1.59), FLT(-1.04)};
            Vector3f p2 = {FLT(-1.01), FLT(0), FLT(0.99)};
            auto [u, v, t] = moeller_trumbore(r, p0, p1, p2);
            CHECK(u == -1);
            CHECK(v == -1);
            CHECK(t == -1);
        }
    }
}

TEST_CASE("Ray-objmesh intersection"){

}

