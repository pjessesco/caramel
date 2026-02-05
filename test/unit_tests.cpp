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

#include <utils.h>
#include <image.h>
#include <ray.h>
#include <shape.h>
#include <rayintersectinfo.h>
#include <sampler.h>
#include <scene.h>

// Dependencies headers
#include "catch_amalgamated.hpp"
#include <random>

namespace Caramel {
// Helper class to create Ray objects for testing (must be in Caramel namespace for friend access)
class RayTestHelper {
public:
    static Ray create(const Vector3f &o, const Vector3f &d) {
        return Ray(o, d);
    }
};
}

using namespace Caramel;

constexpr Float TEST_EPSILON = EPSILON;

bool is_approx(Float a, Float b, Float epsilon = TEST_EPSILON) {
    if (std::abs(a) < 1e-3f && std::abs(b) < 1e-3f) return std::abs(a - b) < epsilon;
    return std::abs(a - b) <= ( (std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

TEST_CASE("uv <-> vector mapping test", "[UnitTest]") {
    for(Float u = 0.05f; u <= 0.98f; u += 0.1f){
        for(Float v = 0.05f; v <= 0.98f; v += 0.1f){
            CHECK(is_zero(vec_to_normalized_uv(normalized_uv_to_vec({u, v})) - Vector2f{u, v}));
        }
    }
}

// =============================================================================
// Triangle::ray_intersect Tests
// =============================================================================

TEST_CASE("Triangle::ray_intersect - XY plane triangle", "[UnitTest]") {
    // Triangle on XY plane: p0=(0,0,0), p1=(1,0,0), p2=(0,1,0)
    // For ray with dir (0,0,-1): hit point = (u, v, 0)
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{1.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 1.0f, 0.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Basic hit through center") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));

        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit near vertex p0") {
        const Float bary_u = 0.01f;
        const Float bary_v = 0.01f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit near vertex p1") {
        const Float bary_u = 0.95f;
        const Float bary_v = 0.01f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit near vertex p2") {
        const Float bary_u = 0.01f;
        const Float bary_v = 0.95f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit on edge p0-p1") {
        const Float bary_u = 0.5f;
        const Float bary_v = 0.0f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit on edge p0-p2") {
        const Float bary_u = 0.0f;
        const Float bary_v = 0.5f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit on edge p1-p2 (hypotenuse)") {
        const Float bary_u = 0.5f;
        const Float bary_v = 0.5f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Miss - ray passes beside triangle") {
        Ray ray = RayTestHelper::create({2.0f, 0.25f, 1.0f}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == false);
    }

    SECTION("Miss - ray pointing away from triangle") {
        Ray ray = RayTestHelper::create({0.25f, 0.25f, 1.0f}, {0.0f, 0.0f, 1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == false);
    }

    SECTION("Hit from backface (negative normal side)") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, bary_v, 0.0f};

        Ray ray = RayTestHelper::create({bary_u, bary_v, -expected_t}, {0.0f, 0.0f, 1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("maxt constraint - hit within maxt") {
        const Float expected_t = 1.0f;
        Ray ray = RayTestHelper::create({0.25f, 0.25f, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, 2.0f);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("maxt constraint - miss because t > maxt") {
        Ray ray = RayTestHelper::create({0.25f, 0.25f, 1.0f}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, 0.5f);

        CHECK(hit == false);
    }

    SECTION("Diagonal ray direction") {
        // Ray from (1,1,1) with direction (-1,-1,-1), hits at p0 = (0,0,0)
        const Float expected_t = std::sqrt(3.0f);
        const Vector3f expected_hit{0.0f, 0.0f, 0.0f};

        Ray ray = RayTestHelper::create({1.0f, 1.0f, 1.0f}, {-1.0f, -1.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Far away intersection") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 10000.0f;

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(std::abs(info.t - expected_t) < 1.0f);  // Larger epsilon for far distances
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("UV coordinates (no texture)") {
        // Without explicit texture coords, tex_uv should be barycentric (u, v)
        const Float bary_u = 0.5f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.tex_uv[0], bary_u));
        CHECK(is_approx(info.tex_uv[1], bary_v));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{2.788536f, -9.499785f, 0.000000f}, p1{-4.499414f, -5.535785f, 0.000000f}, p2{4.729424f, 3.533990f, 0.000000f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({1.206238f, -1.265551f, 3.071827f}, {-0.141065f, -0.849523f, -0.508341f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 6.042842f));
            CHECK(is_approx(info.p[0], 0.353803f));
            CHECK(is_approx(info.p[1], -6.399085f));
            CHECK(is_approx(info.p[2], 0.000000f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-9.469281f, -6.023247f, 0.000000f}, p1{2.997689f, 0.898830f, 0.000000f}, p2{-5.591188f, 1.785314f, 0.000000f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-7.293184f, -4.469489f, 1.302422f}, {0.768631f, 0.497994f, -0.401507f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 3.243836f));
            CHECK(is_approx(info.p[0], -4.799870f));
            CHECK(is_approx(info.p[1], -2.854078f));
            CHECK(is_approx(info.p[2], 0.000000f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{9.144261f, -3.268109f, 0.000000f}, p1{-8.145083f, -8.065672f, 0.000000f}, p2{6.949887f, 2.074521f, 0.000000f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({2.054499f, -9.403423f, 1.591183f}, {0.073965f, 0.965935f, -0.247990f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 6.416325f));
            CHECK(is_approx(info.p[0], 2.529083f));
            CHECK(is_approx(info.p[1], -3.205671f));
            CHECK(is_approx(info.p[2], 0.000000f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - XZ plane triangle", "[UnitTest]") {
    // Triangle on XZ plane
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{1.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 0.0f, 1.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit from +Y direction") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, 0.0f, bary_v};

        Ray ray = RayTestHelper::create({bary_u, expected_t, bary_v}, {0.0f, -1.0f, 0.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit from -Y direction (backface)") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{bary_u, 0.0f, bary_v};

        Ray ray = RayTestHelper::create({bary_u, -expected_t, bary_v}, {0.0f, 1.0f, 0.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{6.588093f, 0.000000f, 2.370395f}, p1{7.234138f, 0.000000f, 1.547043f}, p2{4.091437f, 0.000000f, -9.083512f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({9.011930f, 1.768735f, 2.733078f}, {-0.658527f, -0.418756f, -0.625289f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 4.223789f));
            CHECK(is_approx(info.p[0], 6.230451f));
            CHECK(is_approx(info.p[1], 0.000000f));
            CHECK(is_approx(info.p[2], 0.091990f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{2.713689f, 0.000000f, -2.703356f}, p1{-2.596381f, 0.000000f, -5.809859f}, p2{-4.660444f, 0.000000f, 8.733092f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({2.229378f, -2.204437f, 2.854007f}, {-0.628318f, 0.437767f, -0.643099f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.035644f));
            CHECK(is_approx(info.p[0], -0.934609f));
            CHECK(is_approx(info.p[1], 0.000000f));
            CHECK(is_approx(info.p[2], -0.384410f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{9.790467f, 0.000000f, 2.799995f}, p1{1.138995f, 0.000000f, 3.692285f}, p2{6.857038f, 0.000000f, 5.519998f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({12.261615f, 5.351762f, 9.908323f}, {-0.445588f, -0.560789f, -0.697831f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 9.543278f));
            CHECK(is_approx(info.p[0], 8.009247f));
            CHECK(is_approx(info.p[1], 0.000000f));
            CHECK(is_approx(info.p[2], 3.248731f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - YZ plane triangle", "[UnitTest]") {
    // Triangle on YZ plane
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{0.0f, 1.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 0.0f, 1.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit from +X direction") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{0.0f, bary_u, bary_v};

        Ray ray = RayTestHelper::create({expected_t, bary_u, bary_v}, {-1.0f, 0.0f, 0.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit from -X direction (backface)") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;
        const Vector3f expected_hit{0.0f, bary_u, bary_v};

        Ray ray = RayTestHelper::create({-expected_t, bary_u, bary_v}, {1.0f, 0.0f, 0.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{0.000000f, 7.527353f, -3.706442f}, p1{0.000000f, 3.108773f, -2.087362f}, p2{0.000000f, 8.290952f, -0.822963f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-2.173618f, 15.270765f, -5.910162f}, {0.236711f, -0.915160f, 0.326267f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 9.182583f));
            CHECK(is_approx(info.p[0], 0.000000f));
            CHECK(is_approx(info.p[1], 6.867234f));
            CHECK(is_approx(info.p[2], -2.914184f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{0.000000f, -2.011990f, -5.613585f}, p1{0.000000f, 9.950752f, 0.190526f}, p2{0.000000f, -8.181812f, -9.057672f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-2.779816f, -1.459726f, -1.680900f}, {0.550137f, -0.146613f, -0.822103f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.052954f));
            CHECK(is_approx(info.p[0], 0.000000f));
            CHECK(is_approx(info.p[1], -2.200556f));
            CHECK(is_approx(info.p[2], -5.834952f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{0.000000f, 9.922428f, 0.582287f}, p1{0.000000f, 9.421568f, 7.215594f}, p2{0.000000f, -9.770380f, 4.414436f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({2.692119f, 3.000818f, 8.087580f}, {-0.491431f, 0.297086f, -0.818679f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.478122f));
            CHECK(is_approx(info.p[0], 0.000000f));
            CHECK(is_approx(info.p[1], 4.628292f));
            CHECK(is_approx(info.p[2], 3.602756f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - 45 degree tilted triangle", "[UnitTest]") {
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{1.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 0.707f, 0.707f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    const Vector3f edge1 = tri_p1 - tri_p0;
    const Vector3f edge2 = tri_p2 - tri_p0;
    const Vector3f normal = Vector3f::cross(edge1, edge2).normalize();

    SECTION("Hit through centroid") {
        const Float bary_u = 1.0f / 3.0f;
        const Float bary_v = 1.0f / 3.0f;
        const Float ray_dist = 2.0f;

        Vector3f centroid = (tri_p0 + tri_p1 + tri_p2) * (1.0f / 3.0f);
        Vector3f ray_origin = centroid + normal * ray_dist;
        Vector3f ray_dir = normal * (-1.0f);

        Ray ray = RayTestHelper::create(ray_origin, ray_dir);
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, ray_dist));
        CHECK(is_approx(info.p[0], centroid[0]));
        CHECK(is_approx(info.p[1], centroid[1]));
        CHECK(is_approx(info.p[2], centroid[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit with specific barycentric") {
        const Float bary_u = 0.4f;
        const Float bary_v = 0.3f;
        const Float ray_dist = 3.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;
        Vector3f ray_origin = expected_hit + normal * ray_dist;

        Ray ray = RayTestHelper::create(ray_origin, normal * (-1.0f));
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, ray_dist));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{-0.462763f, 3.208963f, 3.208963f}, p1{3.758529f, -1.673092f, -1.673092f}, p2{0.005861f, -2.272274f, -2.272274f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({3.708369f, -2.266556f, -1.906009f}, {-0.752166f, 0.518533f, 0.406656f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 3.222714f));
            CHECK(is_approx(info.p[0], 1.284353f));
            CHECK(is_approx(info.p[1], -0.595472f));
            CHECK(is_approx(info.p[2], -0.595472f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{2.625108f, 0.278452f, 0.278452f}, p1{2.786265f, 0.214633f, 0.214633f}, p2{-4.994281f, -1.243404f, -1.243404f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-1.974515f, -1.820591f, 0.575104f}, {0.702684f, 0.615375f, -0.357140f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 2.463401f));
            CHECK(is_approx(info.p[0], -0.243523f));
            CHECK(is_approx(info.p[1], -0.304675f));
            CHECK(is_approx(info.p[2], -0.304675f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{3.780096f, 3.160410f, 3.160410f}, p1{-4.143465f, -0.099062f, -0.099062f}, p2{-4.307875f, 1.842736f, 1.842736f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({0.966282f, 0.050761f, 10.641188f}, {-0.102379f, 0.206284f, -0.973122f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 8.979464f));
            CHECK(is_approx(info.p[0], 0.046974f));
            CHECK(is_approx(info.p[1], 1.903077f));
            CHECK(is_approx(info.p[2], 1.903077f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - equilateral triangle", "[UnitTest]") {
    const Float h = std::sqrt(3.0f) / 2.0f;
    const Vector3f tri_p0{0.0f, h * 2.0f / 3.0f, 0.0f};
    const Vector3f tri_p1{-0.5f, -h / 3.0f, 0.0f};
    const Vector3f tri_p2{0.5f, -h / 3.0f, 0.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit at centroid") {
        const Float expected_t = 1.0f;
        Vector3f centroid = (tri_p0 + tri_p1 + tri_p2) * (1.0f / 3.0f);

        Ray ray = RayTestHelper::create({centroid[0], centroid[1], expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], centroid[0]));
        CHECK(is_approx(info.p[1], centroid[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit with specific barycentric coords") {
        const Float bary_u = 0.2f;
        const Float bary_v = 0.5f;
        const Float expected_t = 2.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Miss outside triangle") {
        Ray ray = RayTestHelper::create({1.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{-0.768621f, 1.037706f, 0.392961f}, p1{-2.728483f, -4.013545f, 0.392961f}, p2{1.191242f, -4.013545f, 0.392961f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-6.539250f, -2.516135f, 1.122489f}, {0.950336f, 0.287660f, -0.118804f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 6.140607f));
            CHECK(is_approx(info.p[0], -0.703613f));
            CHECK(is_approx(info.p[1], -0.749730f));
            CHECK(is_approx(info.p[2], 0.392961f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-3.789958f, 0.600208f, -1.619144f}, p1{-5.466575f, -3.721022f, -1.619144f}, p2{-2.113341f, -3.721022f, -1.619144f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({3.770935f, -3.159937f, 3.161178f}, {-0.818520f, 0.250137f, -0.517163f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 9.243360f));
            CHECK(is_approx(info.p[0], -3.794936f));
            CHECK(is_approx(info.p[1], -0.847834f));
            CHECK(is_approx(info.p[2], -1.619144f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{3.596354f, -0.615515f, -2.619954f}, p1{1.758398f, -5.352571f, -2.619954f}, p2{5.434310f, -5.352571f, -2.619954f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-4.602179f, -3.385426f, -2.108322f}, {0.985068f, 0.160689f, -0.061814f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 8.276955f));
            CHECK(is_approx(info.p[0], 3.551182f));
            CHECK(is_approx(info.p[1], -2.055412f));
            CHECK(is_approx(info.p[2], -2.619954f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - needle triangle", "[UnitTest]") {
    // Very thin (needle-like) triangle
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{10.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{5.0f, 0.001f, 0.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit in the middle") {
        const Float bary_u = 0.5f;
        const Float bary_v = 0.499f;
        const Float expected_t = 1.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Miss - just outside thin edge") {
        Ray ray = RayTestHelper::create({5.0f, 0.01f, 1.0f}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{3.074970f, -3.095901f, -4.030692f}, p1{10.230226f, -3.095901f, -4.030692f}, p2{5.498549f, -3.095381f, -4.030692f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({2.501500f, -0.121437f, -3.309459f}, {0.760636f, -0.630896f, -0.152984f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 4.714421f));
            CHECK(is_approx(info.p[0], 6.087456f));
            CHECK(is_approx(info.p[1], -3.095744f));
            CHECK(is_approx(info.p[2], -4.030692f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{3.616725f, -2.513437f, -3.097911f}, p1{10.859793f, -2.513437f, -3.097911f}, p2{6.038607f, -2.513086f, -3.097911f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({6.167205f, -4.866606f, -3.425657f}, {-0.154016f, 0.978623f, 0.136293f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 2.404707f));
            CHECK(is_approx(info.p[0], 5.796842f));
            CHECK(is_approx(info.p[1], -2.513304f));
            CHECK(is_approx(info.p[2], -3.097911f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{4.992825f, 3.360276f, 4.689963f}, p1{14.624660f, 3.360276f, 4.689963f}, p2{7.841520f, 3.360526f, 4.689963f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({9.915459f, 12.675683f, 7.244335f}, {-0.211347f, -0.942615f, -0.258475f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 9.882471f));
            CHECK(is_approx(info.p[0], 7.826833f));
            CHECK(is_approx(info.p[1], 3.360317f));
            CHECK(is_approx(info.p[2], 4.689963f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - negative coordinates", "[UnitTest]") {
    const Vector3f tri_p0{-1.0f, -1.0f, -1.0f};
    const Vector3f tri_p1{1.0f, -1.0f, -1.0f};
    const Vector3f tri_p2{0.0f, 1.0f, -1.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit at centroid from +Z") {
        const Float bary_u = 1.0f / 3.0f;
        const Float bary_v = 1.0f / 3.0f;
        const Float expected_t = 1.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], 0.0f}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit from -Z (backface)") {
        const Float bary_u = 1.0f / 3.0f;
        const Float bary_v = 1.0f / 3.0f;
        const Float expected_t = 1.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], -2.0f}, {0.0f, 0.0f, 1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{-7.613172f, -2.943365f, -5.904925f}, p1{-6.192933f, -1.384141f, -1.041196f}, p2{-4.998085f, -3.534326f, -8.606829f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-7.061073f, -3.272679f, -8.351741f}, {0.300258f, 0.160007f, 0.940342f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 2.457322f));
            CHECK(is_approx(info.p[0], -6.323243f));
            CHECK(is_approx(info.p[1], -2.879489f));
            CHECK(is_approx(info.p[2], -6.041018f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-4.742402f, -5.474347f, -2.325521f}, p1{-8.583105f, -1.352990f, -9.278997f}, p2{-8.327575f, -4.644684f, -3.923087f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-11.599106f, -0.942955f, -5.112577f}, {0.821585f, -0.534236f, 0.198970f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 6.955052f));
            CHECK(is_approx(info.p[0], -5.884938f));
            CHECK(is_approx(info.p[1], -4.658597f));
            CHECK(is_approx(info.p[2], -3.728729f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-6.226976f, -4.746949f, -5.294956f}, p1{-1.587644f, -8.161667f, -3.554274f}, p2{-7.851826f, -6.437927f, -3.954788f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-10.714489f, 2.849504f, -3.866304f}, {0.505863f, -0.858535f, -0.083782f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 9.987636f));
            CHECK(is_approx(info.p[0], -5.662114f));
            CHECK(is_approx(info.p[1], -5.725235f));
            CHECK(is_approx(info.p[2], -4.703092f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - far from origin", "[UnitTest]") {
    const Vector3f tri_p0{1000.0f, 1000.0f, 1000.0f};
    const Vector3f tri_p1{1001.0f, 1000.0f, 1000.0f};
    const Vector3f tri_p2{1000.0f, 1001.0f, 1000.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit from far away") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1000.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], 0.0f}, {0.0f, 0.0f, 1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(std::abs(info.t - expected_t) < 1.0f);  // Larger tolerance for large distances
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit from nearby") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], expected_hit[2] + expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{1004.960964f, 995.732607f, 997.131543f}, p1{997.652004f, 1004.332594f, 1003.808642f}, p2{1003.792702f, 998.695271f, 996.577468f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({1001.598132f, 997.743875f, 998.689808f}, {0.213514f, 0.931531f, 0.294384f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 2.062585f));
            CHECK(is_approx(info.p[0], 1002.038524f));
            CHECK(is_approx(info.p[1], 999.665237f));
            CHECK(is_approx(info.p[2], 999.297001f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{1003.171041f, 997.993788f, 1001.633887f}, p1{1004.389300f, 996.342911f, 996.154287f}, p2{996.070360f, 1000.532236f, 997.723482f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({1005.631816f, 996.356407f, 1002.327816f}, {-0.737379f, 0.333952f, -0.587152f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.908255f));
            CHECK(is_approx(info.p[0], 1001.275190f));
            CHECK(is_approx(info.p[1], 998.329482f));
            CHECK(is_approx(info.p[2], 998.858773f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{1004.053365f, 1003.461037f, 995.922985f}, p1{999.235758f, 997.766802f, 995.035457f}, p2{1002.711192f, 1001.371134f, 997.619553f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({1003.148202f, 1007.879908f, 1001.986062f}, {-0.110784f, -0.751187f, -0.650727f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 9.064851f));
            CHECK(is_approx(info.p[0], 1002.143964f));
            CHECK(is_approx(info.p[1], 1001.070513f));
            CHECK(is_approx(info.p[2], 996.087319f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - very large triangle", "[UnitTest]") {
    const Vector3f tri_p0{-1000.0f, -1000.0f, 0.0f};
    const Vector3f tri_p1{1000.0f, -1000.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 1000.0f, 0.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit at centroid") {
        const Float bary_u = 1.0f / 3.0f;
        const Float bary_v = 1.0f / 3.0f;
        const Float expected_t = 100.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Area calculation") {
        Vector3f edge1 = tri_p1 - tri_p0;
        Vector3f edge2 = tri_p2 - tri_p0;
        Float expected_area = Vector3f::cross(edge1, edge2).length() * 0.5f;

        CHECK(is_approx(tri.get_area(), expected_area));
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{807.857143f, 91.180578f, 669.190040f}, p1{165.019133f, -703.812429f, -745.108961f}, p2{-383.483300f, 797.962977f, 592.244610f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({141.503342f, 71.427177f, 140.059259f}, {-0.525344f, -0.453855f, -0.719742f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 8.240930f));
            CHECK(is_approx(info.p[0], 137.174015f));
            CHECK(is_approx(info.p[1], 67.686991f));
            CHECK(is_approx(info.p[2], 134.127913f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{768.269403f, -187.245220f, 241.323020f}, p1{-690.893323f, 859.762031f, 729.211392f}, p2{952.412066f, 621.543440f, 762.832409f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({673.314784f, 177.886059f, 456.251680f}, {-0.303809f, 0.779944f, 0.547163f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 8.912512f));
            CHECK(is_approx(info.p[0], 670.607079f));
            CHECK(is_approx(info.p[1], 184.837319f));
            CHECK(is_approx(info.p[2], 461.128276f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{621.498633f, -466.388581f, 574.749018f}, p1{-783.808747f, 744.333566f, 717.186503f}, p2{-555.132565f, 633.173211f, -79.393531f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-44.105425f, 141.354617f, 382.796893f}, {-0.433279f, -0.757646f, -0.488099f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 4.626096f));
            CHECK(is_approx(info.p[0], -46.109816f));
            CHECK(is_approx(info.p[1], 137.849672f));
            CHECK(is_approx(info.p[2], 380.538898f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - very small triangle", "[UnitTest]") {
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{0.0001f, 0.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 0.0001f, 0.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    SECTION("Hit at center") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        Ray ray = RayTestHelper::create({expected_hit[0], expected_hit[1], expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Area calculation") {
        Float expected_area = 0.5f * 0.0001f * 0.0001f;
        CHECK(std::abs(tri.get_area() - expected_area) < 1e-12f);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{0.000729f, 0.000934f, -0.000442f}, p1{0.000283f, -0.000201f, 0.000962f}, p2{0.000072f, 0.000878f, -0.000769f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-3.897540f, 1.977007f, 3.300232f}, {0.711766f, -0.360910f, -0.602606f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.476510f));
            CHECK(is_approx(info.p[0], 0.000454f));
            CHECK(is_approx(info.p[1], 0.000482f));
            CHECK(is_approx(info.p[2], 0.000057f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-0.000159f, 0.000881f, 0.000355f}, p1{0.000806f, 0.000231f, -0.000398f}, p2{0.000096f, -0.000999f, -0.000426f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-3.399038f, 0.769491f, 1.270825f}, {0.916314f, -0.207373f, -0.342585f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 3.709611f));
            CHECK(is_approx(info.p[0], 0.000132f));
            CHECK(is_approx(info.p[1], 0.000217f));
            CHECK(is_approx(info.p[2], -0.000032f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-0.000054f, 0.000802f, 0.000592f}, p1{-0.000661f, -0.000830f, 0.000031f}, p2{0.000266f, -0.000330f, 0.000637f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({1.739759f, 1.901038f, 3.005438f}, {-0.439512f, -0.480231f, -0.759084f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 3.958740f));
            CHECK(is_approx(info.p[0], -0.000155f));
            CHECK(is_approx(info.p[1], -0.000071f));
            CHECK(is_approx(info.p[2], 0.000423f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - arbitrary coordinates", "[UnitTest]") {
    // Completely arbitrary triangle with no vertex at origin or axis-aligned
    const Vector3f tri_p0{3.7f, -2.1f, 5.3f};
    const Vector3f tri_p1{7.2f, 1.4f, 4.8f};
    const Vector3f tri_p2{4.1f, 2.9f, 6.7f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    const Vector3f centroid = (tri_p0 + tri_p1 + tri_p2) * (1.0f / 3.0f);
    const Vector3f edge1 = tri_p1 - tri_p0;
    const Vector3f edge2 = tri_p2 - tri_p0;
    const Vector3f normal = Vector3f::cross(edge1, edge2).normalize();

    SECTION("Hit through centroid - front face") {
        const Float expected_t = 5.0f;

        Vector3f ray_origin = centroid + normal * expected_t;
        Vector3f ray_dir = normal * (-1.0f);

        Ray ray = RayTestHelper::create(ray_origin, ray_dir);
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], centroid[0]));
        CHECK(is_approx(info.p[1], centroid[1]));
        CHECK(is_approx(info.p[2], centroid[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit through centroid - back face") {
        const Float expected_t = 3.0f;

        Vector3f ray_origin = centroid - normal * expected_t;
        Vector3f ray_dir = normal;

        Ray ray = RayTestHelper::create(ray_origin, ray_dir);
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], centroid[0]));
        CHECK(is_approx(info.p[1], centroid[1]));
        CHECK(is_approx(info.p[2], centroid[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit with various barycentric coordinates") {
        const Float bary_u = 0.4f;
        const Float bary_v = 0.3f;
        const Float expected_t = 2.0f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;
        Vector3f ray_origin = expected_hit + normal * expected_t;

        Ray ray = RayTestHelper::create(ray_origin, normal * (-1.0f));
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        CHECK(is_approx(info.p[2], expected_hit[2]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Miss - outside triangle") {
        Vector3f outside = tri_p0 + (tri_p0 - centroid) * 3.0f;
        Vector3f ray_origin = outside + normal * 2.0f;

        Ray ray = RayTestHelper::create(ray_origin, normal * (-1.0f));
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == false);
    }

    SECTION("Miss - ray pointing away") {
        Vector3f ray_origin = centroid + normal * 2.0f;

        Ray ray = RayTestHelper::create(ray_origin, normal);
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == false);
    }

    SECTION("Area calculation") {
        Float expected_area = Vector3f::cross(edge1, edge2).length() * 0.5f;
        CHECK(is_approx(tri.get_area(), expected_area));
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{-0.248637f, 3.497377f, -4.271718f}, p1{-0.855590f, 1.297654f, -3.055648f}, p2{1.963543f, -0.056228f, -2.560156f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-2.718043f, -0.222121f, 0.204426f}, {0.465483f, 0.500875f, -0.729692f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.401170f));
            CHECK(is_approx(info.p[0], -0.203893f));
            CHECK(is_approx(info.p[1], 2.483189f));
            CHECK(is_approx(info.p[2], -3.736764f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-3.241133f, 4.579660f, 0.179578f}, p1{-4.497816f, -2.508017f, 3.483363f}, p2{-0.435382f, 3.014166f, 1.675777f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-8.706609f, -3.666243f, 0.466880f}, {0.741425f, 0.644861f, 0.185591f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 7.754192f));
            CHECK(is_approx(info.p[0], -2.957459f));
            CHECK(is_approx(info.p[1], 1.334131f));
            CHECK(is_approx(info.p[2], 1.905986f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{0.047782f, 3.305692f, 0.478720f}, p1{3.972081f, 2.436554f, -0.253256f}, p2{-2.408085f, -2.527603f, 1.376614f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-0.384460f, 3.473629f, 4.123079f}, {0.255866f, -0.455018f, -0.852931f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 4.285825f));
            CHECK(is_approx(info.p[0], 0.712137f));
            CHECK(is_approx(info.p[1], 1.523501f));
            CHECK(is_approx(info.p[2], 0.467566f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - with vertex normals", "[UnitTest]") {
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{1.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 1.0f, 0.0f};
    const Vector3f n0{0.0f, 0.0f, 1.0f};
    const Vector3f n1{0.1f, 0.0f, 0.995f};
    const Vector3f n2{0.0f, 0.1f, 0.995f};
    Triangle tri(tri_p0, tri_p1, tri_p2, n0, n1, n2, nullptr);

    SECTION("Hit returns interpolated normal") {
        const Float bary_u = 0.25f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, expected_t));
        // Shading normal should be interpolated from vertex normals
        Vector3f interpolated = n0 * (1.0f - bary_u - bary_v) + n1 * bary_u + n2 * bary_v;
        Vector3f expected_normal = interpolated.normalize();
        Vector3f actual_normal = info.sh_coord.to_world({0.0f, 0.0f, 1.0f});
        CHECK(std::abs(actual_normal[0] - expected_normal[0]) < 0.01f);
        CHECK(std::abs(actual_normal[1] - expected_normal[1]) < 0.01f);
        CHECK(std::abs(actual_normal[2] - expected_normal[2]) < 0.01f);
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{-2.282849f, -1.802904f, 0.401522f}, p1{-3.616259f, -2.687385f, 1.939498f}, p2{2.064191f, -4.357711f, -0.924006f};
            Vector3f n0{0.402454f, 0.734279f, 0.546686f}, n1{-0.225888f, -0.933064f, -0.279938f}, n2{0.403423f, 0.562537f, 0.721666f};
            Triangle tri(p0, p1, p2, n0, n1, n2, nullptr);
            Ray ray = RayTestHelper::create({2.208288f, -1.556782f, -4.828939f}, {-0.579174f, -0.157763f, 0.799793f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 6.672635f));
            Vector3f expected_n{0.466998f, 0.506273f, 0.724983f};
            Vector3f actual_n = info.sh_coord.to_world(Vector3f{0.0f,0.0f,1.0f});
            CHECK(is_approx(actual_n[0], expected_n[0]));
            CHECK(is_approx(actual_n[1], expected_n[1]));
            CHECK(is_approx(actual_n[2], expected_n[2]));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-0.809787f, 2.475157f, 0.461323f}, p1{1.032526f, -2.794613f, -2.805784f}, p2{-0.641640f, -4.709752f, -1.638705f};
            Vector3f n0{-0.967313f, -0.216719f, 0.131676f}, n1{-0.770469f, 0.232575f, -0.593537f}, n2{-0.081972f, -0.962463f, -0.258737f};
            Triangle tri(p0, p1, p2, n0, n1, n2, nullptr);
            Ray ray = RayTestHelper::create({4.443842f, -0.262111f, 4.181269f}, {-0.667353f, -0.064971f, -0.741902f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 6.978056f));
            Vector3f expected_n{-0.920853f, -0.317405f, -0.226458f};
            Vector3f actual_n = info.sh_coord.to_world(Vector3f{0.0f,0.0f,1.0f});
            CHECK(is_approx(actual_n[0], expected_n[0]));
            CHECK(is_approx(actual_n[1], expected_n[1]));
            CHECK(is_approx(actual_n[2], expected_n[2]));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-2.883397f, -1.731542f, 2.612297f}, p1{-1.208738f, 2.520098f, 3.319243f}, p2{-2.477285f, -4.180938f, -4.806167f};
            Vector3f n0{0.425398f, 0.752317f, -0.503047f}, n1{-0.684763f, -0.496301f, -0.533652f}, n2{0.505560f, 0.190846f, -0.841419f};
            Triangle tri(p0, p1, p2, n0, n1, n2, nullptr);
            Ray ray = RayTestHelper::create({0.789469f, -4.672145f, -5.927849f}, {-0.425831f, 0.426128f, 0.798175f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 7.214037f));
            Vector3f expected_n{0.239257f, 0.288074f, -0.927237f};
            Vector3f actual_n = info.sh_coord.to_world(Vector3f{0.0f,0.0f,1.0f});
            CHECK(is_approx(actual_n[0], expected_n[0]));
            CHECK(is_approx(actual_n[1], expected_n[1]));
            CHECK(is_approx(actual_n[2], expected_n[2]));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - with texture coordinates", "[UnitTest]") {
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{1.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{0.0f, 1.0f, 0.0f};
    const Vector3f n0{0.0f, 0.0f, 1.0f};
    const Vector3f n1{0.0f, 0.0f, 1.0f};
    const Vector3f n2{0.0f, 0.0f, 1.0f};
    const Vector2f uv0{0.0f, 0.0f};
    const Vector2f uv1{1.0f, 0.0f};
    const Vector2f uv2{0.0f, 1.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, n0, n1, n2, uv0, uv1, uv2, nullptr);

    SECTION("UV coordinates are interpolated") {
        const Float bary_u = 0.5f;
        const Float bary_v = 0.25f;
        const Float expected_t = 1.0f;

        // Expected texture UV: interpolated from uv0, uv1, uv2
        // tex_uv = uv0 * (1 - u - v) + uv1 * u + uv2 * v
        const Float expected_tex_u = 0.5f;
        const Float expected_tex_v = 0.25f;

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.tex_uv[0], expected_tex_u));
        CHECK(is_approx(info.tex_uv[1], expected_tex_v));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("UV wrapping") {
        const Vector2f uv0_wrap{0.5f, 0.5f};
        const Vector2f uv1_wrap{1.5f, 0.5f};
        const Vector2f uv2_wrap{0.5f, 1.5f};
        Triangle tri_wrap(tri_p0, tri_p1, tri_p2, n0, n1, n2, uv0_wrap, uv1_wrap, uv2_wrap, nullptr);

        const Float bary_u = 0.5f;
        const Float bary_v = 0.5f;
        const Float expected_t = 1.0f;

        // tex_uv = uv0 * 0 + uv1 * 0.5 + uv2 * 0.5 = (1.0, 1.0)
        // After wrapping: (0.0, 0.0)

        Ray ray = RayTestHelper::create({bary_u, bary_v, expected_t}, {0.0f, 0.0f, -1.0f});
        auto [hit, info] = tri_wrap.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.tex_uv[0], 0.0f));
        CHECK(is_approx(info.tex_uv[1], 0.0f));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Additional Random Cases") {
        {
            Vector3f p0{1.994650f, 2.668981f, -3.322109f}, p1{1.072475f, 2.479257f, -3.854671f}, p2{3.193012f, 4.647208f, -3.919013f};
            Vector3f n{0.000000f, 0.000000f, 1.000000f};
            Vector2f uv0{0.075996f, 0.690614f}, uv1{0.627242f, 0.101901f}, uv2{0.772481f, 0.850293f};
            Triangle tri(p0, p1, p2, n, n, n, uv0, uv1, uv2, nullptr);
            Ray ray = RayTestHelper::create({-0.599784f, -4.013897f, -1.905858f}, {0.353246f, 0.912603f, -0.205847f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 7.720118f));
            CHECK(is_approx(info.tex_uv[0], 0.270198f));
            CHECK(is_approx(info.tex_uv[1], 0.658120f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{1.004116f, -3.789449f, 4.838444f}, p1{2.826353f, -1.527962f, -0.716220f}, p2{-1.294291f, 0.059608f, -1.587688f};
            Vector3f n{0.000000f, 0.000000f, 1.000000f};
            Vector2f uv0{0.707309f, 0.435487f}, uv1{0.733795f, 0.965474f}, uv2{0.270082f, 0.808199f};
            Triangle tri(p0, p1, p2, n, n, n, uv0, uv1, uv2, nullptr);
            Ray ray = RayTestHelper::create({6.330773f, -8.050197f, -1.243206f}, {-0.634654f, 0.741368f, 0.218145f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 8.629658f));
            CHECK(is_approx(info.tex_uv[0], 0.565122f));
            CHECK(is_approx(info.tex_uv[1], 0.752784f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{0.381729f, -0.165025f, -0.644255f}, p1{2.310262f, -2.316045f, 3.517132f}, p2{3.307310f, -4.133371f, 3.816312f};
            Vector3f n{0.000000f, 0.000000f, 1.000000f};
            Vector2f uv0{0.181840f, 0.212120f}, uv1{0.797832f, 0.340339f}, uv2{0.880320f, 0.701184f};
            Triangle tri(p0, p1, p2, n, n, n, uv0, uv1, uv2, nullptr);
            Ray ray = RayTestHelper::create({-0.531559f, 0.648600f, 9.463958f}, {0.221133f, -0.242536f, -0.944604f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 8.807623f));
            CHECK(is_approx(info.tex_uv[0], 0.455729f));
            CHECK(is_approx(info.tex_uv[1], 0.351410f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}

TEST_CASE("Triangle::ray_intersect - oblique rays", "[UnitTest]") {
    const Vector3f tri_p0{0.0f, 0.0f, 0.0f};
    const Vector3f tri_p1{2.0f, 0.0f, 0.0f};
    const Vector3f tri_p2{1.0f, 2.0f, 0.0f};
    Triangle tri(tri_p0, tri_p1, tri_p2, nullptr);

    const Vector3f edge1 = tri_p1 - tri_p0;
    const Vector3f edge2 = tri_p2 - tri_p0;
    const Vector3f normal = Vector3f::cross(edge1, edge2).normalize();
    const Vector3f centroid = (tri_p0 + tri_p1 + tri_p2) * (1.0f / 3.0f);

    SECTION("45 degree angle hit") {
        const Float bary_u = 0.3f;
        const Float bary_v = 0.3f;

        Vector3f expected_hit = tri_p0 * (1.0f - bary_u - bary_v)
                              + tri_p1 * bary_u + tri_p2 * bary_v;

        // Ray at 45 degrees
        Vector3f ray_dir{1.0f, 0.0f, -1.0f};
        Float z_dist = 2.0f;
        Vector3f ray_origin{expected_hit[0] - z_dist, expected_hit[1], z_dist};

        Ray ray = RayTestHelper::create(ray_origin, ray_dir);
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.p[0], expected_hit[0]));
        CHECK(is_approx(info.p[1], expected_hit[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }

    SECTION("Hit from diagonal direction using normal") {
        const Float ray_dist = 3.0f;

        Vector3f ray_origin = centroid + normal * ray_dist;
        Vector3f ray_dir = normal * (-1.0f);

        Ray ray = RayTestHelper::create(ray_origin, ray_dir);
        auto [hit, info] = tri.ray_intersect(ray, INF);

        CHECK(hit == true);
        CHECK(is_approx(info.t, ray_dist));
        CHECK(is_approx(info.p[0], centroid[0]));
        CHECK(is_approx(info.p[1], centroid[1]));
        // Self-intersection check
        Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
        CHECK(tri.ray_intersect(self_check, INF).first == false);
    }
        
    SECTION("Additional Random Cases") {
        {
            Vector3f p0{-2.237314f, -4.898489f, 4.480626f}, p1{-4.143870f, 2.200747f, -0.114222f}, p2{2.581647f, 1.906093f, 1.459029f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-0.431744f, -3.217315f, 3.539715f}, {-0.236381f, 0.864741f, -0.443111f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 2.744427f));
            CHECK(is_approx(info.p[0], -1.080474f));
            CHECK(is_approx(info.p[1], -0.844097f));
            CHECK(is_approx(info.p[2], 2.323630f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-2.784036f, 1.917872f, -1.937940f}, p1{0.815556f, -0.267395f, 0.309219f}, p2{-0.744962f, 2.459354f, -1.692087f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-4.397598f, 3.358230f, -2.737596f}, {0.773367f, -0.503962f, 0.384612f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 4.011229f));
            CHECK(is_approx(info.p[0], -1.295447f));
            CHECK(is_approx(info.p[1], 1.336725f));
            CHECK(is_approx(info.p[2], -1.194829f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        {
            Vector3f p0{-3.793441f, -3.074157f, -3.804453f}, p1{0.358640f, 2.621896f, -3.148502f}, p2{-2.836154f, -0.158014f, 2.245850f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-4.069262f, -3.750749f, -2.314107f}, {0.505140f, 0.859671f, 0.076155f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 4.263990f));
            CHECK(is_approx(info.p[0], -1.915350f));
            CHECK(is_approx(info.p[1], -0.085121f));
            CHECK(is_approx(info.p[2], -1.989382f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }
}
        
TEST_CASE("Triangle::ray_intersect - Randomized Robustness & Watertightness", "[UnitTest]") {
    std::mt19937 rng(42);
    std::uniform_real_distribution<Float> dist_coord(-10.0f, 10.0f);
    std::uniform_real_distribution<Float> dist_01(0.0f, 1.0f);

    SECTION("Fixed randomized robustness cases (Arbitrary)") {
        // Robustness Case 1
        {
            Vector3f p0{1.100000f, 2.200000f, 3.300000f}; Vector3f p1{4.500000f, 1.200000f, 5.600000f}; Vector3f p2{2.300000f, 5.800000f, 1.900000f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({2.130050f, 4.033900f, 8.476500f}, {0.097590f, -0.195180f, -0.975900f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.000000f));
            CHECK(is_approx(info.p[0], 2.618000f));
            CHECK(is_approx(info.p[1], 3.058000f));
            CHECK(is_approx(info.p[2], 3.597000f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        // Robustness Case 2
        {
            Vector3f p0{-5.500000f, 3.200000f, 1.000000f}; Vector3f p1{2.100000f, 8.800000f, -2.200000f}; Vector3f p2{0.500000f, -4.100000f, 3.300000f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({-1.012000f, -0.896534f, 4.238534f}, {0.000000f, 0.707107f, -0.707107f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.000000f));
            CHECK(is_approx(info.p[0], -1.012000f));
            CHECK(is_approx(info.p[1], 2.639000f));
            CHECK(is_approx(info.p[2], 0.703000f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
        // Robustness Case 3
        {
            Vector3f p0{100.100000f, 0.200000f, 0.300000f}; Vector3f p1{10.500000f, 50.200000f, 0.100000f}; Vector3f p2{10.300000f, 0.500000f, 20.800000f};
            Triangle tri(p0, p1, p2, nullptr);
            Ray ray = RayTestHelper::create({38.011249f, 13.912249f, 4.112249f}, {0.577350f, 0.577350f, 0.577350f});
            auto [hit, info] = tri.ray_intersect(ray, INF);
            CHECK(hit == true);
            CHECK(is_approx(info.t, 5.000000f));
            CHECK(is_approx(info.p[0], 40.898000f));
            CHECK(is_approx(info.p[1], 16.799000f));
            CHECK(is_approx(info.p[2], 6.999000f));
            // Self-intersection check
            Ray self_check = info.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);
        }
    }

    SECTION("Edge cases - Watertightness check (Explicit GT)") {
        // Edge Case 1 (Edge p0-p1)
        {
            Vector3f p0{0.000000f, 0.000000f, 0.000000f}; Vector3f p1{10.000000f, 0.000000f, 10.000000f}; Vector3f p2{5.000000f, 10.000000f, 5.000000f};
            Triangle tri(p0, p1, p2, nullptr);
            // 1. Slightly Inside (Hit)
            Ray ray_in = RayTestHelper::create({1.464466f, 0.000100f, 8.535534f}, {0.707107f, -0.000000f, -0.707107f});
            auto [hit_in, info_in] = tri.ray_intersect(ray_in, INF);
            CHECK(hit_in == true);
            CHECK(is_approx(info_in.t, 5.000000f));
            // Self-intersection check
            Ray self_check = info_in.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);

            // 2. Slightly Outside (Miss)
            Ray ray_out = RayTestHelper::create({1.464466f, -0.000100f, 8.535534f}, {0.707107f, -0.000000f, -0.707107f});
            auto [hit_out, info_out] = tri.ray_intersect(ray_out, INF);
            CHECK(hit_out == false);
        }
        // Edge Case 2 (Edge p0-p1)
        {
            Vector3f p0{-2.000000f, -2.000000f, 0.000000f}; Vector3f p1{2.000000f, -2.000000f, 0.000000f}; Vector3f p2{0.000000f, 2.000000f, 2.000000f};
            Triangle tri(p0, p1, p2, nullptr);
            // 1. Slightly Inside (Hit)
            Ray ray_in = RayTestHelper::create({0.000000f, -4.235979f, 4.472181f}, {-0.000000f, 0.447214f, -0.894427f});
            auto [hit_in, info_in] = tri.ray_intersect(ray_in, INF);
            CHECK(hit_in == true);
            CHECK(is_approx(info_in.t, 5.000000f));
            // Self-intersection check
            Ray self_check = info_in.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);

            // 2. Slightly Outside (Miss)
            Ray ray_out = RayTestHelper::create({0.000000f, -4.236157f, 4.472091f}, {-0.000000f, 0.447214f, -0.894427f});
            auto [hit_out, info_out] = tri.ray_intersect(ray_out, INF);
            CHECK(hit_out == false);
        }
        // Edge Case 3 (Edge p0-p1)
        {
            Vector3f p0{3.500000f, 1.200000f, 0.500000f}; Vector3f p1{8.900000f, 2.100000f, 1.200000f}; Vector3f p2{5.200000f, 5.500000f, 3.300000f};
            Triangle tri(p0, p1, p2, nullptr);
            // 1. Slightly Inside (Hit)
            Ray ray_in = RayTestHelper::create({6.104954f, -1.051355f, 5.056382f}, {0.019005f, 0.540287f, -0.841266f});
            auto [hit_in, info_in] = tri.ray_intersect(ray_in, INF);
            CHECK(hit_in == true);
            CHECK(is_approx(info_in.t, 5.000000f));
            // Self-intersection check
            Ray self_check = info_in.recursive_ray_to(Vector3f{0.0f,0.0f,1.0f});
            CHECK(tri.ray_intersect(self_check, INF).first == false);

            // 2. Slightly Outside (Miss)
            Ray ray_out = RayTestHelper::create({6.104995f, -1.051520f, 5.056277f}, {0.019005f, 0.540287f, -0.841266f});
            auto [hit_out, info_out] = tri.ray_intersect(ray_out, INF);
            CHECK(hit_out == false);
        }
    }
}

TEST_CASE("Triangle::parallel_ray", "[UnitTest]") {
    Vector3f p0{0.0f, 0.0f, 0.0f};
    Vector3f p1{1.0f, 0.0f, 0.0f};
    Vector3f p2{0.0f, 1.0f, 0.0f};
    Triangle tri(p0, p1, p2, nullptr);

    SECTION("Parallel ray above triangle") {
        // Ray parallel to the triangle plane (z=0), shifted by z=1
        Ray ray = RayTestHelper::create({0.5f, 0.5f, 1.0f}, {1.0f, 1.0f, 0.0f});
        auto [hit, info] = tri.ray_intersect(ray, INF);
        CHECK(hit == false);
    }
}

TEST_CASE("Scene::is_visible", "[UnitTest]") {
    // 1. Setup Scene
    Scene scene;

    // Triangle 1 (Target): Z=0 plane
    // (-1, -1, 0), (1, -1, 0), (0, 1, 0)
    Vector3f t1_p0{-1.0f, -1.0f, 0.0f};
    Vector3f t1_p1{ 1.0f, -1.0f, 0.0f};
    Vector3f t1_p2{ 0.0f,  1.0f, 0.0f};
    Shape* tri1 = new Triangle(t1_p0, t1_p1, t1_p2, nullptr);
    scene.add_mesh_and_arealight(tri1);

    // Triangle 2 (Occluder): Z=2 plane (Smaller triangle)
    // (-0.5, -0.5, 2), (0.5, -0.5, 2), (0.0, 0.5, 2)
    Vector3f t2_p0{-0.5f, -0.5f, 2.0f};
    Vector3f t2_p1{ 0.5f, -0.5f, 2.0f};
    Vector3f t2_p2{ 0.0f,  0.5f, 2.0f};
    Shape* tri2 = new Triangle(t2_p0, t2_p1, t2_p2, nullptr);
    scene.add_mesh_and_arealight(tri2);

    scene.build_bvh();

    SECTION("Visible - Unobstructed view") {
        Vector3f obs_pos{0.0f, 0.0f, 1.0f}; // In front of target
        Vector3f target_pos{0.0f, 0.0f, 0.0f}; // On target (centroid)

        bool visible = scene.is_visible(obs_pos, target_pos);
        CHECK(visible == true);
    }

    SECTION("Occluded - Obstruction in between") {
        Vector3f obs_pos{0.0f, 0.0f, 4.0f}; // Behind occluder (Z=2)
        Vector3f target_pos{0.0f, 0.0f, 0.0f}; // On target (Z=0)
        
        bool visible = scene.is_visible(obs_pos, target_pos);
        CHECK(visible == false);
    }

    SECTION("Visible - Target is empty space") {
        Vector3f obs_pos{0.0f, 0.0f, 4.0f};
        Vector3f target_pos{10.0f, 10.0f, 0.0f}; // Way outside
        
        bool visible = scene.is_visible(obs_pos, target_pos);
        CHECK(visible == true);
    }

    SECTION("Robustness - Very close to target") {
        Vector3f target_pos{0.0f, 0.0f, 0.0f};

        // 1. Just outside epsilon (should be visible)
        // Epsilon is 1e-3. Let's try 2e-3.
        Vector3f obs_safe{0.0f, 0.0f, 0.002f};
        CHECK(scene.is_visible(obs_safe, target_pos) == true);

        // 2. Inside epsilon (ray starts behind target, so it misses)
        Vector3f obs_unsafe{0.0f, 0.0f, 0.0005f};
        CHECK(scene.is_visible(obs_unsafe, target_pos) == true);
    }
        
            SECTION("Robustness - Grazing angle") {        // Observer at (10, 0, 0.1). Target at (0, 0, 0).
        // Ray travels almost parallel to Z=0 plane.
        Vector3f obs_pos{10.0f, 0.0f, 0.1f};
        Vector3f target_pos{0.0f, 0.0f, 0.0f};
        CHECK(scene.is_visible(obs_pos, target_pos) == true);
    }

    SECTION("Robustness - Very close to Occluder") {
        Vector3f target_pos{0.0f, 0.0f, 0.0f};
        // Occluder is at Z=2.0

        // 1. Observer just BEFORE occluder (should be blocked)
        // Z=2.002. Ray origin starts at 2.001. Hits Z=2.0.
        Vector3f obs_blocked{0.0f, 0.0f, 2.002f};
        CHECK(scene.is_visible(obs_blocked, target_pos) == false);

        // 2. Observer inside occluder epsilon (should see through)
        // Z=2.0005. Ray origin starts at 1.9995 (past occluder).
        Vector3f obs_past{0.0f, 0.0f, 2.0005f};
        CHECK(scene.is_visible(obs_past, target_pos) == true);
    }

    delete tri1;
    delete tri2;
}

