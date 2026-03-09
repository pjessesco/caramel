#include <cmath>
#include <cstring>
#include <memory>

#include <common.h>
#include <polygon_sampling.h>
#include <sampler.h>

#include "catch_amalgamated.hpp"
#include <hypothesis.h>

using namespace Caramel;

// Check if point p (assumed coplanar with polygon) is inside convex polygon
static bool inside_convex_polygon(const Vector3f& p, const Vector3f* verts, Index n, const Vector3f& normal) {
    for (Index i = 0; i < n; ++i) {
        Index j = (i + 1) % n;
        Vector3f edge = verts[j] - verts[i];
        Vector3f to_p = p - verts[i];
        if (Vector3f::cross(edge, to_p).dot(normal) < Float0) {
            return false;
        }
    }
    return true;
}

static bool run_polygon_chi2_test(
    Index vertex_count,
    const Vector3f* vertices,
    const Vector3f& shading_pos,
    int cosThetaResolution = 50,
    int testCount = 5,
    float significanceLevel = 0.01f,
    int minExpFrequency = 5,
    int sampleCount = -1)
{
    const int cosThetaRes = cosThetaResolution;
    const int phiRes = 2 * cosThetaRes;
    const int res = cosThetaRes * phiRes;

    if (sampleCount < 0)
        sampleCount = res * 500;

    UniformStdSampler sampler(42);

    auto obsFreq = std::make_unique<double[]>(res);
    auto expFreq = std::make_unique<double[]>(res);

    // Precompute polygon data
    const auto polygon = prepare_solid_angle_polygon(vertex_count, vertices, shading_pos);

    // Plane from first 3 vertices
    const Vector3f plane_n = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
    const Vector3f plane_n_normalized = plane_n.normalize();

    int passed = 0;

    for (int t = 0; t < testCount; ++t) {
        std::memset(obsFreq.get(), 0, res * sizeof(double));
        std::memset(expFreq.get(), 0, res * sizeof(double));

        // Sample directions and bin them
        for (int i = 0; i < sampleCount; ++i) {
            Vector3f dir = sample_solid_angle_polygon(polygon, sampler.sample_1d(), sampler.sample_1d());

            int cosThetaBin = std::clamp(
                (int)std::floor((dir[2] * Float0_5 + Float0_5) * cosThetaRes),
                0, cosThetaRes - 1);

            Float scaledPhi = std::atan2(dir[1], dir[0]) * PI_2_INV;
            if (scaledPhi < Float0) scaledPhi += Float1;

            int phiBin = std::clamp(
                (int)std::floor(scaledPhi * phiRes),
                0, phiRes - 1);

            obsFreq[cosThetaBin * phiRes + phiBin] += 1;
        }

        // Expected frequencies via numerical integration
        double* ptr = expFreq.get();
        for (int i = 0; i < cosThetaRes; ++i) {
            double cosThetaStart = -1.0 + i       * 2.0 / cosThetaRes;
            double cosThetaEnd   = -1.0 + (i + 1) * 2.0 / cosThetaRes;
            for (int j = 0; j < phiRes; ++j) {
                double phiStart = j       * 2.0 * static_cast<double>(PI) / phiRes;
                double phiEnd   = (j + 1) * 2.0 * static_cast<double>(PI) / phiRes;

                auto integrand = [&](double ct, double phi) -> double {
                    double st = std::sqrt(1.0 - ct * ct);
                    Vector3f dir{
                        static_cast<Float>(st * std::cos(phi)),
                        static_cast<Float>(st * std::sin(phi)),
                        static_cast<Float>(ct)};

                    // Ray-plane intersection
                    Float denom = plane_n.dot(dir);
                    if (std::abs(denom) < 1e-8f) return 0.0;

                    Float t_val = plane_n.dot(vertices[0] - shading_pos) / denom;
                    if (t_val <= Float0) return 0.0;

                    Vector3f hit = shading_pos + dir * t_val;

                    if (!inside_convex_polygon(hit, vertices, vertex_count, plane_n_normalized))
                        return 0.0;

                    return 1.0 / polygon.solid_angle;
                };

                *ptr++ = hypothesis::adaptiveSimpson2D(
                    integrand, cosThetaStart, phiStart, cosThetaEnd, phiEnd, 1e-6, 10) * sampleCount;
            }
        }

        auto [success, message] = hypothesis::chi2_test(
            res, obsFreq.get(), expFreq.get(),
            sampleCount, minExpFrequency, significanceLevel, testCount);

        if (success) ++passed;
    }

    return passed == testCount;
}

TEST_CASE("Chi2 Polygon Sampling: Tilted Triangle", "[chi2][polygon]") {
    // Large elongated triangle through (0,0,1.5), tilted in x
    const Vector3f v0{-50.0f, -5.0f, -38.5f};
    const Vector3f v1{ 50.0f, -5.0f,  41.5f};
    const Vector3f v2{  0.0f,  5.0f,   1.5f};
    const Vector3f verts[3] = {v0, v1, v2};
    const Vector3f shading_pos{Float0, Float0, Float0};
    CHECK(run_polygon_chi2_test(3, verts, shading_pos));
}

TEST_CASE("Chi2 Polygon Sampling: Tilted Quad", "[chi2][polygon]") {
    // Large elongated quad through (0,0,1.5), tilted in x
    const Vector3f v0{-40.0f, -5.0f, -30.0f};
    const Vector3f v1{ 40.0f, -5.0f,  33.0f};
    const Vector3f v2{ 40.0f,  5.0f,  33.0f};
    const Vector3f v3{-40.0f,  5.0f, -30.0f};
    const Vector3f verts[4] = {v0, v1, v2, v3};
    const Vector3f shading_pos{Float0, Float0, Float0};
    CHECK(run_polygon_chi2_test(4, verts, shading_pos));
}
