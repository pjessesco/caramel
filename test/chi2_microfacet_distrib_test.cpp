#include <cmath>
#include <cstring>
#include <memory>

#include <common.h>
#include <microfacet_distrib.h>
#include <sampler.h>

#include "catch_amalgamated.hpp"
#include <hypothesis.h>

using namespace Caramel;

// For Beckmann (non-visible sampling): sample on upper hemisphere, PDF = D(wm) * cos(theta_m)
// For GGX (visible normal sampling): sample depends on w, PDF = G1(w)/|cos(theta)| * D(wm) * |w . wm|
static bool run_distrib_chi2_test(MicrofacetDistribution &distrib,
                                  const Vector3f &w,
                                  int cosThetaResolution = 10,
                                  int testCount = 5,
                                  float significanceLevel = 0.01f,
                                  int minExpFrequency = 5,
                                  int sampleCount = -1) {
    const int cosThetaRes = cosThetaResolution;
    const int phiRes = 2 * cosThetaRes;
    const int res = cosThetaRes * phiRes;

    if (sampleCount < 0)
        sampleCount = res * 5000;

    UniformStdSampler sampler(42);

    auto obsFreq = std::make_unique<double[]>(res);
    auto expFreq = std::make_unique<double[]>(res);

    int passed = 0;

    for (int t = 0; t < testCount; ++t) {
        std::memset(obsFreq.get(), 0, res * sizeof(double));
        std::memset(expFreq.get(), 0, res * sizeof(double));

        for (int i = 0; i < sampleCount; ++i) {
            Vector3f wm = distrib.Sample_wm(w, sampler);

            if (wm[2] <= Float0) continue;

            int cosThetaBin = std::clamp(
                static_cast<int>(std::floor(wm[2] * cosThetaRes)),
                0, cosThetaRes - 1);

            Float scaledPhi = std::atan2(wm[1], wm[0]) * PI_2_INV;
            if (scaledPhi < Float0) scaledPhi += Float1;

            int phiBin = std::clamp(
                static_cast<int>(std::floor(scaledPhi * phiRes)),
                0, phiRes - 1);

            obsFreq[cosThetaBin * phiRes + phiBin] += 1;
        }

        double *ptr = expFreq.get();
        for (int i = 0; i < cosThetaRes; ++i) {
            double cosThetaStart = static_cast<double>(i)     / cosThetaRes;
            double cosThetaEnd   = static_cast<double>(i + 1) / cosThetaRes;
            for (int j = 0; j < phiRes; ++j) {
                double phiStart = j       * 2.0 * static_cast<double>(PI) / phiRes;
                double phiEnd   = (j + 1) * 2.0 * static_cast<double>(PI) / phiRes;

                auto integrand = [&](double ct, double phi) -> double {
                    double st = std::sqrt(1.0 - ct * ct);
                    Vector3f wm{static_cast<Float>(st * std::cos(phi)),
                                static_cast<Float>(st * std::sin(phi)),
                                static_cast<Float>(ct)};
                    return distrib.PDF(w, wm);
                };

                *ptr++ = hypothesis::adaptiveSimpson2D(
                    integrand, cosThetaStart, phiStart, cosThetaEnd, phiEnd) * sampleCount;
            }
        }

        auto [success, message] = hypothesis::chi2_test(
            res, obsFreq.get(), expFreq.get(),
            sampleCount, minExpFrequency, significanceLevel, testCount);

        if (success) ++passed;
    }

    return passed == testCount;
}

// --- Beckmann ---

TEST_CASE("Chi2 MicrofacetDistrib: Beckmann (alpha=0.1)", "[chi2][distrib]") {
    BeckmannDistribution distrib(0.1f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: Beckmann (alpha=0.3)", "[chi2][distrib]") {
    BeckmannDistribution distrib(0.3f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: Beckmann (alpha=0.6)", "[chi2][distrib]") {
    BeckmannDistribution distrib(0.6f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: Beckmann (alpha=1.0)", "[chi2][distrib]") {
    BeckmannDistribution distrib(1.0f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

// --- GGX ---

TEST_CASE("Chi2 MicrofacetDistrib: GGX (alpha=0.1, normal incidence)", "[chi2][distrib]") {
    GGXDistribution distrib(0.1f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: GGX (alpha=0.3, normal incidence)", "[chi2][distrib]") {
    GGXDistribution distrib(0.3f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: GGX (alpha=0.6, normal incidence)", "[chi2][distrib]") {
    GGXDistribution distrib(0.6f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: GGX (alpha=1.0, normal incidence)", "[chi2][distrib]") {
    GGXDistribution distrib(1.0f);
    Vector3f w{Float0, Float0, Float1};
    CHECK(run_distrib_chi2_test(distrib, w));
}

TEST_CASE("Chi2 MicrofacetDistrib: GGX (alpha=0.3, grazing)", "[chi2][distrib]") {
    GGXDistribution distrib(0.3f);
    Vector3f w = Vector3f{0.8f, Float0, 0.6f}.normalize();
    CHECK(run_distrib_chi2_test(distrib, w, 10, 5, 0.01f, 5, 200 * 10000));
}

TEST_CASE("Chi2 MicrofacetDistrib: GGX (alpha=0.6, grazing)", "[chi2][distrib]") {
    GGXDistribution distrib(0.6f);
    Vector3f w = Vector3f{0.8f, Float0, 0.6f}.normalize();
    CHECK(run_distrib_chi2_test(distrib, w, 10, 5, 0.01f, 5, 200 * 10000));
}
