#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <sstream>

#include <common.h>
#include <bsdf.h>
#include <sampler.h>

#include "catch_amalgamated.hpp"
#include <hypothesis.h>

using namespace Caramel;

bool run_chi2_test(BSDF &bsdf,
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

        // Generate random incoming direction
        // In Caramel convention, local_incoming_dir points towards the surface (z < 0 for front-side)
        Float cosTheta_i = sampler.sample_1d();
        Float sinTheta_i = std::sqrt(std::max(Float0, Float1 - cosTheta_i * cosTheta_i));
        Float phi_i = PI_2 * sampler.sample_1d();
        Vector3f wi{std::cos(phi_i) * sinTheta_i,
                     std::sin(phi_i) * sinTheta_i,
                     cosTheta_i};
        Vector3f local_incoming_dir = -wi;

        // Accumulate samples into contingency table
        for (int i = 0; i < sampleCount; ++i) {
            auto [wo, weight, pdf] = bsdf.sample_recursive_dir(local_incoming_dir, Vector2f{Float0, Float0}, sampler);

            if (pdf <= Float0 || is_zero(wo)) {
                continue;
            }

            int cosThetaBin = std::clamp((int)std::floor((wo[2] * Float0_5 + Float0_5) * cosThetaRes), 0, cosThetaRes - 1);

            Float scaledPhi = std::atan2(wo[1], wo[0]) * PI_2_INV;
            if (scaledPhi < Float0) {
                scaledPhi += Float1;
            }

            int phiBin = std::clamp((int)std::floor(scaledPhi * phiRes), 0, phiRes - 1);

            obsFreq[cosThetaBin * phiRes + phiBin] += 1;
        }

        // Numerically integrate expected frequencies
        double *ptr = expFreq.get();
        for (int i = 0; i < cosThetaRes; ++i) {
            double cosThetaStart = -1.0 + i       * 2.0 / cosThetaRes;
            double cosThetaEnd   = -1.0 + (i + 1) * 2.0 / cosThetaRes;
            for (int j = 0; j < phiRes; ++j) {
                double phiStart = j       * 2.0 * static_cast<double>(PI) / phiRes;
                double phiEnd   = (j + 1) * 2.0 * static_cast<double>(PI) / phiRes;

                auto integrand = [&](double ct, double phi) -> double {
                    double st = std::sqrt(1.0 - ct * ct);
                    Vector3f wo{static_cast<Float>(st * std::cos(phi)),
                                static_cast<Float>(st * std::sin(phi)),
                                static_cast<Float>(ct)};
                    return bsdf.pdf(local_incoming_dir, wo);
                };

                *ptr++ = hypothesis::adaptiveSimpson2D(integrand, cosThetaStart, phiStart, cosThetaEnd, phiEnd) * sampleCount;
            }
        }

        // Perform chi-square test
        auto [success, message] = hypothesis::chi2_test(
            res, obsFreq.get(), expFreq.get(),
            sampleCount, minExpFrequency, significanceLevel, testCount);

        if (success) {
            ++passed;
        }

    }

    return passed == testCount;
}

TEST_CASE("Chi2 BSDF: Diffuse", "[chi2][bsdf]") {
    Diffuse bsdf(Vector3f{Float0_5, Float0_5, Float0_5});
    CHECK(run_chi2_test(bsdf));
}

TEST_CASE("Chi2 BSDF: OrenNayar (sigma=20)", "[chi2][bsdf]") {
    OrenNayar bsdf(Vector3f{Float0_5, Float0_5, Float0_5}, 20.0f);
    CHECK(run_chi2_test(bsdf));
}

TEST_CASE("Chi2 BSDF: Microfacet (alpha=0.3, ior=1.5)", "[chi2][bsdf]") {
    Microfacet bsdf(0.3f, IOR::GLASS, IOR::VACUUM, Vector3f{0.3f, 0.3f, 0.3f});
    CHECK(run_chi2_test(bsdf));
}

TEST_CASE("Chi2 BSDF: Microfacet (alpha=0.1, ior=1.33)", "[chi2][bsdf]") {
    Microfacet bsdf(0.1f, 1.33f, IOR::VACUUM, Vector3f{0.3f, 0.3f, 0.3f});
    CHECK(run_chi2_test(bsdf));
}

TEST_CASE("Chi2 BSDF: Microfacet (alpha=0.6, ior=1.8)", "[chi2][bsdf]") {
    Microfacet bsdf(0.6f, 1.8f, IOR::VACUUM, Vector3f{0.3f, 0.3f, 0.3f});
    CHECK(run_chi2_test(bsdf));
}
