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

#pragma once

#include <cmath>

#include <common.h>
#include <sampler.h>

namespace Caramel {

// Base class for microfacet normal distributions.
//
// Subclasses implement D, G1, Sample_wm, PDF.
// G (masking-shadowing for two directions) is provided by the base class
// as G1(wo) * G1(wi) [uncorrelated].
//
// Reference: microfacet_rough_bsdf_guide.md
class MicrofacetDistribution {
public:
    explicit MicrofacetDistribution(Float alpha)
        : m_alpha{std::max(alpha, static_cast<Float>(1e-4))} {}

    virtual ~MicrofacetDistribution() = default;

    // D(wm): normal distribution function
    virtual Float D(const Vector3f &wm) const = 0;

    // G1(w): Smith single-direction masking function
    virtual Float G1(const Vector3f &w) const = 0;

    // Sample_wm: sample a microfacet normal
    virtual Vector3f Sample_wm(const Vector3f &w, Sampler &sampler) const = 0;

    // PDF(w, wm): probability density of the sampled normal
    virtual Float PDF(const Vector3f &w, const Vector3f &wm) const = 0;

    // G(wo, wi): uncorrelated masking-shadowing
    //   G = G1(wo) * G1(wi)
    Float G(const Vector3f &wo, const Vector3f &wi) const {
        return G1(wo) * G1(wi);
    }

    Float alpha() const { return m_alpha; }

protected:
    Float m_alpha;
};

// Beckmann microfacet distribution with Smith masking-shadowing.
//
// Formulae:
//   D(wm)      = exp(-tan2(theta) / a2) / (pi * a2 * cos4(theta))
//   G1(w)      = rational approximation (Walter et al. 2007)
//   Sample_wm  = non-visible normal sampling
//   PDF(w, wm) = D(wm) * cos(theta_m)
class BeckmannDistribution final : public MicrofacetDistribution {
public:
    explicit BeckmannDistribution(Float alpha)
        : MicrofacetDistribution(alpha) {}

    Float D(const Vector3f &wm) const override {
        using std::exp;
        using std::isinf;

        const Float cos_theta = wm[2];
        if (cos_theta <= Float0) return Float0;

        const Float cos2 = cos_theta * cos_theta;
        const Float cos4 = cos2 * cos2;
        if (cos4 < static_cast<Float>(1e-16)) return Float0;

        const Float tan2 = (wm[0]*wm[0] + wm[1]*wm[1]) / cos2;
        if (isinf(tan2)) return Float0;

        const Float a2 = m_alpha * m_alpha;
        return exp(-tan2 / a2) / (PI * a2 * cos4);
    }

    // Rational approximation for 1/(1 + Lambda(w)).
    // Error < 0.35% (Walter et al. 2007)
    Float G1(const Vector3f &w) const override {
        using std::sqrt;

        if (w[2] <= Float0) return Float0;

        const Float sin2 = w[0]*w[0] + w[1]*w[1];
        if (sin2 == Float0) return Float1;

        // a = 1 / (alpha * tan(theta))
        const Float a = w[2] / (sqrt(sin2) * m_alpha);
        if (a >= static_cast<Float>(1.6)) return Float1;

        const Float a2 = a * a;
        return (static_cast<Float>(3.535) * a + static_cast<Float>(2.181) * a2)
             / (Float1 + static_cast<Float>(2.276) * a + static_cast<Float>(2.577) * a2);
    }

    // Non-visible normal sampling
    Vector3f Sample_wm(const Vector3f & /*w*/, Sampler &sampler) const override {
        using std::atan;
        using std::sqrt;
        using std::log;
        using std::cos;
        using std::sin;

        const Float s1 = sampler.sample_1d();
        const Float s2 = sampler.sample_1d();

        const Float phi   = PI_2 * s1;
        const Float theta = atan(sqrt(-m_alpha * m_alpha * log(Float1 - s2)));

        return Vector3f{sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)};
    }

    // p(wm) = D(wm) * cos(theta_m)
    Float PDF(const Vector3f & /*w*/, const Vector3f &wm) const override {
        return D(wm) * std::max(Float0, wm[2]);
    }
};

} // namespace Caramel
