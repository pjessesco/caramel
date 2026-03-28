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

#include <bsdf.h>

#include <common.h>

namespace Caramel{

    // When the roughness alpha is below this threshold, the surface is
    // treated as perfectly smooth (delta BRDF) to avoid numerical issues
    // with the microfacet distribution at near-zero roughness.
    static bool effectively_smooth(Float alpha) {
        return alpha < static_cast<Float>(1e-3);
    }

    // Look up the complex IOR (eta + i*k) for the given conductor material
    // from the built-in IOR tables.
    RoughConductor::RoughConductor(Float alpha, const Conductors &mat, Float ex_ior)
        : m_distrib{alpha}, m_ex_ior{ex_ior} {
        m_eta = IOR::eta_map.find(mat)->second;
        m_k = IOR::k_map.find(mat)->second;
    }

    // Evaluate the microfacet conductor BRDF for the given direction pair.
    //
    // The Cook-Torrance microfacet BRDF for conductors is:
    //   f_r(wo, wi) = D(wm) * F(cos_theta_d, eta, k) * G(wo, wi) / (4 * cos_o * cos_i)
    // where:
    //   wm = normalize(wo + wi)       -- the microfacet (half-vector) normal
    //   D  = GGX normal distribution  -- probability density of microfacet normals
    //   F  = conductor Fresnel         -- per-channel reflectance using complex IOR
    //   G  = height-correlated Smith   -- joint masking-shadowing function
    //
    // Returns zero for the effectively-smooth case (handled as delta in sampling)
    // and when either direction is below the surface (conductors are opaque).
    //
    // Direction convention:
    //   wo = -local_incoming_dir  (points away from surface, toward viewer)
    //   wi = local_outgoing_dir   (points away from surface, toward light)
    Vector3f RoughConductor::get_reflection(const Vector3f &local_incoming_dir,
                                            const Vector3f &local_outgoing_dir,
                                            const Vector2f &) const {
        if (effectively_smooth(m_distrib.alpha())) return vec3f_zero;

        const Vector3f wo = -local_incoming_dir.normalize();
        const Vector3f wi = local_outgoing_dir.normalize();

        // Conductors are opaque: both directions must be in the upper hemisphere
        if (wo[2] <= Float0 || wi[2] <= Float0) return vec3f_zero;

        // Half-vector (microfacet normal)
        Vector3f wm = Vector3f(wo + wi);
        if (wm.dot(wm) == Float0) return vec3f_zero;
        wm = wm.normalize();

        const Float D = m_distrib.D(wm);
        const Float G = m_distrib.G(wo, wi);
        // Conductor Fresnel is evaluated at the angle between wo and the microfacet
        // normal wm, not the macro surface normal. This gives per-channel (R,G,B) reflectance.
        const Vector3f F = fresnel_conductor(
            std::abs(wo.dot(wm)),
            {m_ex_ior, m_ex_ior, m_ex_ior}, m_eta, m_k);

        const Float denom = static_cast<Float>(4) * wo[2] * wi[2];
        return F * (D * G / denom);
    }

    // Evaluate the sampling PDF for the given direction pair.
    //
    // We use visible-normal sampling (VNDF), so the PDF of sampling a
    // microfacet normal wm given wo is D_vis(wm, wo). To convert from the
    // microfacet normal PDF to the reflected direction PDF, we apply the
    // reflection Jacobian:
    //   pdf(wi) = pdf_wm(wm) / (4 * |wo . wm|)
    //
    // The factor 4*|wo.wm| comes from the Jacobian of the half-vector
    // mapping: d(wm) / d(wi) = 1 / (4 * |wo . wm|) for reflection.
    Float RoughConductor::pdf(const Vector3f &local_incoming_dir,
                              const Vector3f &local_outgoing_dir) const {
        if (effectively_smooth(m_distrib.alpha())) return Float0;

        const Vector3f wo = -local_incoming_dir.normalize();
        const Vector3f wi = local_outgoing_dir.normalize();

        if (wo[2] <= Float0 || wi[2] <= Float0) return Float0;

        Vector3f wm = Vector3f(wo + wi);
        if (wm.dot(wm) == Float0) return Float0;
        wm = wm.normalize();

        return m_distrib.PDF(wo, wm) / (static_cast<Float>(4) * std::abs(wo.dot(wm)));
    }

    // Sample an outgoing direction from the microfacet conductor BRDF.
    //
    // Sampling strategy:
    //   1. Sample a microfacet normal wm from the visible normal distribution (VNDF).
    //   2. Reflect wo around wm to get the outgoing direction wi.
    //   3. Evaluate the full BRDF and PDF, then return weight = f * cos(theta_i) / pdf.
    //
    // For effectively-smooth surfaces (alpha < 1e-3), we bypass microfacet
    // sampling entirely and return a perfect mirror reflection weighted by
    // the conductor Fresnel term. The PDF is set to 0 to signal a delta
    // distribution to the integrator.
    //
    // Returns: {wi, weight, pdf}
    //   wi     -- sampled outgoing direction (upper hemisphere)
    //   weight -- BRDF * cos(theta_i) / pdf (per-channel, includes Fresnel color)
    //   pdf    -- sampling probability density (0 for delta distributions)
    std::tuple<Vector3f, Vector3f, Float> RoughConductor::sample_recursive_dir(
        const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {

        const Vector3f wo = -local_incoming_dir.normalize();
        if (wo[2] <= Float0) return {vec3f_zero, vec3f_zero, Float0};

        // Effectively-smooth fallback: perfect mirror reflection with conductor Fresnel
        if (effectively_smooth(m_distrib.alpha())) {
            const Vector3f wi{-wo[0], -wo[1], wo[2]};
            const Vector3f F = fresnel_conductor(
                wo[2], {m_ex_ior, m_ex_ior, m_ex_ior}, m_eta, m_k);
            // PDF = 0 signals a delta distribution; weight = F (Fresnel reflectance)
            return {wi, F, Float0};
        }

        // Step 1: Sample microfacet normal from visible normal distribution
        Vector3f wm = m_distrib.Sample_wm(wo, sampler);
        // Step 2: Reflect wo around the sampled microfacet normal
        Vector3f wi = reflect(Vector3f(-wo), wm);

        // Reject if the reflected direction goes below the surface
        if (wi[2] <= Float0) return {vec3f_zero, vec3f_zero, Float0};

        // Step 3: Compute PDF with reflection Jacobian
        const Float pdf_ = m_distrib.PDF(wo, wm) /
                           (static_cast<Float>(4) * std::abs(wo.dot(wm)));
        if (pdf_ <= Float0) return {vec3f_zero, vec3f_zero, Float0};

        // Step 4: Evaluate the full BRDF value
        const Vector3f F = fresnel_conductor(
            std::abs(wo.dot(wm)),
            {m_ex_ior, m_ex_ior, m_ex_ior}, m_eta, m_k);
        const Float D = m_distrib.D(wm);
        const Float G = m_distrib.G(wo, wi);
        const Float denom = static_cast<Float>(4) * wo[2] * wi[2];

        // f_r = F * D * G / (4 * cos_o * cos_i)
        // Return weight = f_r * cos(theta_i) / pdf for Monte Carlo integration
        const Vector3f f = F * (D * G / denom);
        return {wi, f * wi[2] / pdf_, pdf_};
    }

    // Returns true when the BRDF acts as a delta distribution (perfect mirror).
    // The integrator uses this to skip PDF evaluation for delta BSDFs.
    bool RoughConductor::is_discrete(bool /*frontside*/) const {
        return effectively_smooth(m_distrib.alpha());
    }

}
