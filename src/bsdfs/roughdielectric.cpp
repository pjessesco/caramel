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
    // treated as perfectly smooth (delta BSDF) to avoid numerical issues
    // with the microfacet distribution at near-zero roughness.
    static bool effectively_smooth(Float alpha) {
        return alpha < static_cast<Float>(1e-3);
    }

    // Pre-compute eta = in_ior / ex_ior, which is the relative IOR used
    // for half-vector computation and non-symmetric scattering correction.
    RoughDielectric::RoughDielectric(Float alpha, Float in_ior, Float ex_ior)
        : m_distrib{alpha}, m_in_ior{in_ior}, m_ex_ior{ex_ior},
          m_eta{in_ior / ex_ior} {}

    // Evaluate the microfacet dielectric BSDF for the given direction pair.
    //
    // This function handles both reflection and transmission:
    //
    //   Reflection (wo and wi on the same side):
    //     f_r = D(wm) * F * G(wo, wi) / (4 * |cos_o| * |cos_i|)
    //     Half-vector: wm = normalize(wo + wi)
    //
    //   Transmission (wo and wi on opposite sides):
    //     f_t = D(wm) * (1-F) * G * |dot(wi,wm)| * |dot(wo,wm)|
    //           / (|cos_o| * |cos_i| * denom^2)
    //     where denom = dot(wi, wm) + dot(wo, wm) / eta'
    //     Generalized half-vector: wm = normalize(eta' * wi + wo)
    //     eta' = in_ior/ex_ior when wo is in the upper hemisphere, inverted otherwise.
    //
    // The 1/eta'^2 non-symmetric scattering correction is applied to the
    // transmission term for radiance transport (ensures reciprocity when
    // radiance crosses a refractive boundary).
    //
    // Direction convention:
    //   wo = -local_incoming_dir  (points away from surface, toward viewer)
    //   wi = local_outgoing_dir   (points away from surface, toward light)
    //   wo[2] > 0 means the viewer is outside (external medium)
    Vector3f RoughDielectric::get_reflection(const Vector3f &local_incoming_dir,
                                             const Vector3f &local_outgoing_dir,
                                             const Vector2f &) const {
        if (effectively_smooth(m_distrib.alpha())) return vec3f_zero;

        const Vector3f wo = -local_incoming_dir.normalize();
        const Vector3f wi = local_outgoing_dir.normalize();
        const Float cos_o = wo[2];
        const Float cos_i = wi[2];

        if (cos_o == Float0 || cos_i == Float0) return vec3f_zero;

        // Determine whether this is reflection or transmission by checking
        // if wo and wi are on the same hemisphere (same sign of z-component).
        const bool is_reflect = (cos_i * cos_o > Float0);

        // eta' is the relative IOR from the medium containing wo to the medium
        // containing wi. For reflection, it is not used in the half-vector but
        // we keep it at 1 for clarity.
        Float etap = Float1;
        if (!is_reflect)
            etap = cos_o > Float0 ? m_eta : (Float1 / m_eta);

        // Compute the generalized half-vector.
        // For reflection: wm = normalize(wo + wi)
        // For transmission: wm = normalize(eta' * wi + wo)
        //   The eta' scaling accounts for the change in solid angle across
        //   the refractive interface (Snell's law in differential form).
        Vector3f wm;
        if (is_reflect) {
            wm = Vector3f(wi + wo);
        } else {
            wm = Vector3f(wi * etap + wo);
        }
        if (wm.dot(wm) == Float0) return vec3f_zero;
        wm = wm.normalize();
        // By convention, the microfacet normal should point into the upper hemisphere
        if (wm[2] < Float0) wm = -wm;

        // Backfacing microfacet check: the microfacet is only valid if both
        // wo and wi are on the correct side of the microfacet plane.
        if (wm.dot(wi) * cos_i < Float0 || wm.dot(wo) * cos_o < Float0)
            return vec3f_zero;

        // Evaluate Fresnel reflectance at the angle between wo and the microfacet normal
        const Float F = fresnel_dielectric(std::abs(wo.dot(wm)), m_ex_ior, m_in_ior);
        const Float D = m_distrib.D(wm);
        const Float G = m_distrib.G(wo, wi);

        if (is_reflect) {
            // Standard Cook-Torrance reflection BRDF
            const Float val = D * F * G /
                (static_cast<Float>(4) * std::abs(cos_i) * std::abs(cos_o));
            return {val, val, val};
        } else {
            // Microfacet transmission BTDF
            // The denominator includes the generalized half-vector Jacobian term
            // denom = dot(wi, wm) + dot(wo, wm) / eta'
            const Float denom_sqr = wi.dot(wm) + wo.dot(wm) / etap;
            Float ft = D * (Float1 - F) * G *
                       std::abs(wi.dot(wm) * wo.dot(wm)) /
                       (std::abs(cos_i) * std::abs(cos_o) * denom_sqr * denom_sqr);
            // Non-symmetric scattering correction for radiance transport:
            // When a ray crosses a refractive boundary, the radiance changes by
            // a factor of 1/eta'^2 due to the change in solid angle measure.
            ft /= (etap * etap);
            return {ft, ft, ft};
        }
    }

    // Evaluate the sampling PDF for the given direction pair.
    //
    // The PDF is a mixture of reflection and transmission PDFs, weighted by the
    // Fresnel reflectance R and transmittance T = 1-R at the microfacet normal:
    //   pdf = P(reflect) * pdf_reflect + P(transmit) * pdf_transmit
    //
    // Reflection PDF (same Jacobian as RoughConductor):
    //   pdf_reflect = pdf_wm(wm) / (4 * |wo . wm|) * R
    //
    // Transmission PDF (uses the generalized half-vector Jacobian):
    //   pdf_transmit = pdf_wm(wm) * |wi . wm| / denom^2 * T
    //   where denom = dot(wi, wm) + dot(wo, wm) / eta'
    //
    // The Jacobian |wi . wm| / denom^2 converts from microfacet normal space
    // to transmitted direction space, accounting for the solid angle compression
    // due to refraction.
    Float RoughDielectric::pdf(const Vector3f &local_incoming_dir,
                               const Vector3f &local_outgoing_dir) const {
        if (effectively_smooth(m_distrib.alpha())) return Float0;

        const Vector3f wo = -local_incoming_dir.normalize();
        const Vector3f wi = local_outgoing_dir.normalize();
        const Float cos_o = wo[2];
        const Float cos_i = wi[2];

        if (cos_o == Float0 || cos_i == Float0) return Float0;

        const bool is_reflect = (cos_i * cos_o > Float0);

        Float etap = Float1;
        if (!is_reflect)
            etap = cos_o > Float0 ? m_eta : (Float1 / m_eta);

        // Compute generalized half-vector (same logic as get_reflection)
        Vector3f wm;
        if (is_reflect) {
            wm = Vector3f(wi + wo);
        } else {
            wm = Vector3f(wi * etap + wo);
        }
        if (wm.dot(wm) == Float0) return Float0;
        wm = wm.normalize();
        if (wm[2] < Float0) wm = -wm;

        // Backfacing microfacet check
        if (wm.dot(wi) * cos_i < Float0 || wm.dot(wo) * cos_o < Float0)
            return Float0;

        // Fresnel reflectance determines the lobe selection probability
        const Float R = fresnel_dielectric(std::abs(wo.dot(wm)), m_ex_ior, m_in_ior);
        const Float T = Float1 - R;

        if (is_reflect) {
            // Reflection Jacobian: d(wm)/d(wi) = 1 / (4 * |wo . wm|)
            return m_distrib.PDF(wo, wm) /
                   (static_cast<Float>(4) * std::abs(wo.dot(wm)))
                   * R / (R + T);
        } else {
            // Transmission Jacobian: d(wm)/d(wi) = |wi . wm| / denom^2
            const Float denom = wi.dot(wm) + wo.dot(wm) / etap;
            const Float dwm_dwi = std::abs(wi.dot(wm)) / (denom * denom);
            return m_distrib.PDF(wo, wm) * dwm_dwi * T / (R + T);
        }
    }

    // Sample an outgoing direction from the microfacet dielectric BSDF.
    //
    // Sampling strategy:
    //   1. Sample a microfacet normal wm from the visible normal distribution (VNDF).
    //   2. Evaluate the Fresnel reflectance R at the angle between wo and wm.
    //   3. With probability R/(R+T), reflect wo around wm (reflection lobe).
    //      With probability T/(R+T), refract wo through wm (transmission lobe).
    //   4. Evaluate the full BSDF and PDF for the chosen lobe.
    //
    // Effectively-smooth fallback (alpha < 1e-3):
    //   Bypasses microfacet sampling entirely and behaves as a perfect delta
    //   dielectric: specular reflection or Snell's law refraction, chosen
    //   by Fresnel probability. Handles front/back incidence by flipping
    //   the normal and IOR values when the ray comes from inside.
    //
    // Transmission refraction:
    //   Uses Snell's law around the microfacet normal wm (not the macro normal).
    //   The refracted direction is:
    //     wi = eta_ratio * (-wo) + (eta_ratio * cos(theta_i) - cos(theta_t)) * wm
    //   where eta_ratio = n_incident / n_transmitted.
    //   Total internal reflection (sin^2(theta_t) >= 1) is rejected.
    //
    // Returns: {wi, weight, pdf}
    //   wi     -- sampled outgoing direction
    //   weight -- BSDF * |cos(theta_i)| / pdf (achromatic for dielectrics)
    //   pdf    -- sampling probability density (0 for delta distributions)
    std::tuple<Vector3f, Vector3f, Float> RoughDielectric::sample_recursive_dir(
        const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {

        const Vector3f wo = -local_incoming_dir.normalize();

        // --- Effectively-smooth delta fallback ---
        // When alpha is near zero, treat the surface as a perfect dielectric
        // interface (specular reflection + Snell refraction).
        if (effectively_smooth(m_distrib.alpha())) {
            Vector3f n{Float0, Float0, Float1};
            Float cos_theta_o = wo[2];
            Float ex_ior = m_ex_ior;
            Float in_ior = m_in_ior;

            // If wo is in the lower hemisphere, the ray is coming from inside
            // the medium. Flip the normal and swap IOR values.
            if (cos_theta_o < Float0) {
                ex_ior = m_in_ior;
                in_ior = m_ex_ior;
                n = -n;
                cos_theta_o = -cos_theta_o;
            }

            const Float R = fresnel_dielectric(cos_theta_o, ex_ior, in_ior);

            if (sampler.sample_1d() <= R) {
                // Perfect specular reflection
                const Vector3f wi{-wo[0], -wo[1], wo[2]};
                return {wi, vec3f_one, Float0};
            } else {
                // Perfect Snell refraction through the macro surface normal
                const Vector3f neg_wo = -wo;
                const Vector3f wi = refract(neg_wo, n, in_ior, ex_ior);
                // Weight accounts for the solid angle compression: (n_out/n_in)^2
                const Float tmp = (ex_ior * ex_ior) / (in_ior * in_ior);
                return {wi, {tmp, tmp, tmp}, Float0};
            }
        }

        // --- Rough surface sampling ---

        // Step 1: Sample a microfacet normal from the visible normal distribution
        Vector3f wm = m_distrib.Sample_wm(wo, sampler);

        // Step 2: Evaluate Fresnel to decide between reflection and transmission
        const Float R = fresnel_dielectric(std::abs(wo.dot(wm)), m_ex_ior, m_in_ior);
        const Float T = Float1 - R;

        if (sampler.sample_1d() < R / (R + T)) {
            // --- Reflection lobe ---
            // Reflect wo around the microfacet normal wm
            Vector3f wi = reflect(Vector3f(-wo), wm);
            // Reject if reflected direction is on the wrong side of the macro surface
            if (wi[2] * wo[2] <= Float0)
                return {vec3f_zero, vec3f_zero, Float0};

            // PDF = VNDF_pdf / (4 * |wo.wm|) * R/(R+T)
            const Float pdf_ = m_distrib.PDF(wo, wm) /
                               (static_cast<Float>(4) * std::abs(wo.dot(wm)))
                               * R / (R + T);
            if (pdf_ <= Float0) return {vec3f_zero, vec3f_zero, Float0};

            // BRDF = D * F * G / (4 * |cos_i| * |cos_o|)
            const Float D = m_distrib.D(wm);
            const Float G = m_distrib.G(wo, wi);
            const Float f = D * R * G /
                            (static_cast<Float>(4) * std::abs(wi[2]) * std::abs(wo[2]));

            // weight = f * |cos_i| / pdf for Monte Carlo integration
            const Vector3f weight{f, f, f};
            return {wi, weight * std::abs(wi[2]) / pdf_, pdf_};
        } else {
            // --- Transmission lobe ---

            // eta' = in_ior/ex_ior when wo is outside (wo.wm > 0), inverted otherwise.
            // eta_ratio = n_incident / n_transmitted (for Snell's law around wm).
            Float etap = (wo.dot(wm) > Float0) ? m_eta : (Float1 / m_eta);
            Float eta_ratio = (wo.dot(wm) > Float0)
                ? (m_ex_ior / m_in_ior)
                : (m_in_ior / m_ex_ior);

            // Refract wo through the microfacet normal wm using Snell's law:
            //   wi = eta_ratio * (-wo) + (eta_ratio * cos_i - cos_t) * wm
            // where cos_i = dot(wo, wm) and cos_t = sqrt(1 - sin^2_t)
            Float cos_theta_i = wo.dot(wm);
            Float sin2_t = eta_ratio * eta_ratio * (Float1 - cos_theta_i * cos_theta_i);
            if (sin2_t >= Float1)
                return {vec3f_zero, vec3f_zero, Float0}; // Total internal reflection
            Float cos_t = std::sqrt(Float1 - sin2_t);
            Vector3f wi = Vector3f(eta_ratio * Vector3f(-wo) + (eta_ratio * cos_theta_i - cos_t) * wm);

            // Transmitted direction must be on the opposite side of the macro surface
            if (wi[2] * wo[2] >= Float0 || wi[2] == Float0)
                return {vec3f_zero, vec3f_zero, Float0};

            // PDF = VNDF_pdf * |wi.wm| / denom^2 * T/(R+T)
            // where denom = dot(wi, wm) + dot(wo, wm) / eta'
            const Float denom = wi.dot(wm) + wo.dot(wm) / etap;
            const Float dwm_dwi = std::abs(wi.dot(wm)) / (denom * denom);
            const Float pdf_ = m_distrib.PDF(wo, wm) * dwm_dwi * T / (R + T);
            if (pdf_ <= Float0) return {vec3f_zero, vec3f_zero, Float0};

            // BTDF = D * (1-F) * G * |wi.wm| * |wo.wm| / (|cos_i| * |cos_o| * denom^2)
            const Float D = m_distrib.D(wm);
            const Float G = m_distrib.G(wo, wi);
            Float ft = D * T * G *
                       std::abs(wi.dot(wm) * wo.dot(wm)) /
                       (std::abs(wi[2]) * std::abs(wo[2]) * denom * denom);
            // Non-symmetric scattering correction for radiance transport:
            // accounts for the solid angle change across the refractive boundary.
            ft /= (etap * etap);

            // weight = f_t * |cos_i| / pdf for Monte Carlo integration
            const Vector3f weight{ft, ft, ft};
            return {wi, weight * std::abs(wi[2]) / pdf_, pdf_};
        }
    }

    // Returns true when the BSDF acts as a delta distribution (perfect specular).
    // The integrator uses this to skip PDF evaluation for delta BSDFs.
    bool RoughDielectric::is_discrete(bool /*frontside*/) const {
        return effectively_smooth(m_distrib.alpha());
    }

}
