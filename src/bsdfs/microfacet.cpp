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

#include <bsdf.h>

#include <common.h>
#include <warp_sample.h>

namespace Caramel{
    Microfacet::Microfacet(Float alpha, Float in_ior, Float ex_ior, const Vector3f &kd)
        : m_alpha{alpha}, m_in_index_of_refraction{in_ior}, m_ex_index_of_refraction{ex_ior}, m_kd(kd), m_ks{Float1 - kd.max()} {}

    std::tuple<Vector3f, Vector3f, Float> Microfacet::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {
        // from hitpoint to incoming point
        const Vector3f local_incoming_flipped = -local_incoming_dir.normalize();
        Vector3f local_outgoing;

        if(local_incoming_flipped[2] <= Float0){
            // Not allow ray from backside
            return {vec3f_zero, vec3f_zero, Float0};
        }

        if(sampler.sample_1d() < m_ks){ // specular
            const Vector3f sampled_normal = sample_beckmann_distrib(sampler, m_alpha).first;
            local_outgoing = -local_incoming_flipped + static_cast<Float>(2) * local_incoming_flipped.dot(sampled_normal) * sampled_normal;
        }
        else{ // diffuse
            local_outgoing = sample_unit_hemisphere_cosine(sampler).first;
        }

        if(local_outgoing[2] <= Float0){
            // Not allow ray from backside
            return {vec3f_zero, vec3f_zero, Float0};
        }

        // pdf -------------
        const Float pdf_ = pdf(local_incoming_dir, local_outgoing);

        return {local_outgoing,
                get_reflection(local_incoming_dir, local_outgoing, Vector2f(/*dummy*/)) * local_outgoing[2] / pdf_,
                pdf_};
    }

    Float Microfacet::pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const{
        Vector3f local_incoming_flipped = -local_incoming_dir.normalize();
        Vector3f local_outgoing = local_outgoing_dir.normalize();

        if(local_incoming_flipped[2] <= Float0 || local_outgoing_dir[2] <= Float0){
            // Not allow ray from backside
            return Float0;
        }

        const Vector3f wh = Vector3f(local_incoming_flipped + local_outgoing).normalize();
        const Float Jh = Float1 / (static_cast<Float>(4) * wh.dot(local_outgoing));

        const Float pdf =  (m_ks * sample_beckmann_distrib_pdf(wh, m_alpha) * Jh) +
                          (Float1 - m_ks) * local_outgoing[2] * PI_INV;

        return pdf;
    }

    Vector3f Microfacet::get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const {
        // from hitpoint to incoming point
        const Vector3f local_incoming_flipped = -local_incoming_dir.normalize();
        const Vector3f local_outgoing = local_outgoing_dir.normalize();

        if(local_incoming_flipped[2] <= Float0 || local_outgoing_dir[2] <= Float0){
            // Not allow ray from backside
            return vec3f_zero;
        }

        const Vector3f wh = Vector3f(local_incoming_flipped + local_outgoing).normalize();
        const Float D = sample_beckmann_distrib_pdf(wh, m_alpha);
        const Float F = fresnel_dielectric(wh.dot(local_incoming_flipped), m_ex_index_of_refraction, m_in_index_of_refraction);
        const Float G = G1(local_incoming_flipped, wh) * G1(local_outgoing, wh);

        const Float denom = static_cast<Float>(4) * local_incoming_flipped[2] * local_outgoing[2] * wh[2];
        const Float spec_contrib = ((m_ks / denom) * D * F * G);
        return (m_kd * PI_INV) + Vector3f{spec_contrib, spec_contrib, spec_contrib};
    }

    bool Microfacet::is_discrete() const{
        return false;
    }

    Float Microfacet::G1(const Vector3f &wv, const Vector3f &wh) const{
        const Vector3f n = Vector3f{Float0, Float0, Float1};
        if(wv.dot(wh) / wv[2] <= 0){
            return Float0;
        }

        const Float b = Float1 / (m_alpha * wv[2]);
        if(b >= static_cast<Float>(1.6)){
            return Float1;
        }

        const Float b_2 = b * b;
        return (static_cast<Float>(3.535) * b + static_cast<Float>(2.181) * (b_2)) /
               (Float1 + static_cast<Float>(2.276) * b + static_cast<Float>(2.577) * b_2);
    }

}