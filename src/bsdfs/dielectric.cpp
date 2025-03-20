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
#include <warp_sample.h>

namespace Caramel{

    Dielectric::Dielectric(Float in_ior, Float ex_ior) : m_in_index_of_refraction{in_ior}, m_ex_index_of_refraction{ex_ior} {}

    std::tuple<Vector3f, Vector3f, Float> Dielectric::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {
        // Multiply -1 to calculate cosine, since we use incoming direction as point-toward direction
        // Values varies on `local_incoming_cos`
        Vector3f n{Float0, Float0, Float1};
        Float local_incoming_cos = n.dot(-local_incoming_dir);

        Float ex_ior = m_ex_index_of_refraction;
        Float in_ior = m_in_index_of_refraction;

        // Flip internal <-> external if ray incidents from back
        if(local_incoming_cos < Float0){
            in_ior = m_ex_index_of_refraction;
            ex_ior = m_in_index_of_refraction;
            n = -n;
            local_incoming_cos *= -1;
        }

        const Float reflect_ratio = fresnel_dielectric(local_incoming_cos, ex_ior, in_ior);

        if(sampler.sample_1d() <= reflect_ratio){
            const Vector3f local_outgoing_dir{local_incoming_dir[0], local_incoming_dir[1], -local_incoming_dir[2]};
            return {local_outgoing_dir, vec3f_one, Float0};
        }
        else{
            const Vector3f local_outgoing_dir = refract(local_incoming_dir, n, in_ior, ex_ior);
            // TODO : consider https://www.pbr-book.org/4ed/Reflection_Models/Dielectric_BSDF#Non-SymmetricScatteringandRefraction
            const Float tmp = ((ex_ior*ex_ior)/(in_ior*in_ior));
            return {local_outgoing_dir, {tmp, tmp, tmp}, Float0};
        }

    }

    Float Dielectric::pdf(const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    Vector3f Dielectric::get_reflection(const Vector3f &, const Vector3f &, const Vector2f &) const {
        return vec3f_zero;
    }

    bool Dielectric::is_discrete() const {
        return true;
    }


}