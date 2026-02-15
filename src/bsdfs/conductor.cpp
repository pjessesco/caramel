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
#include <logger.h>
#include <warp_sample.h>

namespace Caramel{

    Conductor::Conductor(const Conductors &mat, Float ex_ior)
    : m_ex_ior{ex_ior} {
        if(!IOR::eta_map.contains(mat) || !IOR::k_map.contains(mat)){
            CRM_ERROR("Given material is not found");
        }
        m_in_ior = IOR::eta_map.find(mat)->second;
        m_in_ior_img = IOR::k_map.find(mat)->second;
    }

    std::tuple<Vector3f, Vector3f, Float> Conductor::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {
        const Vector3f local_outgoing{local_incoming_dir[0], local_incoming_dir[1], -local_incoming_dir[2]};
        const Vector3f n{Float0, Float0, Float1};
        const Float local_incoming_cos = n.dot(-local_incoming_dir);

        if(local_incoming_cos <= Float0){
            return {local_outgoing, vec3f_one, Float0};
        }

        return {local_outgoing, fresnel_conductor(local_incoming_cos, {m_ex_ior, m_ex_ior, m_ex_ior}, m_in_ior, m_in_ior_img), Float0};
    }

    Float Conductor::pdf(const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    Vector3f Conductor::get_reflection(const Vector3f &, const Vector3f &, const Vector2f &) const {
        return vec3f_zero;
    }

    bool Conductor::is_discrete(bool /*frontside*/) const {
        return true;
    }


}