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
#include <textures.h>

namespace Caramel{
    Diffuse::Diffuse(const Vector3f &albedo)
        : m_albedo{albedo}, m_texture{nullptr} {}

    Diffuse::Diffuse(Texture *texture)
        : m_albedo{vec3f_zero}, m_texture{texture} {}

    Diffuse::~Diffuse(){
        delete m_texture;
    }

    std::tuple<Vector3f, Vector3f, Float> Diffuse::sample_recursive_dir(const Vector3f &, const Vector2f &uv, Sampler &sampler) const {
        auto [local_outgoing, dir_pdf] = sample_unit_hemisphere_cosine(sampler);
        return {local_outgoing, m_texture == nullptr ? m_albedo : m_texture->get_val(uv), dir_pdf};
    }

    Float Diffuse::pdf(const Vector3f &, const Vector3f &local_outgoing_dir) const{
        return sample_unit_hemisphere_cosine_pdf(local_outgoing_dir);
    }

    Vector3f Diffuse::get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &uv) const {
        // from hitpoint to incoming point
        const Vector3f local_incoming_flipped = -local_incoming_dir.normalize();

        if(local_incoming_flipped[2] <= Float0 || local_outgoing_dir[2] <= Float0){
            // Not allow ray from backside
            return vec3f_zero;
        }

        return (m_texture == nullptr ? m_albedo : m_texture->get_val(uv)) * PI_INV;
    }

    bool Diffuse::is_discrete() const{
        return false;
    }

}