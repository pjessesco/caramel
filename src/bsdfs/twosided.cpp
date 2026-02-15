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

    // Flip z-component of a direction vector
    inline Vector3f flip_z(const Vector3f &v){
        return {v[0], v[1], -v[2]};
    }

    TwoSided::TwoSided(BSDF *front, BSDF *back) : m_front{front}, m_back{back} {}

    std::tuple<Vector3f, Vector3f, Float> TwoSided::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &uv, Sampler &sampler) const {
        const bool frontside = local_incoming_dir[2] < Float0;
        const Vector3f flipped_in = frontside ? local_incoming_dir : flip_z(local_incoming_dir);
        const BSDF *bsdf = frontside ? m_front : m_back;

        auto [out_dir, reflectance, pdf] = bsdf->sample_recursive_dir(flipped_in, uv, sampler);

        return {frontside ? out_dir : flip_z(out_dir), reflectance, pdf};
    }

    Float TwoSided::pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const {
        const bool frontside = local_incoming_dir[2] < Float0;
        return frontside ? m_front->pdf(local_incoming_dir, local_outgoing_dir) :
                           m_back->pdf(flip_z(local_incoming_dir), flip_z(local_outgoing_dir));
    }

    Vector3f TwoSided::get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &uv) const {
        const bool frontside = local_incoming_dir[2] < Float0;
        return frontside ? m_front->get_reflection(local_incoming_dir, local_outgoing_dir, uv) :
                           m_back->get_reflection(flip_z(local_incoming_dir), flip_z(local_outgoing_dir), uv);
    }

    bool TwoSided::is_discrete(bool frontside) const {
        return (frontside ? m_front : m_back)->is_discrete(true);
    }

}
