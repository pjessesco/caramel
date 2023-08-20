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

    Conductor::Conductor(const Vector3f &eta_i, const Vector3f &eta_t, const Vector3f &eta_t_img)
    : m_in_ior{eta_i}, m_ex_ior{eta_t}, m_ex_ior_img{eta_t_img} {}

    std::tuple<Vector3f, Vector3f, Float> Conductor::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {
        
    }

    Float Conductor::pdf(const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    Vector3f Conductor::get_reflection(const Vector3f &, const Vector3f &, const Vector2f &) const {
        return vec3f_zero;
    }

    bool Conductor::is_discrete() const {
        return true;
    }


}