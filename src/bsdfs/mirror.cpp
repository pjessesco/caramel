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
    Mirror::Mirror() = default;

    std::tuple<Vector3f, Vector3f, Float> Mirror::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &) const {
        const Vector3f local_outgoing{local_incoming_dir[0], local_incoming_dir[1], -local_incoming_dir[2]};
        return {local_outgoing, vec3f_one, Float0};
    }

    Float Mirror::pdf(const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    Vector3f Mirror::get_reflection(const Vector3f &, const Vector3f &, const Vector2f &) const {
        return vec3f_zero;
    }

    bool Mirror::is_discrete() const{
        return true;
    }

}