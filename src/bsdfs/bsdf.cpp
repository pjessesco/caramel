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

#include <common.h>
#include <bsdf.h>
#include <warp_sample.h>
#include <coordinate.h>

namespace Caramel{

    // Snell's Law : eta_i * sin_i = eta_t * sin_t
    // Calculate `sin_t` using above equation.
    Float snell_get_sin_t(Float sin_i, Float eta_i, Float eta_t) {
        assert(sin_i >= Float0);
        return eta_i * sin_i / eta_t;
    }

    // Calculate fresnel reflectance for unpolarized light.
    Float fresnel_dielectric(Float cos_i, Float eta_i, Float eta_t) {
        assert(Float0 <= cos_i && cos_i <= Float1);

        const Float sin_i = sqrt(Float1 - (cos_i * cos_i));
        const Float sin_t = snell_get_sin_t(sin_i, eta_i, eta_t);

        // Total reflection
        if(sin_t >= Float1) return Float1;

        const Float cos_t = sqrt(Float1 - (sin_t * sin_t));

        const Float eta_t_cos_t = eta_t * cos_t;
        const Float eta_t_cos_i = eta_t * cos_i;
        const Float eta_i_cos_t = eta_i * cos_t;
        const Float eta_i_cos_i = eta_i * cos_i;

        const Float r_parallel = (eta_t_cos_i - eta_i_cos_t) / (eta_t_cos_i + eta_i_cos_t);
        const Float r_perpendicular = (eta_i_cos_i - eta_t_cos_t) / (eta_i_cos_i + eta_t_cos_t);

        return (r_parallel * r_parallel + r_perpendicular * r_perpendicular) * static_cast<Float>(0.5);
    }


}