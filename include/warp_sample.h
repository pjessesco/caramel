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

#pragma once

#include <cmath>
#include <tuple>

#include <sampler.h>
#include <common.h>

namespace Caramel{

    // ----------------------------------------------

    inline std::pair<Vector2f, Float> sample_unit_disk_uniformly(Sampler &sampler){
        const Float sqrt_x = std::sqrt(sampler.sample_1d());
        const Float angle = sampler.sample_1d() * PI_2;
        return {{sqrt_x * cos(angle), sqrt_x * sin(angle)},
                 PI_INV};
    }

    inline Float sample_unit_disk_uniformly_pdf(const Vector2f &vec){
        return vec.length() >= Float1 ? Float0 : PI_INV;
    }

    // ----------------------------------------------

    inline std::pair<Vector3f, Float> sample_unit_sphere_uniformly(Sampler &sampler){
        const Float phi = PI_2 * sampler.sample_1d();
        const Float theta = std::acos(Float1 - 2 * sampler.sample_1d());

        const Float sin_theta = std::sin(theta);
        const Float cos_theta = std::cos(theta);
        const Float sin_phi = std::sin(phi);
        const Float cos_phi = std::cos(phi);

        return {{sin_theta * cos_phi, sin_theta * sin_phi, cos_theta},
                PI_4_INV};
    }

    // `sample_unit_sphere_uniformly()` does not sample
    // inside of the sphere, so no need to check vector's length.
    inline Float sample_unit_sphere_uniformly_pdf(const Vector3f &vec){
        return PI_4_INV;
    }

    // ----------------------------------------------

    inline std::pair<Vector3f, Float> sample_unit_hemisphere_uniformly(Sampler &sampler){
        const Float phi = PI_2 * sampler.sample_1d();
        const Float theta = std::acos(Float1 - sampler.sample_1d());

        const Float sin_theta = std::sin(theta);
        const Float cos_theta = std::cos(theta);
        const Float sin_phi = std::sin(phi);
        const Float cos_phi = std::cos(phi);

        return {{sin_theta * cos_phi, sin_theta * sin_phi, cos_theta},
                PI_2_INV};
    }

    inline Float sample_unit_hemisphere_uniformly_pdf(const Vector3f &vec){
        return vec[2] <= 0 ? Float0 : PI_2_INV;
    }

    // ----------------------------------------------

    inline std::pair<Vector3f, Float> sample_unit_hemisphere_cosine(Sampler &sampler){
        auto [xy, _] = sample_unit_disk_uniformly(sampler);
        const Float z = std::sqrt(Float1 - xy.dot(xy));
        return {{xy[0], xy[1], z},
                z*PI_INV};
    }

    inline Float sample_unit_hemisphere_cosine_pdf(const Vector3f &vec){
        return vec[2] <= 0 ? Float0 : vec[2]*PI_INV;
    }

}










