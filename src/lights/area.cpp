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

#include <tuple>

#include <common.h>
#include <light.h>
#include <sampler.h>
#include <scene.h>
#include <shape.h>

namespace Caramel{
    AreaLight::AreaLight(const Scene &scene, const Vector3f &radiance)
        : Light(scene), m_radiance{radiance} {}

    Vector3f AreaLight::radiance() const{
        return m_radiance;
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float> AreaLight::sample_contribution(const Vector3f &hitpos, Sampler &sampler){
        // Sample point on the shape
        const auto [light_pos, light_normal_world, pos_pdf] = m_shape.lock()->sample_point(sampler);
        const Vector3f light_to_hitpos = hitpos - light_pos;

        // If hitpoint is behind of a sampled point, zero contribution
        if(light_normal_world.dot(light_to_hitpos) <= 0){
            return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf};
        }

        // If hitpoint and sampled point is not visible to each other, zero contribution
        if(!m_scene.is_visible(hitpos, light_pos)){
            return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf};
        }

        return {m_radiance, light_pos, light_normal_world, pos_pdf};
    }


}