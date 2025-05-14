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

#include <tuple>

#include <light.h>

#include <ray.h>
#include <common.h>
#include <sampler.h>
#include <scene.h>
#include <shape.h>
#include <rayintersectinfo.h>

namespace Caramel{
    AreaLight::AreaLight(const Vector3f &radiance)
        : m_radiance{radiance} {}

    AreaLight::~AreaLight() = default;

    Vector3f AreaLight::radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &light_normal_world) const{
        return Peanut::select(light_normal_world.dot(hitpos - lightpos) > 0, m_radiance, vec3f_zero);
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> AreaLight::sample_direct_contribution(const Scene &scene, const RayIntersectInfo &hitpos_info, Sampler &sampler) const{
        // Sample point on the shape
        const auto [light_pos, light_normal_world, pos_pdf] = m_shape->sample_point(sampler);
        const Vector3f light_to_hitpos = hitpos_info.p - light_pos;


        Bool cond = light_normal_world.dot(light_to_hitpos) > 0;
        if (pn_none_of(cond)) {
            return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf, RayIntersectInfo()};
        }

        // If hitpoint and sampled point is not visible to each other, zero contribution
        auto [is_visible, info] = scene.is_visible(hitpos_info.p, light_pos);
        cond &= is_visible;


        return {Peanut::select(cond, m_radiance, vec3f_zero),
                Peanut::select(cond, light_pos, vec3f_zero),
                Peanut::select(cond, light_normal_world, vec3f_zero),
                pos_pdf,
                Peanut::select(cond, info, RayIntersectInfo())};
    }

    Float AreaLight::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const{
        return m_shape->pdf_solidangle(hitpos_world, lightpos_world, light_normal_world);
    }

    bool AreaLight::is_delta() const {
        return false;
    }

    bool AreaLight::is_envlight() const {
        return false;
    }

}
