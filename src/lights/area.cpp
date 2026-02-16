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

    Float AreaLight::power() const {
        // Integrate radiance over hemisphere and area, one-sided
        //     radiance = d^2Watt / (dSolidAngle * cos * dArea)
        //     d^2Watt = radiance * dSolidAngle * cos * dArea
        //     Watt = radiance * (integral over hemisphere (integral over area (cos) dArea) dSolidAngle)
        //     Watt = radiance * (integral over hemisphere (cos * integral over area dArea) dSolidAngle)
        //     Watt = radiance * area * (integral over hemisphere (cos) dSolidAngle)   (integral over area dArea == area)
        //     Watt = radiance * area * pi
        return luminance(m_radiance) * m_shape->get_area() * PI;
    }

    Vector3f AreaLight::radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &light_normal_world) const{
        if(light_normal_world.dot(hitpos - lightpos) <= 0){
            return vec3f_zero;
        }
        return m_radiance;
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float> AreaLight::sample_direct_contribution(const Scene &scene, const RayIntersectInfo &hitpos_info, Sampler &sampler) const{
        // Sample point on the shape
        const auto [light_pos, light_normal_world, pos_pdf] = m_shape->sample_point(sampler);
        const Vector3f light_to_hitpos = hitpos_info.p - light_pos;

        // If hitpoint is behind of a sampled point, zero contribution
        if(light_normal_world.dot(light_to_hitpos) <= 0){
            return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf};
        }

        // If hitpoint and sampled point is not visible to each other, zero contribution
        bool is_visible = scene.is_visible(hitpos_info.p, light_pos);
        if(!is_visible){
            return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf};
        }

        return {m_radiance, light_pos, light_normal_world, pos_pdf};
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
