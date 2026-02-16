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
    PointLight::PointLight(const Vector3f &pos, const Vector3f &radiant_intensity)
        : m_pos{pos}, m_radiant_intensity{radiant_intensity} {}

    PointLight::~PointLight() = default;

    Vector3f PointLight::radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &) const{
        return vec3f_zero;
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float> PointLight::sample_direct_contribution(const Scene &scene, const RayIntersectInfo &hitpos_info, Sampler &) const{
        // If hitpoint and sampled point is not visible to each other, zero contribution
        bool is_visible = scene.is_visible(m_pos, hitpos_info.p);

        if(!is_visible){
            return {vec3f_zero, vec3f_zero, vec3f_zero, Float1};
        }

        const Vector3f light_to_hitpos = hitpos_info.p - m_pos;

        return {m_radiant_intensity / light_to_hitpos.dot(light_to_hitpos)/* = dist * dist*/,
                m_pos, light_to_hitpos.normalize(), Float1};
    }

    Float PointLight::pdf_solidangle(const Vector3f &, const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    bool PointLight::is_delta() const {
        return true;
    }

    bool PointLight::is_envlight() const {
        return false;
    }

}
