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
#include <transform.h>
#include <image.h>
#include <rayintersectinfo.h>
#include <warp_sample.h>


namespace Caramel{
    ImageEnvLight::ImageEnvLight(const std::string &path, Float scale, const Matrix44f &transform)
        : m_image(new Image(path)), m_scale(scale), m_transform(transform) {}

    Vector3f ImageEnvLight::radiance(const Vector3f &, const Vector3f &, const Vector3f &light_normal_world) const{
        const Vector3f dir = transform_vector(-light_normal_world, m_transform).normalize();
        Vector2f uv = vec_to_uv(dir);

        const auto size = m_image->size();
        return m_image->get_pixel_value(uv[0] * size[0], uv[1] * size[1]);
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> ImageEnvLight::sample_direct_contribution(const Scene &scene, const RayIntersectInfo &hitpos_info, Sampler &sampler) const{
        auto [pos_to_light_dir_local, pos_pdf] = sample_unit_sphere_uniformly(sampler);
        auto pos_to_light_dir_world = hitpos_info.sh_coord.to_world(pos_to_light_dir_local);

        const Vector3f light_pos = hitpos_info.p + (pos_to_light_dir_world * scene.m_sceneRadius * 2);
        auto [visible, info] = scene.is_visible(light_pos, hitpos_info.p);
        if (!visible) {
            return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf, RayIntersectInfo()};
        }

        return {radiance(hitpos_info.p, light_pos, -pos_to_light_dir_world), light_pos, -pos_to_light_dir_world, PI_4_INV, RayIntersectInfo()/*TODO?*/};
    }

    Float ImageEnvLight::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const{
        return PI_4_INV;
    }

    bool ImageEnvLight::is_delta() const {
        return false;
    }

    bool ImageEnvLight::is_envlight() const {
        return true;
    }

}
