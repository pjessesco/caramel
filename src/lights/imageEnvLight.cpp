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
    ImageEnvLight::ImageEnvLight(const std::string &path, Float scale)
        : m_image(new Image(path)), m_scale(scale), m_imageDistrib(m_image->get_data_for_sampling(true)),
          m_width(m_image->size()[0]), m_height(m_image->size()[1]), m_width_height(m_width * m_height) {

    }

    Vector3f ImageEnvLight::radiance(const Vector3f &, const Vector3f &, const Vector3f &light_normal_world) const{
        const Vector3f dir = -light_normal_world.normalize();
        Vector2f uv = vec_to_normalized_uv(dir);

        const auto size = m_image->size();
        return m_image->get_pixel_value(uv[0] * size[0], uv[1] * size[1]);
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> ImageEnvLight::sample_direct_contribution(const Scene &scene, const RayIntersectInfo &hitpos_info, Sampler &sampler) const{
        const auto sampled_uv = m_imageDistrib.sample(sampler.sample_1d(), sampler.sample_1d());
        const auto pos_to_light_world = normalized_uv_to_vec(Vector2f{static_cast<Float>(sampled_uv[0] + Float0_5) / m_width, static_cast<Float>(sampled_uv[1] + Float0_5) / m_height});

        const Vector3f light_pos = hitpos_info.p + (pos_to_light_world * scene.m_sceneRadius * 2);

        const auto pdf = m_width_height * m_imageDistrib.pdf(sampled_uv[0], sampled_uv[1]) / (2 * PI * PI * std::sin(static_cast<Float>((sampled_uv[1] + Float0_5) / m_height) * PI));

        auto [visible, info] = scene.is_visible(light_pos, hitpos_info.p);
        if (!visible) {
            return {vec3f_zero, vec3f_zero, vec3f_zero, pdf, RayIntersectInfo()};
        }

        return {radiance(hitpos_info.p, light_pos, -pos_to_light_world), light_pos, -pos_to_light_world, pdf, RayIntersectInfo()/*TODO?*/};
    }

    Float ImageEnvLight::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const{
        const auto dir = Vector3f(lightpos_world - hitpos_world).normalize();
        const Vector2f uv = vec_to_normalized_uv(dir);
        if (std::abs(uv[1] - Float1) < 1e-6 || std::abs(uv[1] - Float0) < 1e-6) {
            return 0;
        }

        Vector2i pixel_idx(static_cast<int>(uv[0] * m_width), static_cast<int>(uv[1] * m_height));
        if (pixel_idx[0] >=  m_width) {
            pixel_idx[0] -= m_width;
        }

        return m_width_height * m_imageDistrib.pdf(pixel_idx[0], pixel_idx[1]) / (2 * PI * PI * std::sin(uv[1] * PI)); // ???
    }

    bool ImageEnvLight::is_delta() const {
        return false;
    }

    bool ImageEnvLight::is_envlight() const {
        return true;
    }

}
