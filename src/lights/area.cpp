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

#include <logger.h>
#include <ray.h>
#include <common.h>
#include <sampler.h>
#include <scene.h>
#include <shape.h>
#include <rayintersectinfo.h>
#include <polygon_sampling.h>

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
        // Solid angle sampling of convex polygons.
        // Based on: Christoph Peters, 2021
        //   "BRDF Importance Sampling for Polygonal Lights", Section 5 & Supplement C
        //   https://doi.org/10.1145/3450626.3459672
        if (TRY_SOLID_ANGLE_SAMPLING && m_shape->is_solid_angle_sampling_possible()) {
            const auto& verts = m_shape->get_polygon_vertices();
            const auto polygon = prepare_solid_angle_polygon(verts.size(), verts.data(), hitpos_info.p);
            if(polygon.solid_angle <= Float0){
                return {vec3f_zero, vec3f_zero, vec3f_zero, Float0};
            }
            const Float pdf = Float1 / polygon.solid_angle;

            const Vector3f dir = sample_solid_angle_polygon(polygon, sampler.sample_1d(), sampler.sample_1d());

            if(std::isnan(dir[0]) || std::isnan(dir[1]) || std::isnan(dir[2])){
                return {vec3f_zero, vec3f_zero, vec3f_zero, Float0};
            }

            // Use light mesh intersection for exact surface point
            const Ray sample_ray(hitpos_info.p, dir);
            const auto [hit, mesh_info] = m_shape->ray_intersect(sample_ray, INF);

            if(!hit){
                return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
            }

            const Vector3f light_pos = mesh_info.p;
            const Vector3f light_normal_world = mesh_info.sh_coord.m_world_n;

            if(light_normal_world.dot(hitpos_info.p - light_pos) <= Float0){
                return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
            }

            if(!scene.is_visible(hitpos_info.p, light_pos)){
                return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
            }

            return {m_radiance, light_pos, light_normal_world, pdf};
        }
        else /* Traditional uniform area sampling */{
            const auto [light_pos, light_normal_world, pos_pdf] = m_shape->sample_point(sampler);
            const Vector3f light_to_hitpos = hitpos_info.p - light_pos;

            if(light_normal_world.dot(light_to_hitpos) <= 0){
                return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf};
            }

            bool is_visible = scene.is_visible(hitpos_info.p, light_pos);
            if(!is_visible){
                return {vec3f_zero, vec3f_zero, vec3f_zero, pos_pdf};
            }

            return {m_radiance, light_pos, light_normal_world, pos_pdf};
        }
    }

    Float AreaLight::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const{
        // Solid angle sampling of convex polygons.
        // Based on: Christoph Peters, 2021
        //   "BRDF Importance Sampling for Polygonal Lights", Section 5 & Supplement C
        //   https://doi.org/10.1145/3450626.3459672
        if constexpr(TRY_SOLID_ANGLE_SAMPLING) {
            if(m_shape->is_solid_angle_sampling_possible()){
                const auto& verts = m_shape->get_polygon_vertices();
                const auto polygon = prepare_solid_angle_polygon(verts.size(), verts.data(), hitpos_world);
                return (polygon.solid_angle > Float0) ? Float1 / polygon.solid_angle : Float0;
            }
        }

        // Non-solid-angle path: area-based PDF
        return m_shape->pdf_solidangle(hitpos_world, lightpos_world, light_normal_world);
    }

    bool AreaLight::is_delta() const {
        return false;
    }

    bool AreaLight::is_envlight() const {
        return false;
    }

}
