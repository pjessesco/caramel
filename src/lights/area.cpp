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

    void AreaLight::init_is_triangle_mesh() {
        m_is_triangle_mesh = (dynamic_cast<const TriangleMesh*>(m_shape) != nullptr);
    }

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
        if constexpr(SOLID_ANGLE_SAMPLING) {
            const auto type = m_shape->get_solid_angle_sampling_type();
            if(type == SolidAngleSamplingType::SinglePolygon || type == SolidAngleSamplingType::PerTriangle){
                SolidAnglePolygon polygon;
                Float pdf;
                Vector3f plane_n;
                Vector3f plane_point;

                if(type == SolidAngleSamplingType::SinglePolygon){
                    // Full polygon sampling (Triangle or coplanar mesh with ≤ 8 boundary vertices)
                    const auto& verts = m_shape->get_polygon_vertices();
                    polygon = prepare_solid_angle_polygon(verts.size(), verts.data(), hitpos_info.p);
                    if(polygon.solid_angle <= Float0){
                        return {vec3f_zero, vec3f_zero, vec3f_zero, Float0};
                    }
                    pdf = Float1 / polygon.solid_angle;
                    // See also Triangle::ray_intersect() for geometry normal calc
                    plane_n = Vector3f::cross(verts[1] - verts[0], verts[2] - verts[0]);
                    plane_point = verts[0];
                }
                else{
                    // Per-triangle solid angle sampling for non-coplanar or large meshes
                    const auto *mesh = static_cast<const TriangleMesh*>(m_shape);
                    const Index tri_idx = mesh->sample_triangle_index(sampler.sample_1d());
                    const Float tri_pdf = mesh->triangle_select_pdf(tri_idx);
                    const auto [p0, p1, p2] = mesh->get_triangle_vertices(tri_idx);
                    const Vector3f verts[3] = {p0, p1, p2};
                    polygon = prepare_solid_angle_polygon(3, verts, hitpos_info.p);
                    if(polygon.solid_angle <= Float0){
                        return {vec3f_zero, vec3f_zero, vec3f_zero, Float0};
                    }
                    pdf = tri_pdf / polygon.solid_angle;
                    // See also Triangle::ray_intersect() for geometry normal calc
                    plane_n = Vector3f::cross(p1 - p0, p2 - p0);
                    plane_point = p0;
                }

                // sample_solid_angle_polygon returns a *direction*, not a surface point.
                // We need the actual 3D position on the light for radiance/visibility evaluation,
                // so we intersect the ray (hitpos, dir) with the polygon's plane.
                // sample_solid_angle_polygon returns a *direction*, not a surface point.
                // We need the actual 3D position on the light for radiance/visibility evaluation,
                // so we intersect the ray (hitpos, dir) with the polygon's plane.
                const Vector3f dir = sample_solid_angle_polygon(polygon, sampler.sample_1d(), sampler.sample_1d());

                if(std::isnan(dir[0]) || std::isnan(dir[1]) || std::isnan(dir[2])){
                    return {vec3f_zero, vec3f_zero, vec3f_zero, Float0};
                }

                // Ray-plane intersection to recover the light surface point:
                //   plane equation: plane_n · (x - plane_point) = 0
                //   ray: x = hitpos + t * dir
                //   => t = plane_n · (plane_point - hitpos) / (plane_n · dir)
                const Float denom = plane_n.dot(dir);

                if(denom == Float0){
                    return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
                }

                const Float t = plane_n.dot(plane_point - hitpos_info.p) / denom;

                // Intersection behind the shading point
                if(t <= Float0){
                    return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
                }

                const Vector3f light_pos = hitpos_info.p + dir * t;
                const Vector3f light_normal_world = plane_n.normalize();

                // If hitpoint is behind of a sampled point, zero contribution
                if(light_normal_world.dot(hitpos_info.p - light_pos) <= Float0){
                    return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
                }

                // If hitpoint and sampled point is not visible to each other, zero contribution
                if(!scene.is_visible(hitpos_info.p, light_pos)){
                    return {vec3f_zero, vec3f_zero, vec3f_zero, pdf};
                }

                return {m_radiance, light_pos, light_normal_world, pdf};
            }
            else{
                CRM_ERROR("Unexpected SolidAngleSamplingType");
            }
        }
        else /* traditional uniform sampling */{
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

    }

    Float AreaLight::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const{
        // Solid angle sampling of convex polygons.
        // Based on: Christoph Peters, 2021
        //   "BRDF Importance Sampling for Polygonal Lights", Section 5 & Supplement C
        //   https://doi.org/10.1145/3450626.3459672
        if constexpr(SOLID_ANGLE_SAMPLING) {
            const auto type = m_shape->get_solid_angle_sampling_type();
            if(type == SolidAngleSamplingType::SinglePolygon){
                // Uniform over entire polygon solid angle: pdf = 1 / Ω
                const auto& verts = m_shape->get_polygon_vertices();
                const auto polygon = prepare_solid_angle_polygon(verts.size(), verts.data(), hitpos_world);
                return (polygon.solid_angle > Float0) ? Float1 / polygon.solid_angle : Float0;
            }
            else if(type == SolidAngleSamplingType::PerTriangle){
                // Per-triangle: pdf = P(select triangle) / Ω_triangle
                // Re-shoot ray to identify which triangle was hit
                const auto *mesh = static_cast<const TriangleMesh*>(m_shape);

                const Vector3f diff = lightpos_world - hitpos_world;
                const Float dist = diff.length();

                if(dist <= Float0){
                    return m_shape->pdf_solidangle(hitpos_world, lightpos_world, light_normal_world);
                }

                const Ray ray(hitpos_world, diff / dist);
                const auto [hit, info] = mesh->ray_intersect(ray, dist * Float(1.01));

                if(!hit){
                    return m_shape->pdf_solidangle(hitpos_world, lightpos_world, light_normal_world);
                }

                const auto [p0, p1, p2] = mesh->get_triangle_vertices(info.tri_index);
                const Vector3f verts[3] = {p0, p1, p2};
                const auto polygon = prepare_solid_angle_polygon(3, verts, hitpos_world);

                if(polygon.solid_angle <= Float0){
                    return m_shape->pdf_solidangle(hitpos_world, lightpos_world, light_normal_world);
                }

                return mesh->triangle_select_pdf(info.tri_index) / polygon.solid_angle;
            }
            else{
                CRM_ERROR("Unexpected SolidAngleSamplingType");
            }
        }
        else/* traditional uniform sampling */ {
            return m_shape->pdf_solidangle(hitpos_world, lightpos_world, light_normal_world);
        }
    }

    bool AreaLight::is_delta() const {
        return false;
    }

    bool AreaLight::is_envlight() const {
        return false;
    }

}
