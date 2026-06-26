//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2026 Jino Park
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

#include <algorithm>

#include <shape.h>

#include <ray.h>
#include <rayintersectinfo.h>
#include <transform.h>
#include <aabb.h>
#include <sampler.h>

namespace Caramel{
    namespace {
        // World AABB = tight box around the 8 transformed corners of the local AABB.
        AABB transform_aabb(const AABB &local, const Matrix44f &to_world){
            Vector3f mn{INF, INF, INF};
            Vector3f mx{-INF, -INF, -INF};
            for(Index i = 0; i < 8; ++i){
                const Vector3f c = transform_point(local.corner(i), to_world);
                for(int a = 0; a < 3; ++a){
                    mn[a] = std::min(mn[a], c[a]);
                    mx[a] = std::max(mx[a], c[a]);
                }
            }
            return {mn, mx};
        }

        Float transformed_triangle_area(const Vector3f &a, const Vector3f &b, const Vector3f &c, const Matrix44f &m){
            const Vector3f w0 = transform_point(a, m);
            const Vector3f w1 = transform_point(b, m);
            const Vector3f w2 = transform_point(c, m);
            return Vector3f::cross(w1 - w0, w2 - w0).length() * Float0_5;
        }

        Float polygon_world_area(const std::vector<Vector3f> &poly, const Matrix44f &to_world){
            Float area = Float0;
            for(std::size_t i = 1; i + 1 < poly.size(); ++i){
                area += transformed_triangle_area(poly[0], poly[i], poly[i + 1], to_world);
            }
            return area;
        }
    }

    Instance::Instance(const Shape *geometry, const Matrix44f &to_world, BSDF *bsdf, AreaLight *arealight)
        : Shape{bsdf, arealight}, m_geometry{geometry}, m_to_world{to_world},
          m_to_local{Inverse(to_world)},
          m_world_aabb{transform_aabb(geometry->get_aabb(), to_world)} {
        if(arealight != nullptr){
            // World area must be exact under an arbitrary (non-uniform / sheared) to_world,
            // which no single scalar captures, so a mesh emitter is measured per-triangle in
            // world space. Templates carry no arealight, so finalize() built no boundary
            // polygon to reuse -- hence the type check. m_world_triangle_pdf weights triangles
            // by world area so sample_point()'s 1/m_world_area pdf is exact.
            if(const auto *mesh = dynamic_cast<const TriangleMesh*>(geometry)){
                std::vector<Float> world_tri_areas(mesh->get_triangle_num());
                m_world_area = Float0;
                for(Index i = 0; i < mesh->get_triangle_num(); ++i){
                    const auto [a, b, c] = mesh->get_triangle_vertices(i);
                    world_tri_areas[i] = transformed_triangle_area(a, b, c, to_world);
                    m_world_area += world_tri_areas[i];
                }
                m_world_triangle_pdf = Distrib1D(world_tri_areas);
            }
            else{
                m_world_area = polygon_world_area(m_geometry->get_polygon_vertices(), to_world);
            }
            if(m_geometry->is_solid_angle_sampling_possible()){
                for(const auto &v : m_geometry->get_polygon_vertices()){
                    m_world_polygon_vertices.push_back(transform_point(v, to_world));
                }
            }
        }
    }

    std::pair<bool, RayIntersectInfo> Instance::ray_intersect(const Ray &ray, Float maxt) const{
        // World ray -> local space. Ray's ctor renormalizes the direction, so the
        // local t is measured along a unit local dir; k = |M_inv * d| converts
        // between world and local distance (handles non-uniform scale + rotation).
        const Vector3f d_local = transform_vector(ray.m_d, m_to_local);
        const Float k = d_local.length();
        const Ray local{transform_point(ray.m_o, m_to_local), d_local};

        auto [hit, info] = m_geometry->ray_intersect(local, maxt * k);
        if(!hit){
            return {false, info};
        }

        info.p = transform_point(info.p, m_to_world);
        // Normal transform = inverse-transpose of to_world = transpose(m_to_local).
        // Using the transpose avoids a per-ray matrix inverse.
        const Matrix44f normal_mat{T(m_to_local)};
        info.sh_coord = Coordinate{transform_vector(info.sh_coord.m_world_n, normal_mat).normalize()};
        info.t /= k;
        return {true, info};
        // info.shape is overwritten by the scene BVH with this Instance*, so the
        // integrator reads the BSDF carried by this Instance (gotcha 3 in the design doc).
    }

    AABB Instance::get_aabb() const{
        return m_world_aabb;
    }

    Float Instance::get_area() const{
        return m_world_area;
    }

    std::tuple<Vector3f, Vector3f, Float> Instance::sample_point(Sampler &sampler) const{
        // Pick a triangle by its WORLD area, then sample uniformly within it, so the
        // world-space density is exactly 1/m_world_area even under non-uniform scale.
        const TriangleMesh *mesh = dynamic_cast<const TriangleMesh*>(m_geometry);
        const auto [local_p, local_n, local_pdf] = mesh
            ? mesh->get_triangle_sample_point(m_world_triangle_pdf.sample(sampler.sample_1d()), sampler)
            : m_geometry->sample_point(sampler);
        const Vector3f world_p = transform_point(local_p, m_to_world);
        const Vector3f world_n = transform_vector(local_n, T(m_to_local)).normalize();
        return {world_p, world_n, Float1 / m_world_area};
    }

    Float Instance::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const{
        const Vector3f shape_to_hitpos_world = hitpos_world - shapepos_world;
        using std::abs;
        const Float cos = abs(shape_normal_world.dot(shape_to_hitpos_world.normalize()));
        const Float dist_squared = shape_to_hitpos_world.dot(shape_to_hitpos_world);
        return dist_squared / (cos * get_area());
    }

    bool Instance::is_solid_angle_sampling_possible() const{
        return m_geometry->is_solid_angle_sampling_possible();
    }

    const std::vector<Vector3f>& Instance::get_polygon_vertices() const{
        return m_world_polygon_vertices;
    }
}
