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

#include <filesystem>
#include <tuple>

#include <shape.h>

#include <logger.h>
#include <mesh_accel.h>
#include <rayintersectinfo.h>
#include <sampler.h>
#include <transform.h>

#include "happly.h"

namespace Caramel {
    PLYMesh::PLYMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight, const Matrix44f &transform)
    : TriangleMesh(bsdf, arealight) {
        if (!std::filesystem::exists(path)) {
            CRM_ERROR(path.string() + " does not exist");
        }

        CRM_LOG("Loading ply : " + path.string());

        happly::PLYData plyData(path.string());

        // Get vertex positions
        std::vector<std::array<double, 3>> vertices = plyData.getVertexPositions();

        // Check if normals exist
        is_vn_exists = plyData.getElement("vertex").hasProperty("nx") &&
                       plyData.getElement("vertex").hasProperty("ny") &&
                       plyData.getElement("vertex").hasProperty("nz");

        std::vector<float> nx, ny, nz;
        if (is_vn_exists) {
            nx = plyData.getElement("vertex").getProperty<float>("nx");
            ny = plyData.getElement("vertex").getProperty<float>("ny");
            nz = plyData.getElement("vertex").getProperty<float>("nz");
        }

        // Get face indices
        std::vector<std::vector<size_t>> faces = plyData.getFaceIndices<size_t>();

        // Used for aabb
        Float min_x = INF,  min_y = INF,  min_z = INF,
              max_x = -INF, max_y = -INF, max_z = -INF;

        // Process vertices
        m_vertices.reserve(vertices.size());
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Vector3f transformed_point = transform_point(
                {static_cast<Float>(vertices[i][0]),
                 static_cast<Float>(vertices[i][1]),
                 static_cast<Float>(vertices[i][2])},
                transform);

            min_x = std::min(min_x, transformed_point[0]);
            min_y = std::min(min_y, transformed_point[1]);
            min_z = std::min(min_z, transformed_point[2]);

            max_x = std::max(max_x, transformed_point[0]);
            max_y = std::max(max_y, transformed_point[1]);
            max_z = std::max(max_z, transformed_point[2]);

            m_vertices.emplace_back(transformed_point[0],
                                    transformed_point[1],
                                    transformed_point[2]);
        }

        // Process normals
        if (is_vn_exists) {
            m_normals.reserve(vertices.size());
            for (size_t i = 0; i < vertices.size(); ++i) {
                const Vector3f transformed_normal = transform_normal(
                    {static_cast<Float>(nx[i]),
                     static_cast<Float>(ny[i]),
                     static_cast<Float>(nz[i])},
                    transform);
                m_normals.emplace_back(transformed_normal[0],
                                       transformed_normal[1],
                                       transformed_normal[2]);
            }
        }

        // Process faces - triangulate if necessary
        for (const auto& face : faces) {
            if (face.size() < 3) {
                CRM_WARNING("Skipping degenerate face with less than 3 vertices");
                continue;
            }

            // Triangulate polygon using fan triangulation
            for (size_t i = 1; i + 1 < face.size(); ++i) {
                m_face_indices.emplace_back(
                    static_cast<Int>(face[0]),
                    static_cast<Int>(face[i]),
                    static_cast<Int>(face[i + 1]));
            }
        }

        // Calculate triangle areas for sampling
        std::vector<Float> triangle_area_vec;
        triangle_area_vec.resize(m_face_indices.size());

        m_area = Float0;
        for (size_t i = 0; i < m_face_indices.size(); ++i) {
            const Float ith_tri_area = get_triangle_area(i);
            triangle_area_vec[i] = ith_tri_area;
            m_area += ith_tri_area;
        }
        m_triangle_pdf = Distrib1D(triangle_area_vec);

        m_aabb = AABB({min_x, min_y, min_z}, {max_x, max_y, max_z});
        m_accel = std::make_unique<Octree>(*this);
        m_accel->build();
    }

    std::pair<bool, RayIntersectInfo> PLYMesh::ray_intersect(const Ray &ray, Float maxt) const {
        return m_accel->ray_intersect(ray, maxt);
    }

    AABB PLYMesh::get_aabb() const {
        return m_aabb;
    }

    Float PLYMesh::get_area() const {
        return m_area;
    }

    std::tuple<Vector3f, Vector3f, Float> PLYMesh::sample_point(Sampler &sampler) const {
        // Sample triangle considering area
        const Index i = m_triangle_pdf.sample(sampler.sample_1d());

        // Sample point in chosen triangle
        const auto [pos, normal, _] = get_triangle_sample_point(i, sampler);

        return {pos, normal, Float1 / m_area};
    }

    Float PLYMesh::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const {
        const Vector3f shape_to_hitpos_world = hitpos_world - shapepos_world;
        using std::abs;
        const Float cos = abs(shape_normal_world.dot(shape_to_hitpos_world.normalize()));
        const Float dist_squared = shape_to_hitpos_world.dot(shape_to_hitpos_world);
        return dist_squared / (cos * m_area);
    }

    Float PLYMesh::get_triangle_area(Index i) const {
        const Vector3i& idx = m_face_indices[i];
        const Vector3f &p0 = m_vertices[idx[0]];
        const Vector3f &p1 = m_vertices[idx[1]];
        const Vector3f &p2 = m_vertices[idx[2]];

        return Vector3f::cross(p1 - p0, p2 - p0).length() * Float0_5;
    }

    std::tuple<Vector3f, Vector3f, Float> PLYMesh::get_triangle_sample_point(Index i, Sampler &sampler) const {
        const Vector3i& idx = m_face_indices[i];
        const Vector3f &p0 = m_vertices[idx[0]];
        const Vector3f &p1 = m_vertices[idx[1]];
        const Vector3f &p2 = m_vertices[idx[2]];

        const Float u = sampler.sample_1d();
        const Float v = sampler.sample_1d();
        using std::sqrt;
        const Float x = Float1 - sqrt(Float1 - u);
        const Float y = v * sqrt(Float1 - u);

        Vector3f n;
        if (is_vn_exists) {
            const Vector3f &n0 = m_normals[idx[0]];
            const Vector3f &n1 = m_normals[idx[1]];
            const Vector3f &n2 = m_normals[idx[2]];
            n = interpolate(n0, n1, n2, x, y).normalize();
        } else {
            n = Vector3f::cross(p1 - p0, p2 - p0).normalize();
        }

        return {interpolate(p0, p1, p2, x, y),
                n,
                Float1 / get_triangle_area(i)};
    }

    std::pair<bool, RayIntersectInfo> PLYMesh::get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const {
        const Vector3i& idx = m_face_indices[i];
        const Vector3f &p0 = m_vertices[idx[0]];
        const Vector3f &p1 = m_vertices[idx[1]];
        const Vector3f &p2 = m_vertices[idx[2]];

#ifdef USE_MOLLER_TRUMBORE
        auto [u, v, t] = moller_trumbore(ray, p0, p1, p2, maxt);
#else
        // Default: watertight intersection (more robust at triangle edges)
        // Reference: https://jcgt.org/published/0002/01/05/paper.pdf
        auto [u, v, t] = watertight_intersection(ray, p0, p1, p2, maxt);
#endif

        if(u==-Float1 && v==-Float1 && t==-Float1){
            return {false, RayIntersectInfo()};
        }

        RayIntersectInfo ret;
        ret.t = t;
        
        // PLYMesh doesn't have UVs in this implementation
        ret.tex_uv = Vector2f{u, v};

        ret.tex_uv[0] -= floor(ret.tex_uv[0]);
        ret.tex_uv[1] -= floor(ret.tex_uv[1]);

        ret.p = interpolate(p0, p1, p2, u, v);
        
        Vector3f n;
        if (is_vn_exists) {
            const Vector3f &n0 = m_normals[idx[0]];
            const Vector3f &n1 = m_normals[idx[1]];
            const Vector3f &n2 = m_normals[idx[2]];
            n = interpolate(n0, n1, n2, u, v).normalize();
        } else {
            n = Vector3f::cross(p1 - p0, p2 - p0).normalize();
        }
        ret.sh_coord = Coordinate(n);

        return {true, ret};
    }

    AABB PLYMesh::get_triangle_aabb(Index i) const {
        const Vector3i& idx = m_face_indices[i];
        const Vector3f &p0 = m_vertices[idx[0]];
        const Vector3f &p1 = m_vertices[idx[1]];
        const Vector3f &p2 = m_vertices[idx[2]];
        
        return AABB(Vector3f{std::min({p0[0], p1[0], p2[0]}),
                             std::min({p0[1], p1[1], p2[1]}),
                             std::min({p0[2], p1[2], p2[2]})},
                    Vector3f{std::max({p0[0], p1[0], p2[0]}),
                             std::max({p0[1], p1[1], p2[1]}),
                             std::max({p0[2], p1[2], p2[2]})});
    }
}
