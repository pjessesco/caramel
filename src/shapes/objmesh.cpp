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

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace Caramel {
    OBJMesh::OBJMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight, const Matrix44f &transform)
    : TriangleMesh(bsdf, arealight){
        if (!m_vertices.empty()) {
            CRM_ERROR("This mesh already loaded obj file");
        }
        if (!std::filesystem::exists(path)) {
            CRM_ERROR(path.string() + " is not exists");
        }
        std::string err;

        tinyobj::ObjReader reader;
        // `triangulate` option is true by default
        if (!reader.ParseFromFile(path.string(), tinyobj::ObjReaderConfig())) {
            if (!reader.Error().empty()) {
                CRM_ERROR("TinyObjReader error : " + reader.Error());
            }
            CRM_ERROR("Cannot read obj file");
        }

        const auto &attrib = reader.GetAttrib();
        const auto &shapes = reader.GetShapes();
        const auto &mats = reader.GetMaterials();

        CRM_LOG("Loading obj : " + path.string());
        if (shapes.size() != 1) {
            CRM_ERROR("We do not support obj file with several shapes");
        }

        is_vn_exists = !attrib.normals.empty();
        is_tx_exists = !attrib.texcoords.empty();

        // Used for aabb
        Float min_x = INF,  min_y = INF,  min_z = INF,
              max_x = -INF, max_y = -INF, max_z = -INF;

        for (int i = 0; i < attrib.vertices.size(); i += 3) {
            const Vector3f transformed_point = transform_point({attrib.vertices[i], attrib.vertices[i+1], attrib.vertices[i+2]}, transform);

            min_x = min_x > transformed_point[0] ? transformed_point[0] : min_x;
            min_y = min_y > transformed_point[1] ? transformed_point[1] : min_y;
            min_z = min_z > transformed_point[2] ? transformed_point[2] : min_z;

            max_x = max_x < transformed_point[0] ? transformed_point[0] : max_x;
            max_y = max_y < transformed_point[1] ? transformed_point[1] : max_y;
            max_z = max_z < transformed_point[2] ? transformed_point[2] : max_z;

            m_vertices.emplace_back(transformed_point[0],
                                    transformed_point[1],
                                    transformed_point[2]);
        }

        for (int i = 0; i < attrib.normals.size(); i += 3) {
            const Vector3f transformed_normal = transform_normal({attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]}, transform);
            m_normals.emplace_back(transformed_normal[0], transformed_normal[1], transformed_normal[2]);
        }

        for (int i = 0; i < attrib.texcoords.size(); i += 2) {
            m_tex_coords.emplace_back(attrib.texcoords[i],
                                      attrib.texcoords[i + 1]);
        }

        const auto &indices = shapes[0].mesh.indices;

        for (int i = 0; i < indices.size(); i += 3) {
            m_vertex_indices.emplace_back(indices[i].vertex_index,
                                          indices[i + 1].vertex_index,
                                          indices[i + 2].vertex_index);

            m_normal_indices.emplace_back(indices[i].normal_index,
                                          indices[i + 1].normal_index,
                                          indices[i + 2].normal_index);

            m_tex_coord_indices.emplace_back(indices[i].texcoord_index,
                                             indices[i + 1].texcoord_index,
                                             indices[i + 2].texcoord_index);
        }

        std::vector<Float> triangle_area_vec;
        triangle_area_vec.resize(m_vertex_indices.size());

        m_area = Float0;
        // Initialize m_triangle_pdf used in sampling triangle.
        for(int i=0;i<m_vertex_indices.size();i++){
            const Float ith_tri_area = get_triangle_area(i);
            triangle_area_vec[i] = ith_tri_area;
            m_area += ith_tri_area;
        }
        m_triangle_pdf = Distrib1D(triangle_area_vec);

        m_aabb = AABB({min_x, min_y, min_z}, {max_x, max_y, max_z});

        m_accel = std::make_unique<Octree>(*this);
        m_accel->build();
    }

    std::pair<bool, RayIntersectInfo> OBJMesh::ray_intersect(const Ray &ray, Float maxt) const {
        return m_accel->ray_intersect(ray, maxt);
    }

    AABB OBJMesh::get_aabb() const {
        return m_aabb;
    }

    Float OBJMesh::get_area() const{
        return m_area;
    }

    std::tuple<Vector3f, Vector3f, Float> OBJMesh::sample_point(Sampler &sampler) const{
        // Sample triangle considering area
        const Index i = m_triangle_pdf.sample(sampler.sample_1d());

        // Sample point in chosen triangle
        const auto [pos, normal, _] = get_triangle_sample_point(i, sampler);

        return {pos, normal, Float1 / m_area};
    }

    // Similar with `Triangle::pdf_solidangle()`
    // This implementation assumes that two given points are visible to each other
    Float OBJMesh::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const{
        const Vector3f shape_to_hitpos_world = hitpos_world - shapepos_world;
        const Float cos = std::abs(shape_normal_world.dot(shape_to_hitpos_world.normalize()));
        const Float dist_squared = shape_to_hitpos_world.dot(shape_to_hitpos_world);
        return dist_squared / (cos * m_area);
    }

    Float OBJMesh::get_triangle_area(Index i) const {
        const auto &v_idx = m_vertex_indices[i];
        const Vector3f &p0 = m_vertices[v_idx[0]];
        const Vector3f &p1 = m_vertices[v_idx[1]];
        const Vector3f &p2 = m_vertices[v_idx[2]];

        return Vector3f::cross(p1 - p0, p2 - p0).length() * Float0_5;
    }

    std::tuple<Vector3f, Vector3f, Float> OBJMesh::get_triangle_sample_point(Index i, Sampler &sampler) const {
        const auto &v_idx = m_vertex_indices[i];
        const Vector3f &p0 = m_vertices[v_idx[0]];
        const Vector3f &p1 = m_vertices[v_idx[1]];
        const Vector3f &p2 = m_vertices[v_idx[2]];

        const Float u = sampler.sample_1d();
        const Float v = sampler.sample_1d();
        const Float x = Float1 - std::sqrt(Float1 - u);
        const Float y = v * std::sqrt(Float1 - u);

        Vector3f normal_vec;
        if (is_vn_exists) {
            const auto &n_idx = m_normal_indices[i];
            const Vector3f &n0 = m_normals[n_idx[0]];
            const Vector3f &n1 = m_normals[n_idx[1]];
            const Vector3f &n2 = m_normals[n_idx[2]];
            normal_vec = interpolate(n0, n1, n2, x, y).normalize();
        } else {
            normal_vec = Vector3f::cross(p1 - p0, p2 - p0).normalize();
        }

        return {interpolate(p0, p1, p2, x, y),
                normal_vec,
                Float1 / get_triangle_area(i)};
    }

    std::pair<bool, RayIntersectInfo> OBJMesh::get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const {
        const auto &v_idx = m_vertex_indices[i];
        const Vector3f &p0 = m_vertices[v_idx[0]];
        const Vector3f &p1 = m_vertices[v_idx[1]];
        const Vector3f &p2 = m_vertices[v_idx[2]];

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
        
        if (is_tx_exists) {
            const auto &uv_idx = m_tex_coord_indices[i];
            const Vector2f &uv0 = m_tex_coords[uv_idx[0]];
            const Vector2f &uv1 = m_tex_coords[uv_idx[1]];
            const Vector2f &uv2 = m_tex_coords[uv_idx[2]];
            ret.tex_uv = interpolate(uv0, uv1, uv2, u, v);
        } else {
            ret.tex_uv = Vector2f{u, v};
        }

        ret.tex_uv[0] -= floor(ret.tex_uv[0]);
        ret.tex_uv[1] -= floor(ret.tex_uv[1]);

        ret.p = interpolate(p0, p1, p2, u, v);
        
        Vector3f n;
        if (is_vn_exists) {
            const auto &n_idx = m_normal_indices[i];
            const Vector3f &n0 = m_normals[n_idx[0]];
            const Vector3f &n1 = m_normals[n_idx[1]];
            const Vector3f &n2 = m_normals[n_idx[2]];
            n = interpolate(n0, n1, n2, u, v).normalize();
        } else {
            n = Vector3f::cross(p1 - p0, p2 - p0).normalize();
        }
        ret.sh_coord = Coordinate(n);

        return {true, ret};
    }

    AABB OBJMesh::get_triangle_aabb(Index i) const {
        const auto &v_idx = m_vertex_indices[i];
        const Vector3f &p0 = m_vertices[v_idx[0]];
        const Vector3f &p1 = m_vertices[v_idx[1]];
        const Vector3f &p2 = m_vertices[v_idx[2]];
        
        return AABB(Vector3f{std::min({p0[0], p1[0], p2[0]}),
                             std::min({p0[1], p1[1], p2[1]}),
                             std::min({p0[2], p1[2], p2[2]})},
                    Vector3f{std::max({p0[0], p1[0], p2[0]}),
                             std::max({p0[1], p1[1], p2[1]}),
                             std::max({p0[2], p1[2], p2[2]})});
    }
}
