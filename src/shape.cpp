//
// This software is released under the MIT license.
//
// Copyright (c) 2022 Jino Park
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
#include <transform.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

namespace Caramel {
    Shape::Shape(std::unique_ptr<BSDF> bsdf) : m_is_light(false), m_bsdf{std::move(bsdf)} {}

    Triangle::Triangle(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3)
        : m_p(Matrix33f::from_cols(p1, p2, p3)), is_vn_exists{false} {}

    Triangle::Triangle(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3,
                       const Vector3f &n1, const Vector3f &n2, const Vector3f &n3)
        : m_p(Matrix33f::from_cols(p1, p2, p3)), m_n(Matrix33f::from_cols(n1, n2, n3)), is_vn_exists{true} {}

    void Triangle::transform(const Matrix44f &transform) {
        for(Index i=0;i<3;i++){
            m_p.set_col(i, transform_point(m_p.get_col(i), transform));
            m_n.set_col(i, transform_normal(m_n.get_col(i), transform));
        }
    }

    AABB Triangle::get_aabb() const{
        const Vector3f p0 = point(0);
        const Vector3f p1 = point(1);
        const Vector3f p2 = point(2);
        return AABB(Vector3f{std::min({p0[0], p1[0], p2[0]}),
                             std::min({p0[1], p1[1], p2[1]}),
                             std::min({p0[2], p1[2], p2[2]})},
                    Vector3f{std::max({p0[0], p1[0], p2[0]}),
                             std::max({p0[1], p1[1], p2[1]}),
                             std::max({p0[2], p1[2], p2[2]})});
    }

    Float Triangle::get_area() const{
        return cross(point(1) - point(0), point(2) - point(0)).length() * Float0_5;
    }

    std::tuple<Vector3f, Vector3f, Float> Triangle::sample_point(Sampler &sampler) const{
        const Float u = sampler.sample_1d();
        const Float v = sampler.sample_1d();
        const Float x = Float1 - std::sqrt(Float1 - u);
        const Float y = v * std::sqrt(Float1 - u);
        // z = 1 - x - y
        const Vector3f p0 = point(0);
        const Vector3f p1 = point(1);
        const Vector3f p2 = point(2);

        return {interpolate(p0, p1, p2, x, y),
                is_vn_exists ?
                    interpolate(normal(0), normal(1), normal(2), x, y) :
                    cross(p1 - p0, p2 - p0).normalize(),
                Float1 / get_area()};
    }

    inline Vector3f Triangle::point(Index i) const {
        return m_p.get_col(i);
    }

    inline Vector3f Triangle::normal(Index i) const {
        return m_n.get_col(i);
    }

    // u, v, t
    std::tuple<bool, RayIntersectInfo> Triangle::ray_intersect(const Ray &ray) const {
        const Vector3f A = m_p.get_col(0);
        const Vector3f B = m_p.get_col(1);
        const Vector3f C = m_p.get_col(2);
        const Vector3f D = ray.m_d;

        const Vector3f T = ray.m_o - A;
        const Vector3f E1 = B - A;
        const Vector3f E2 = C - A;

        const Vector3f DE2 = cross(D, E2);
        const Vector3f TE1 = cross(T, E1);

        const Float denom_inv = static_cast<Float>(1) / DE2.dot(E1);

        const Float u = DE2.dot(T) * denom_inv;

        if (u < 0 || 1 < u) {
            return {false, RayIntersectInfo()};
        }

        const Float v = TE1.dot(D) * denom_inv;
        if (v < 0 || 1 < v) {
            return {false, RayIntersectInfo()};
        }

        if (u + v > 1) {
            return {false, RayIntersectInfo()};
        }

        const Float t = TE1.dot(E2) * denom_inv;
        if (t < 0) {
            return {false, RayIntersectInfo()};
        }

        // Intersect
        Vector3f hitpos = interpolate(A, B, C, u, v);

        RayIntersectInfo ret;
        ret.p = hitpos;
        ret.t = t;
        ret.u = u;
        ret.v = v;

        if(is_vn_exists){
            Vector3f shn = interpolate(m_n.get_col(0), m_n.get_col(1), m_n.get_col(2), u, v);
            ret.sh_n = shn.normalize();
        }
        else{
            ret.sh_n = cross(E1, E2).normalize();
        }

        return {true, ret};
    }


    OBJMesh::OBJMesh(const std::filesystem::path &path) {
        if (!m_vertices.empty()) {
            ERROR("This mesh already loaded obj file");
        }
        if (!std::filesystem::exists(path)) {
            ERROR(path.string() + " is not exists");
        }
        std::string err;

        tinyobj::ObjReader reader;
        // `triangulate` option is true by default
        if (!reader.ParseFromFile(path, tinyobj::ObjReaderConfig())) {
            if (!reader.Error().empty()) {
                ERROR("TinyObjReader error : " + reader.Error());
            }
            ERROR("Cannot read obj file");
        }

        const auto &attrib = reader.GetAttrib();
        const auto &shapes = reader.GetShapes();
        const auto &mats = reader.GetMaterials();

        LOG("Loading obj in " + path.string());
        if (shapes.size() != 1) {
            ERROR("We do not support obj file with several shapes");
        }
        LOG(" - # of vertices : " + std::to_string(attrib.vertices.size() / 3));
        LOG(" - # of normals : " + std::to_string(attrib.normals.size() / 3));
        LOG(" - # of faces : " + std::to_string(shapes[0].mesh.indices.size() / 3));
        LOG(" - # of texture coordinates : " + std::to_string(attrib.texcoords.size()));

        is_vn_exists = !attrib.normals.empty();

        // Used for aabb
        Float min_x = INF,  min_y = INF,  min_z = INF,
              max_x = -INF, max_y = -INF, max_z = -INF;

        for (int i = 0; i < attrib.vertices.size(); i += 3) {
            min_x = min_x > attrib.vertices[i]     ? attrib.vertices[i]     : min_x;
            min_y = min_y > attrib.vertices[i + 1] ? attrib.vertices[i + 1] : min_y;
            min_z = min_z > attrib.vertices[i + 2] ? attrib.vertices[i + 2] : min_z;

            max_x = max_x < attrib.vertices[i]     ? attrib.vertices[i]     : max_x;
            max_y = max_y < attrib.vertices[i + 1] ? attrib.vertices[i + 1] : max_y;
            max_z = max_z < attrib.vertices[i + 2] ? attrib.vertices[i + 2] : max_z;

            m_vertices.emplace_back(Vector3f(attrib.vertices[i],
                                             attrib.vertices[i + 1],
                                             attrib.vertices[i + 2]));
        }

        for (int i = 0; i < attrib.normals.size(); i += 3) {
            m_normals.emplace_back(Vector3f(attrib.normals[i],
                                            attrib.normals[i + 1],
                                            attrib.normals[i + 2]));
        }

        for (int i = 0; i < attrib.texcoords.size(); i += 2) {
            m_tex_coords.emplace_back(Vector2f(attrib.texcoords[i],
                                               attrib.texcoords[i + 1]));
        }

        const auto &indices = shapes[0].mesh.indices;

        for (int i = 0; i < indices.size(); i += 3) {
            m_vertex_indices.emplace_back(Vector3i(indices[i].vertex_index,
                                                   indices[i + 1].vertex_index,
                                                   indices[i + 2].vertex_index));

            m_normal_indices.emplace_back(Vector3i(indices[i].normal_index,
                                                   indices[i + 1].normal_index,
                                                   indices[i + 2].normal_index));

            m_tex_coord_indices.emplace_back(Vector3i(indices[i].texcoord_index,
                                                      indices[i + 1].texcoord_index,
                                                      indices[i + 2].texcoord_index));
        }

        std::vector<Float> triangle_area_vec;
        triangle_area_vec.resize(m_vertex_indices.size());

        // Initialize m_triangle_pdf used in sampling triangle.
        for(int i=0;i<m_vertex_indices.size();i++){
            Float ith_tri_area = get_triangle(i).get_area();
            triangle_area_vec[i] = ith_tri_area;
            m_area += ith_tri_area;
        }
        m_triangle_pdf = Distrib1D(triangle_area_vec);

        m_aabb = AABB({min_x, min_y, min_z}, {max_x, max_y, max_z});

        LOG(" - Loading complete");
        LOG(" - Building accelation structure...");
        m_accel = std::make_unique<Octree>(*this);
        m_accel->build();
        LOG(" - Done.");
    }

    std::tuple<bool, RayIntersectInfo> OBJMesh::ray_intersect(const Ray &ray) const {
        return m_accel->ray_intersect(ray);
    }

    void OBJMesh::transform(const Matrix44f &transform) {
        if(m_vertices.empty()){
            ERROR("OBJ file is not loaded yet");
        }
        for(auto &m_vertice : m_vertices){
            m_vertice = transform_point(m_vertice, transform);
        }
        for(auto &m_normal : m_normals){
            m_normal = transform_point(m_normal, transform);
        }
    }

    AABB OBJMesh::get_aabb() const {
        return m_aabb;
    }

    Float OBJMesh::get_area() const{
        return m_area;
    }

    std::tuple<Vector3f, Vector3f, Float> OBJMesh::sample_point(Sampler &sampler) const{
        // Sample triangle considering area
        Index i = m_triangle_pdf.sample(sampler.sample_1d());

        // Sample point in chosen triangle
        auto [pos, normal, _] = get_triangle(i).sample_point(sampler);

        return {pos, normal, Float1 / m_area};
    }

    Triangle OBJMesh::get_triangle(Index i) const {
        if (is_vn_exists){
            return Triangle(m_vertices[m_vertex_indices[i][0]],
                            m_vertices[m_vertex_indices[i][1]],
                            m_vertices[m_vertex_indices[i][2]],
                            m_normals[m_normal_indices[i][0]],
                            m_normals[m_normal_indices[i][1]],
                            m_normals[m_normal_indices[i][2]]);
        }
        else{
            return Triangle(m_vertices[m_vertex_indices[i][0]],
                            m_vertices[m_vertex_indices[i][1]],
                            m_vertices[m_vertex_indices[i][2]]);
        }
    }
}