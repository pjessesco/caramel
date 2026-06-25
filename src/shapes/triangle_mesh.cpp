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
#include <map>

#include <shape.h>

#include <light.h>
#include <logger.h>
#include <mesh_accel.h>
#include <polygon_sampling.h>
#include <rayintersectinfo.h>
#include <sampler.h>

namespace Caramel {
    TriangleMesh::TriangleMesh(BSDF *bsdf, AreaLight *arealight) : Shape{bsdf, arealight} {}

    TriangleMesh::~TriangleMesh() = default;

    void TriangleMesh::finalize(AreaLight *arealight, const std::string &name) {
        Vector3f mn{INF, INF, INF};
        Vector3f mx{-INF, -INF, -INF};
        for (const auto &v : m_vertices) {
            for (int a = 0; a < 3; ++a) {
                mn[a] = std::min(mn[a], v[a]);
                mx[a] = std::max(mx[a], v[a]);
            }
        }
        m_aabb = AABB(mn, mx);

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

        m_accel = std::make_unique<BVHMesh>(*this, Float1, Float1, 32, 1);
        m_accel->build();

        if (AreaLight::TRY_SOLID_ANGLE_SAMPLING && arealight != nullptr) {
            // Check coplanarity of all triangles
            const auto& idx0 = m_face_indices[0];
            const Vector3f ref_normal = Vector3f::cross(
                m_vertices[idx0[1]] - m_vertices[idx0[0]],
                m_vertices[idx0[2]] - m_vertices[idx0[0]]).normalize();

            bool coplanar = true;
            constexpr Float coplanar_eps = Float(1e-4);

            for (Index i = 0; i < m_face_indices.size() && coplanar; ++i) {
                const auto& idx = m_face_indices[i];
                const Vector3f tri_normal = Vector3f::cross(
                    m_vertices[idx[1]] - m_vertices[idx[0]],
                    m_vertices[idx[2]] - m_vertices[idx[0]]).normalize();

                if (ref_normal.dot(tri_normal) <= Float1 - coplanar_eps) {
                    coplanar = false;
                    break;
                }

                for (int j = 0; j < 3; ++j) {
                    using std::abs;
                    if (abs(ref_normal.dot(m_vertices[idx[j]] - m_vertices[idx0[0]])) > coplanar_eps) {
                        coplanar = false;
                        break;
                    }
                }
            }

            if (!coplanar) {
                return;
            }

            // Build edge map: (min_idx, max_idx) -> count
            std::map<std::pair<Int, Int>, int> edge_count;
            for (Index i = 0; i < m_face_indices.size(); ++i) {
                const auto& idx = m_face_indices[i];
                for (int e = 0; e < 3; ++e) {
                    Int a = idx[e];
                    Int b = idx[(e + 1) % 3];
                    auto edge = std::make_pair(std::min(a, b), std::max(a, b));
                    edge_count[edge]++;
                }
            }

            // Find boundary edges (count == 1) and build adjacency
            std::map<Int, std::vector<Int>> adjacency;
            for (const auto& [edge, count] : edge_count) {
                if (count == 1) {
                    adjacency[edge.first].push_back(edge.second);
                    adjacency[edge.second].push_back(edge.first);
                }
            }

            if (adjacency.empty()) {
                return;
            }

            // Walk boundary edges to get ordered vertex indices
            std::vector<Int> boundary_indices;
            Int start = adjacency.begin()->first;
            Int current = start;
            Int prev = -1;
            do {
                boundary_indices.push_back(current);
                const auto& neighbors = adjacency[current];
                Int next = (neighbors[0] != prev) ? neighbors[0] : neighbors[1];
                prev = current;
                current = next;
            } while (current != start);

            if (boundary_indices.size() > MAX_POLYGON_VERTEX_COUNT) {
                return;
            }

            // Ensure boundary winding matches original triangle winding
            const Vector3f boundary_normal = Vector3f::cross(
                m_vertices[boundary_indices[1]] - m_vertices[boundary_indices[0]],
                m_vertices[boundary_indices[2]] - m_vertices[boundary_indices[0]]);
            if (ref_normal.dot(boundary_normal) < Float0) {
                std::reverse(boundary_indices.begin(), boundary_indices.end());
            }

            m_is_solid_angle_sampling_possible = true;
            CRM_LOG("Area light solid angle sampling enabled : " + name)
            m_polygon_vertices.resize(boundary_indices.size());
            for (Index i = 0; i < boundary_indices.size(); ++i) {
                m_polygon_vertices[i] = m_vertices[boundary_indices[i]];
            }
        }
    }

    std::pair<bool, RayIntersectInfo> TriangleMesh::ray_intersect(const Ray &ray, Float maxt) const {
        return m_accel->ray_intersect(ray, maxt);
    }

    AABB TriangleMesh::get_aabb() const {
        return m_aabb;
    }

    Float TriangleMesh::get_area() const {
        return m_area;
    }

    std::tuple<Vector3f, Vector3f, Float> TriangleMesh::sample_point(Sampler &sampler) const {
        // Sample triangle considering area
        const Index i = m_triangle_pdf.sample(sampler.sample_1d());

        // Sample point in chosen triangle
        const auto [pos, normal, _] = get_triangle_sample_point(i, sampler);

        return {pos, normal, Float1 / m_area};
    }

    Float TriangleMesh::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const {
        const Vector3f shape_to_hitpos_world = hitpos_world - shapepos_world;
        using std::abs;
        const Float cos = abs(shape_normal_world.dot(shape_to_hitpos_world.normalize()));
        const Float dist_squared = shape_to_hitpos_world.dot(shape_to_hitpos_world);
        return dist_squared / (cos * m_area);
    }

    Float TriangleMesh::get_triangle_area(Index i) const {
        const Vector3i& idx = m_face_indices[i];
        const Vector3f &p0 = m_vertices[idx[0]];
        const Vector3f &p1 = m_vertices[idx[1]];
        const Vector3f &p2 = m_vertices[idx[2]];

        return Vector3f::cross(p1 - p0, p2 - p0).length() * Float0_5;
    }

    std::tuple<Vector3f, Vector3f, Float> TriangleMesh::get_triangle_sample_point(Index i, Sampler &sampler) const {
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

    std::pair<bool, RayIntersectInfo> TriangleMesh::get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const {
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
        ret.tri_index = i;

        if (is_tx_exists) {
            const Vector2f &uv0 = m_tex_coords[idx[0]];
            const Vector2f &uv1 = m_tex_coords[idx[1]];
            const Vector2f &uv2 = m_tex_coords[idx[2]];
            ret.tex_uv = interpolate(uv0, uv1, uv2, u, v);
        } else {
            ret.tex_uv = Vector2f{u, v};
        }

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

    AABB TriangleMesh::get_triangle_aabb(Index i) const {
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

    std::tuple<Vector3f, Vector3f, Vector3f> TriangleMesh::get_triangle_vertices(Index i) const {
        const Vector3i& idx = m_face_indices[i];
        return {m_vertices[idx[0]], m_vertices[idx[1]], m_vertices[idx[2]]};
    }

    Index TriangleMesh::sample_triangle_index(Float u) const {
        return m_triangle_pdf.sample(u);
    }

    Float TriangleMesh::triangle_select_pdf(Index i) const {
        return m_triangle_pdf.pdf(i);
    }

    bool TriangleMesh::is_solid_angle_sampling_possible() const {
        return m_is_solid_angle_sampling_possible;
    }

    const std::vector<Vector3f>& TriangleMesh::get_polygon_vertices() const {
        return m_polygon_vertices;
    }
}
