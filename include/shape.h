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

#pragma once

#include <vector>
#include <tuple>
#include <filesystem>

#include <aabb.h>
#include <common.h>
#include <distribution.h>

namespace Caramel{
    class BSDF;
    class Light;
    class AreaLight;
    class RayIntersectInfo;
    struct AABB;
    class Ray;
    class Sampler;
    class Distrib1D;
    class MeshAccel;

    class Shape{
    public:
        Shape(BSDF *bsdf, AreaLight *arealight);
        virtual ~Shape() = default;

        // u, v, t
        virtual std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const = 0;
        virtual AABB get_aabb() const = 0;
        virtual Float get_area() const = 0;
        // point, normal, probability
        virtual std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const = 0;
        // Probability to sample shapepos_world at hitpos_world respect to solid angle
        virtual Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const = 0;

        virtual bool is_solid_angle_sampling_possible() const = 0;
        virtual const std::vector<Vector3f>& get_polygon_vertices() const = 0;

        Vector3f get_center() const;

        bool is_light() const{
            return m_arealight != nullptr;
        }

        AreaLight *get_arealight() const{
            return m_arealight;
        }

        BSDF* get_bsdf() const{
            return m_bsdf;
        }

        template <typename Type, typename ...Param>
        static Shape* Create(Param ...args){
            return dynamic_cast<Shape*>(new Type(args...));
        }

    private:
        BSDF *m_bsdf;
        // Shape can have arealight only
        AreaLight *m_arealight;
    };

    class Triangle final : public Shape{
    public:
        Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2, BSDF *bsdf);

        Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2,
                 const Vector3f &n0, const Vector3f &n1, const Vector3f &n2, BSDF *bsdf);

        Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2,
                 const Vector3f &n0, const Vector3f &n1, const Vector3f &n2,
                 const Vector2f &uv0, const Vector2f &uv1, const Vector2f &uv2, BSDF *bsdf);

        // u, v, t
        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        // point, normal, probability
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;

        bool is_solid_angle_sampling_possible() const override;
        const std::vector<Vector3f>& get_polygon_vertices() const override;

        Vector3f point(Index i) const{
            return m_points[i];
        }

        Vector3f normal(Index i) const{
            return m_normals[i];
        }

    private:
        std::vector<Vector3f> m_points;
        std::vector<Vector3f> m_normals;
        Vector2f m_uv0, m_uv1, m_uv2;
        const bool is_vn_exists;
        const bool is_tx_exists;
    };

    // Interface for triangle mesh shapes (used by acceleration structures)
    class TriangleMesh : public Shape{
    public:
        using Shape::Shape;
        virtual Index get_triangle_num() const = 0;
        virtual std::pair<bool, RayIntersectInfo> get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const = 0;
        virtual AABB get_triangle_aabb(Index i) const = 0;
        virtual Float get_triangle_area(Index i) const = 0;
        virtual std::tuple<Vector3f, Vector3f, Float> get_triangle_sample_point(Index i, Sampler &sampler) const = 0;
        virtual std::tuple<Vector3f, Vector3f, Vector3f> get_triangle_vertices(Index i) const = 0;
        virtual Index sample_triangle_index(Float u) const = 0;
        virtual Float triangle_select_pdf(Index i) const = 0;
    };

    class OBJMesh final : public TriangleMesh{
    public:
        OBJMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight = nullptr, const Matrix44f &transform = Matrix44f::identity());

        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        // point, normal, probability
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;

        std::pair<bool, RayIntersectInfo> get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const override;
        AABB get_triangle_aabb(Index i) const override;
        Float get_triangle_area(Index i) const override;
        std::tuple<Vector3f, Vector3f, Float> get_triangle_sample_point(Index i, Sampler &sampler) const override;
        std::tuple<Vector3f, Vector3f, Vector3f> get_triangle_vertices(Index i) const override;
        Index sample_triangle_index(Float u) const override;
        Float triangle_select_pdf(Index i) const override;

        Index get_triangle_num() const override {
            return m_vertex_indices.size();
        }

        bool is_solid_angle_sampling_possible() const override;
        const std::vector<Vector3f>& get_polygon_vertices() const override;

    private:
        Distrib1D m_triangle_pdf;
        Float m_area;
        AABB m_aabb;
        bool is_vn_exists;
        bool is_tx_exists;
        std::unique_ptr<MeshAccel> m_accel;
        std::vector<Vector3f> m_vertices;
        std::vector<Vector3f> m_normals;
        std::vector<Vector2f> m_tex_coords;
        std::vector<Vector3i> m_vertex_indices;
        std::vector<Vector3i> m_normal_indices;
        std::vector<Vector3i> m_tex_coord_indices;

        // for solid angle sampling
        std::vector<Vector3f> m_polygon_vertices;
        bool m_is_solid_angle_sampling_possible = false;
    };

    class PLYMesh final : public TriangleMesh{
    public:
        PLYMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight = nullptr, const Matrix44f &transform = Matrix44f::identity());

        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        // point, normal, probability
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;

        std::pair<bool, RayIntersectInfo> get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const override;
        AABB get_triangle_aabb(Index i) const override;
        Float get_triangle_area(Index i) const override;
        std::tuple<Vector3f, Vector3f, Float> get_triangle_sample_point(Index i, Sampler &sampler) const override;
        std::tuple<Vector3f, Vector3f, Vector3f> get_triangle_vertices(Index i) const override;
        Index sample_triangle_index(Float u) const override;
        Float triangle_select_pdf(Index i) const override;

        Index get_triangle_num() const override {
            return m_face_indices.size();
        }

        bool is_solid_angle_sampling_possible() const override;
        const std::vector<Vector3f>& get_polygon_vertices() const override;

    private:
        Distrib1D m_triangle_pdf;
        Float m_area;
        AABB m_aabb;
        bool is_vn_exists;
        std::unique_ptr<MeshAccel> m_accel;
        std::vector<Vector3f> m_vertices;
        std::vector<Vector3f> m_normals;
        std::vector<Vector3i> m_face_indices;

        // for solid angle sampling
        std::vector<Vector3f> m_polygon_vertices;
        bool m_is_solid_angle_sampling_possible = false;
    };

    // u, v, t
    std::tuple<Float, Float, Float> moller_trumbore(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2, Float maxt);
    // u, v, t
    // https://jcgt.org/published/0002/01/05/paper.pdf
    std::tuple<Float, Float, Float> watertight_intersection(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2, Float maxt);



}
