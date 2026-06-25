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
    struct MeshAccel;

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

    class TriangleMesh : public Shape{
    public:
        TriangleMesh(BSDF *bsdf, AreaLight *arealight);
        ~TriangleMesh() override;

        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        // point, normal, probability
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;
        bool is_solid_angle_sampling_possible() const override;
        const std::vector<Vector3f>& get_polygon_vertices() const override;

        Index get_triangle_num() const { return m_face_indices.size(); }
        std::pair<bool, RayIntersectInfo> get_triangle_ray_intersect(Index i, const Ray &ray, Float maxt) const;
        AABB get_triangle_aabb(Index i) const;
        Float get_triangle_area(Index i) const;
        std::tuple<Vector3f, Vector3f, Float> get_triangle_sample_point(Index i, Sampler &sampler) const;
        std::tuple<Vector3f, Vector3f, Vector3f> get_triangle_vertices(Index i) const;
        Index sample_triangle_index(Float u) const;
        Float triangle_select_pdf(Index i) const;

    protected:
        void finalize(AreaLight *arealight, const std::string &name);

        Distrib1D m_triangle_pdf;
        Float m_area = Float0;
        AABB m_aabb;
        bool is_vn_exists = false;
        bool is_tx_exists = false;
        std::unique_ptr<MeshAccel> m_accel;
        std::vector<Vector3f> m_vertices;
        std::vector<Vector3f> m_normals;
        std::vector<Vector2f> m_tex_coords;
        std::vector<Vector3i> m_face_indices;

        // for solid angle sampling
        std::vector<Vector3f> m_polygon_vertices;
        bool m_is_solid_angle_sampling_possible = false;
    };

    class PLYMesh final : public TriangleMesh{
    public:
        PLYMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight = nullptr, const Matrix44f &transform = Matrix44f::identity());
    };

    class OBJMesh final : public TriangleMesh{
    public:
        OBJMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight = nullptr, const Matrix44f &transform = Matrix44f::identity());
    };

    class InlineTriangleMesh final : public TriangleMesh{
    public:
        InlineTriangleMesh(std::vector<Vector3f> positions,
                           std::vector<Vector3i> indices,
                           std::vector<Vector3f> normals,
                           BSDF *bsdf, AreaLight *arealight = nullptr,
                           const Matrix44f &transform = Matrix44f::identity());
    };

    // Instanced geometry. Shares a template Shape (kept in LOCAL space) and
    // applies a per-placement transform at intersection time, so one template's
    // geometry can be reused across many placements without copying vertices.
    class Instance final : public Shape{
    public:
        // bsdf belongs to this placement: the scene BVH overwrites info.shape with
        // the top-level Shape* (this Instance), so the integrator reads get_bsdf()
        // from here, not from the template.
        Instance(const Shape *geometry, const Matrix44f &to_world, BSDF *bsdf, AreaLight *arealight = nullptr);

        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;
        bool is_solid_angle_sampling_possible() const override;
        const std::vector<Vector3f>& get_polygon_vertices() const override;

        // The shared template geometry pointer (test accessor for sharing checks).
        const Shape *geometry() const { return m_geometry; }

    private:
        const Shape *m_geometry;       // shared template, LOCAL space (intersection only)
        Matrix44f m_to_world;
        Matrix44f m_to_local;          // Inverse(to_world)
        AABB m_world_aabb;
        std::vector<Vector3f> m_world_polygon_vertices;
    };

    // u, v, t
    std::tuple<Float, Float, Float> moller_trumbore(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2, Float maxt);
    // u, v, t
    // https://jcgt.org/published/0002/01/05/paper.pdf
    std::tuple<Float, Float, Float> watertight_intersection(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2, Float maxt);



}
