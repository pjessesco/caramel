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

#pragma once

#include <vector>
#include <tuple>
#include <cmath>
#include <filesystem>

#include <common.h>
#include <distrib1D.h>
#include <aabb.h>

namespace Caramel{
    class BSDF;
    class Light;
    class AreaLight;
    class RayIntersectInfo;
    class AABB;
    class Ray;
    class Sampler;
    class Distrib1D;
    class AccelerationMesh;

    class Shape{
    public:
        Shape(BSDF *bsdf, AreaLight *arealight);
        virtual ~Shape() = default;

        // u, v, t
        virtual std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) const = 0;
        virtual AABB get_aabb() const = 0;
        virtual Float get_area() const = 0;
        // point, normal, probability
        virtual std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const = 0;
        virtual Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const = 0;

        inline bool is_light() const{
            return m_arealight != nullptr;
        }

        inline AreaLight *get_arealight() const{
            return m_arealight;
        }

        inline BSDF* get_bsdf() const{
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
        Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2);

        Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2,
                 const Vector3f &n0, const Vector3f &n1, const Vector3f &n2);

        // u, v, t
        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        // point, normal, probability
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;

        inline Vector3f point(Index i) const{
            return i == 0 ? m_p0 : i == 1 ? m_p1 : m_p2;
        }
        inline Vector3f normal(Index i) const{
            return i == 0 ? m_n0 : i == 1 ? m_n1 : m_n2;
        }

    private:
        Vector3f m_p0, m_p1, m_p2;
        Vector3f m_n0, m_n1, m_n2;
        const bool is_vn_exists;
    };

    class OBJMesh final : public Shape{
    public:
        OBJMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight = nullptr, const Matrix44f &transform = Matrix44f::identity());

        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) const override;
        AABB get_aabb() const override;
        Float get_area() const override;
        // point, normal, probability
        std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &sampler) const override;
        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const override;

        Triangle get_triangle(Index i) const;

        inline Index get_triangle_num() const{
            return m_vertex_indices.size();
        }

    private:
        Distrib1D m_triangle_pdf;
        Float m_area;
        AABB m_aabb;
        bool is_vn_exists;
        std::unique_ptr<AccelerationMesh> m_accel;
        std::vector<Vector3f> m_vertices;
        std::vector<Vector3f> m_normals;
        std::vector<Vector2f> m_tex_coords;
        std::vector<Vector3i> m_vertex_indices;
        std::vector<Vector3i> m_normal_indices;
        std::vector<Vector3i> m_tex_coord_indices;
    };

    // u, v, t
    std::tuple<Float, Float, Float> moller_trumbore(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2);
    // u, v, t
    // https://jcgt.org/published/0002/01/05/paper.pdf
    std::tuple<Float, Float, Float> watertight_intersection(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2);



}
