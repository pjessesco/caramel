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

#include <common.h>
#include <ray.h>

namespace Caramel{

    struct Shape{
        Shape() = default;

        virtual void transform(const Matrix44f &transform) = 0;
        // u, v, t
        virtual std::tuple<bool, Float, Float, Float> ray_intersect(const Ray &ray) const = 0;
    };

    struct Triangle : Shape{
        Triangle(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3,
                 const Vector3f &n1, const Vector3f &n2, const Vector3f &n3);

        void transform(const Matrix44f &transform) override;
        inline Vector3f point(Index i) const;
        inline Vector3f normal(Index i) const;

        // u, v, t
        std::tuple<bool, Float, Float, Float> ray_intersect(const Ray &ray) const override;

        Matrix33f m_p;
        Matrix33f m_n;
    };

    struct OBJMesh : Shape{
        OBJMesh(const std::filesystem::path &path);
        std::tuple<bool, Float, Float, Float> ray_intersect(const Ray &ray) const override;
        void transform(const Matrix44f &transform) override;
        Triangle get_triangle(Index i) const;

        std::vector<Vector3f> m_vertices;
        std::vector<Vector3f> m_normals;
        std::vector<Vector2f> m_tex_coords;
        std::vector<Vector3i> m_vertex_indices;
        std::vector<Vector3i> m_normal_indices;
        std::vector<Vector3i> m_tex_coord_indices;
    };

}
