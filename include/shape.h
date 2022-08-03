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

#include <vector>
#include <filesystem>
#include <tuple>

#include <common.h>
#include <logger.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

namespace Caramel{

    struct Shape{
        Shape() {}

        virtual void transform(const Matrix44f &transform) = 0;

        Vector3f m_bbox_min;
        Vector3f m_bbox_max;
    };

    struct Triangle : Shape{
        Triangle() {}
        Triangle(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3,
                 const Vector3f &n1, const Vector3f &n2, const Vector3f &n3) {
            m_p.col(0) = p1;
            m_p.col(1) = p2;
            m_p.col(2) = p3;
            m_n.col(0) = n1;
            m_n.col(1) = n2;
            m_n.col(2) = n3;

            // TODO
            // m_bbox_min =
            // m_bbox_max =
        }

        inline Vector3f point(Index i) const{
            return m_p.col(i);
        }

        inline Vector3f normal(Index i) const{
            return m_n.col(i);
        }

        BoolUVD ray_intersect(const Ray &ray) const override{
            const Vector3f e1 = point(1) - point(0);
            const Vector3f e2 = point(2) - point(0);
            const Vector3f n_geo = e1.cross(e2);

            const Vector3f p_vec = ray.m_d.cross(e2);

            const Float denominator = p_vec.dot(e1);
            if(-EPSILON < denominator && denominator < EPSILON){
                return {false, Vector2f(), -1};
            }

            const Vector3f t_vec = ray.m_o - point(0);
            const Float u = p_vec.dot(t_vec) / denominator;
            if(u < 0 || 1 < u){
                return {false, Vector2f(), -1};
            }

            const Vector3f q_vec = t_vec.cross(e1);
            const Float v = q_vec.dot(ray.m_d) / denominator;
            if(v < 0 || 1 < v){
                return {false, Vector2f(), -1};
            }

            if(u+v>1) return {false, Vector2f(), -1};

            const Float t = q_vec.dot(e2) / denominator;

            return {true, Vector2f(u, v), t};
        }

        void transform(const Matrix44f &transform) override{
            Matrix44f points_tmp = Matrix44f::identity();
            points_tmp.block<3,3>(0,0) = m_p;
            points_tmp = transform * points_tmp;
            m_p = points_tmp.block<3, 3>(0, 0);

            Matrix44f normals_tmp = Matrix44f::Identity();
            normals_tmp.block<3, 3>(0, 0) = m_n;
            normals_tmp = transform.inverse().transpose() * normals_tmp;
            m_n = normals_tmp.block<3, 3>(0, 0);
        }

        Matrix33f m_p;
        Matrix33f m_n;
    };

    struct OBJMesh : Shape{

    };

    struct PLYMesh : Shape{

    };
}
