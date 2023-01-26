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

#include <tuple>
#include <algorithm>

#include <shape.h>

#include <transform.h>
#include <light.h>
#include <ray.h>

namespace Caramel {
    Shape::Shape(BSDF *bsdf, AreaLight *arealight) : m_bsdf{bsdf}, m_arealight{arealight} {
        if(arealight!= nullptr){
            arealight->m_shape = this;
        }
    }

    std::tuple<Float, Float, Float> moller_trumbore(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2){
        const Vector3f D = ray.m_d;

        const Vector3f T = ray.m_o - p0;
        const Vector3f E1 = p1 - p0;
        const Vector3f E2 = p2 - p0;

        const Vector3f DE2 = cross(D, E2);

        const Float denom = DE2.dot(E1);
        if(std::abs(denom) < 0){
            return {-Float1, -Float1, -Float1};
        }
        const Float denom_inv = static_cast<Float>(1) / DE2.dot(E1);

        const Vector3f TE1 = cross(T, E1);
        const Float v = TE1.dot(D) * denom_inv;
        if (v < Float0 || Float1 < v) {
            return {-Float1, -Float1, -Float1};
        }

        const Float u = DE2.dot(T) * denom_inv;
        if (u < Float0 || Float1 < u + v) {
            return {-Float1, -Float1, -Float1};
        }

        const Float t = TE1.dot(E2) * denom_inv;
        if(t <= ray.m_min_t){
            return {-Float1, -Float1, -Float1};
        }

        return {u, v, t};
    }

    // https://jcgt.org/published/0002/01/05/paper.pdf
    std::tuple<Float, Float, Float> watertight_intersection(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2){

        // Calculate dimension where the ray direction is maximal
        Index idx_z = ray.m_d[0] > ray.m_d[1] ? ray.m_d[0] > ray.m_d[2] ? 0 :
                                                                          2 :
                                                ray.m_d[1] > ray.m_d[2] ? 1 :
                                                                          2;
        Index idx_x = idx_z == 2 ? 0 : idx_z + 1;
        Index idx_y = idx_x == 2 ? 0 : idx_x + 1;

        // Swap kx and ky dimension to preserve winding direction of triangles
        if(ray.m_d[idx_z] < Float0){
            std::swap(idx_x, idx_y);
        }

        // Calculate shear constants
        const Float sx = ray.m_d[idx_x] / ray.m_d[idx_z];
        const Float sy = ray.m_d[idx_y] / ray.m_d[idx_z];
        const Float sz = Float1 / ray.m_d[idx_z];

        // Calculate vertices relative to ray origin
        const Vector3f A = p0 - ray.m_o;
        const Vector3f B = p1 - ray.m_o;
        const Vector3f C = p2 - ray.m_o;

        // Perform shear and scale of vertices
        const Float ax = A[idx_x] - sx * A[idx_z];
        const Float ay = A[idx_y] - sy * A[idx_z];
        const Float bx = B[idx_x] - sx * B[idx_z];
        const Float by = B[idx_y] - sy * B[idx_z];
        const Float cx = C[idx_x] - sx * C[idx_z];
        const Float cy = C[idx_y] - sy * C[idx_z];

        // Calculate scaled barycentric coordinates
        Float U = cx*by - cy*bx;
        Float V = ax*cy - ay*cx;
        Float W = bx*ay - by*ax;

        // Fallback to test against edges using double precision
        // Note that we are using `Float` keyword instead of standard `float`/`double` in paper.
        if(U==Float0 || V==Float0 || W==Float0){
            U = cx*by - cy*bx;
            V = ax*cy - ay*cx;
            W = bx*ay - by*ax;
        }

        // Perform edge tests. Moving this test before and at the end
        // of the previous conditional gives higher performances.
        // Assumes backface culling
        if(U<Float0 || V<Float0 || W<Float0){
            return {-Float1, -Float1, -Float1};
        }

        // Calculate determinant
        Float det = U + V + W;
        if(det == Float0){
            return {-Float1, -Float1, -Float1};
        }

        // Calculate scaled z-coordinates of vertices and use them to calculate the hit distance
        const Float az = sz * A[idx_z];
        const Float bz = sz * B[idx_z];
        const Float cz = sz * C[idx_z];
        const Float T = U*az + V*bz + W*cz;

        // Assumes backface culling
        if(T <= ray.m_min_t * det){
            return {-Float1, -Float1, -Float1};
        }

        // Normalize U, V, W, and T
        const Float inv_det = Float1 / det;
        // Return v and to match with trumbore_moeller implementation
        return {V * inv_det, W * inv_det, T * inv_det};
    }
}