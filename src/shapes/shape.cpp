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

namespace Caramel {
    Shape::Shape(std::unique_ptr<BSDF> bsdf) : m_bsdf{std::move(bsdf)}, m_arealight{nullptr} {}

    std::tuple<Float, Float, Float> moeller_trumbore(const Ray &ray, const Vector3f &p0, const Vector3f &p1, const Vector3f &p2){
        const Vector3f D = ray.m_d;

        const Vector3f T = ray.m_o - p0;
        const Vector3f E1 = p1 - p0;
        const Vector3f E2 = p2 - p0;

        const Vector3f DE2 = cross(D, E2);

        const Float denom = DE2.dot(E1);
        if(std::abs(denom) < EPSILON){
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
    std::tuple<Float, Float, Float> waterright_intersection(const Ray &ray, const Vector3f p0, const Vector3f p1, const Vector3f p2){

        // Shift vertices
        Vector3f _p0 = p0 - ray.m_o;
        Vector3f _p1 = p1 - ray.m_o;
        Vector3f _p2 = p2 - ray.m_o;

        // Permute to make z value largest.
        Index largest_idx = ray.m_d[0] > ray.m_d[1] ?
                                                    ray.m_d[0] > ray.m_d[2] ?
                                                                            0 :
                                                                            2
                                                    : ray.m_d[1] > ray.m_d[2] ?
                                                                              1 :
                                                                              2;
        Index rest1, rest2;

        switch (largest_idx) {
            case 0:
                rest1 = 1;
                rest2 = 2;
                break;
            case 1:
                rest1 = 2;
                rest2 = 0;
                break;
            case 2:
                rest1 = 0;
                rest2 = 1;
                break;
        }

        _p0 = {_p0[rest1], _p0[rest2], _p0[largest_idx]};
        _p1 = {_p1[rest1], _p1[rest2], _p1[largest_idx]};
        _p2 = {_p2[rest1], _p2[rest2], _p2[largest_idx]};

        // Shear
        const Float sx = -ray.m_d[0] / ray.m_d[2];
        const Float sy = -ray.m_d[1] / ray.m_d[2];
        const Float sz = Float1 / ray.m_d[2];

        _p0[0] += sx * _p0[2];
        _p0[1] += sy * _p0[2];
        _p1[0] += sx * _p1[2];
        _p1[1] += sy * _p1[2];
        _p2[0] += sx * _p2[2];
        _p2[1] += sy * _p2[2];

        const Matrix33f M{Float1, Float0, sx,
                          Float0, Float1, sy,
                          Float0, Float0, sz};

        const Float U = _p2[0] * _p1[1] - _p2[1] * _p1[0];
        const Float V = _p0[0] * _p2[1] - _p0[1] * _p2[0];
        const Float W = _p1[0] * _p0[1] - _p1[1] * _p0[0];

        if(U<0 || V<0 || W<0){
            return {-1, -1, -1};
        }

        const Float det = U + V + W;
        if(std::abs(det)<EPSILON){
            return {-1, -1, -1};
        }

        const Float T = U * _p0[2] + V * _p1[2] + W * _p2[2];

        if(T < EPSILON){
            return {-1, -1, -1};
        }
        return {U/det, V/det, T/det};

    }
}