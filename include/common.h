//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
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

#include <limits>
#include <Peanut/Peanut.h>

namespace Caramel{
    using Float = float;
    using Int = int;
    using Index = unsigned int;

    constexpr Float Float0 = static_cast<Float>(0);
    constexpr Float Float1 = static_cast<Float>(1);
    constexpr Float Float2 = static_cast<Float>(2);
    constexpr Float Float0_5 = static_cast<Float>(0.5f);
    constexpr Int Int0 = static_cast<Int>(0);
    constexpr Int Int1 = static_cast<Int>(1);

    constexpr Float INF = std::numeric_limits<Float>::infinity();

    constexpr Float PI = 3.14159265359;
    constexpr Float PI_2 = PI * 2;
    constexpr Float PI_4 = PI * 4;
    constexpr Float PI_HALF = PI * Float(0.5);
    constexpr Float PI_INV = 1 / PI;
    constexpr Float PI_2_INV = 1 / PI_2;
    constexpr Float PI_4_INV = 1 / PI_4;

    // TODO : lower epsilon...
    constexpr Float EPSILON = static_cast<Float>(1e-3);

    inline Float deg_to_rad(Float deg){
        return deg * PI / static_cast<Float>(180);
    }

    inline Float rad_to_deg(Float rad){
        return rad * static_cast<Float>(180) * PI_INV;
    }

    using Matrix22f = Peanut::Matrix<Float, 2, 2>;
    using Matrix33f = Peanut::Matrix<Float, 3, 3>;
    using Matrix44f = Peanut::Matrix<Float, 4, 4>;

    using Vector2f = Peanut::Matrix<Float, 2, 1>;
    using Vector3f = Peanut::Matrix<Float, 3, 1>;
    using Vector4f = Peanut::Matrix<Float, 4, 1>;

    using Vector2i = Peanut::Matrix<Int, 2, 1>;
    using Vector3i = Peanut::Matrix<Int, 3, 1>;

    using Vector2ui = Peanut::Matrix<Index, 2, 1>;

    static const Vector3f vec3f_zero{Float0, Float0, Float0};
    static const Vector3f vec3f_one{Float1, Float1, Float1};

    // returns squared sin value between given normalized vector and (0, 0, 1)
    inline Float vec_sin_2(const Vector3f &v){
        return Float1 - (v[2] * v[2]);
    }

    // returns sin value between given normalized vector and (0, 0, 1)
    inline Float vec_sin(const Vector3f &v){
        return static_cast<Float>(vec_sin_2(v));
    }                                                                                //              ^  Z   ^
                                                                                     //              |     /!  given v
    // returns cos value of phi of given vector (in spherical coordinate system)     //              |    / !
    inline Float vec_cos_phi(const Vector3f &v){                                     //              |   /  !
        const Float sin = vec_sin(v);                                                //              |  /   !
        return v[0] / sin;                                                           //              | /    !    v[0]
    }                                                                                //              |---@--!----.---> X
                                                                                     //             /  \  @ !   /
    // returns sin value of phi of given vector (in spherical coordinate system)     //            /     @  !  /
    inline Float vec_sin_phi(const Vector3f &v){                                     //           /        \! /
        const Float sin = vec_sin(v);                                                //     v[1] /----------.
        return v[1] / sin;                                                           //         /
    }                                                                                //        v   Y

    // TODO : Implement in Peanut
    inline Vector3f cross(const Vector3f &a, const Vector3f &b){
        return {a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    // TODO : Implement in Peanut
    inline Vector3f div(const Vector3f &a, const Vector3f &b){
        return {a[0]/b[0], a[1]/b[1], a[2]/b[2]};
    }

    // TODO : Implement in Peanut
    inline Vector3f sqrt(const Vector3f &a){
        return {std::sqrt(a[0]), std::sqrt(a[1]), std::sqrt(a[2])};
    }

    inline Float L2(const Vector3f &a, const Vector3f &b) {
        return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
    }

    template <typename T>
    inline T interpolate(const T &a, const T &b, const T &c, Float u, Float v){
        return (a*(Float1-u-v)) + (b*u) + (c*v);
    }

    inline bool is_zero(const Vector3f &v){
        return v[0] == Float0 &&
               v[1] == Float0 &&
               v[2] == Float0;
    }
}
