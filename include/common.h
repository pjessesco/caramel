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

#include <limits>
#include <Peanut/Peanut.h>

namespace Caramel{
    using Float = float;
    using Int = int;
    using Index = unsigned int;

    constexpr Float INF = std::numeric_limits<Float>::infinity();

    constexpr Float PI = 3.14159265359;
    constexpr Float PI_2 = 6.28318530718;
    constexpr Float PI_HALF = 1.57079632679;
    constexpr Float PI_INV = 0.31830988618;
    constexpr Float EPSILON = std::numeric_limits<Float>::epsilon();

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

    // TODO : Implement in Peanut
    template <Index N>
    Float dot(const Peanut::Matrix<Float, N, 1> &a, const Peanut::Matrix<Float, N, 1> &b){
        Float ret = static_cast<Float>(0);
        for(int i=0;i<N;i++){
            ret += a.elem(i, 0) * b.elem(i, 0);
        }
        return ret;
    }

    // TODO : Implement in Peanut
    inline Vector3f cross(const Vector3f &a, const Vector3f &b){
        return Vector3f(a.m_data.d1[1] * b.m_data.d1[2] - a.m_data.d1[2] * b.m_data.d1[1],
                        a.m_data.d1[2] * b.m_data.d1[0] - a.m_data.d1[0] * b.m_data.d1[2],
                        a.m_data.d1[0] * b.m_data.d1[1] - a.m_data.d1[1] * b.m_data.d1[0]);
    }

}
