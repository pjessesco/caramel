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

#include <cmath>

#include <common.h>

namespace Caramel{

    inline auto transform_point(const Vector3f &p, const Matrix44f &mat){
        return Block<0,0,3,1>(mat * Vector4f{p[0], p[1], p[2], Float1}).eval();
    }

    inline auto transform_vector(const Vector3f &v, const Matrix44f &mat){
        return Block<0,0,3,1>(mat * Vector4f{v[0], v[1], v[2], Float0}).eval();
    }

    inline auto transform_normal(const Vector3f &n, const Matrix44f &mat){
        return Block<0, 0, 3, 1>(Inverse(T(mat)) * Vector4f{n[0], n[1], n[2], Float0}).eval();
    }

    inline Matrix44f scale(Float x, Float y, Float z){
        return Matrix44f{     x, Float0, Float0, Float0,
                         Float0,      y, Float0, Float0,
                         Float0, Float0,      z, Float0,
                         Float0, Float0, Float0, Float1};
    }

    inline Matrix44f translate(Float x, Float y, Float z){
        return Matrix44f{Float1, Float0, Float0,      x,
                         Float0, Float1, Float0,      y,
                         Float0, Float0, Float1,      z,
                         Float0, Float0, Float0, Float1};
    }

    inline Matrix44f rotate_x(Float degree){
        const Float s = sin(deg_to_rad(degree));
        const Float c = cos(deg_to_rad(degree));
        return Matrix44f{Float1, Float0, Float0, Float0,
                         Float0,      c,     -s, Float0,
                         Float0,      s,      c, Float0,
                         Float0, Float0, Float0, Float1};
    }

    inline Matrix44f rotate_y(Float degree){
        const Float s = sin(deg_to_rad(degree));
        const Float c = cos(deg_to_rad(degree));
        return Matrix44f{     c, Float0,      s, Float0,
                         Float0, Float1, Float0, Float0,
                             -s, Float0,      c, Float0,
                         Float0, Float0, Float0, Float1};
    }

    inline Matrix44f rotate_z(Float degree){
        const Float s = sin(deg_to_rad(degree));
        const Float c = cos(deg_to_rad(degree));
        return Matrix44f{     c,     -s, Float0, Float0,
                              s,      c, Float0, Float0,
                         Float0, Float0, Float1, Float0,
                         Float0, Float0, Float0, Float1};
    }

}










