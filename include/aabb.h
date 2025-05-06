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

#include <common.h>

namespace Caramel{

    class Triangle;
    class Ray;

    struct AABB {
        AABB();
        AABB(const Vector3f &p1, const Vector3f &p2);

        bool is_overlap(const AABB &aabb) const;

        bool is_contain(const Vector3f &vec) const;

        Vector3f corner(Index i) const;

        std::pair<bool, Float> ray_intersect(const Ray &ray) const;

        Vector3f offset(const Vector3f &p) const;

        Float surface_area() const;

        static AABB merge(const AABB &a, const AABB &b) {
            return {{std::min(a.m_min[0], b.m_min[0]),
                     std::min(a.m_min[1], b.m_min[1]),
                     std::min(a.m_min[2], b.m_min[2])},
                    {std::max(a.m_max[0], b.m_max[0]),
                     std::max(a.m_max[1], b.m_max[1]),
                     std::max(a.m_max[2], b.m_max[2])}};
        }

        static AABB overlapped(const AABB &a, const AABB &b) {
            return {{std::max(a.m_min[0], b.m_min[0]),
                     std::max(a.m_min[1], b.m_min[1]),
                     std::max(a.m_min[2], b.m_min[2])},
                    {std::min(a.m_max[0], b.m_max[0]),
                     std::min(a.m_max[1], b.m_max[1]),
                     std::min(a.m_max[2], b.m_max[2])}};
        }

        std::pair<AABB, AABB> split(int axis) const;

        int longest_axis() const;

        Vector3f get_center() const;

        Vector3f m_min;
        Vector3f m_max;
    };
}
