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
#include <cmath>

#include <aabb.h>

#include <common.h>
#include <ray.h>

namespace Caramel{
    AABB::AABB() = default;
    AABB::AABB(const Vector3f &p1, const Vector3f &p2) {
        m_min = Vector3f{std::min(p1[0], p2[0]),
                         std::min(p1[1], p2[1]),
                         std::min(p1[2], p2[2])};

        m_max = Vector3f{std::max(p1[0], p2[0]),
                         std::max(p1[1], p2[1]),
                         std::max(p1[2], p2[2])};
    }

    bool AABB::is_overlap(const AABB &aabb) const{
        return aabb.m_min[0] <= m_max[0] &&
               aabb.m_min[1] <= m_max[1] &&
               aabb.m_min[2] <= m_max[2] &&
               aabb.m_max[0] >= m_min[0] &&
               aabb.m_max[1] >= m_min[1] &&
               aabb.m_max[2] >= m_min[2];
    }
    
    bool AABB::is_contain(const Vector3f &vec) const{
        return m_min[0] <= vec[0] && vec[0] <= m_max[0] &&
               m_min[1] <= vec[1] && vec[1] <= m_max[1] &&
               m_min[2] <= vec[2] && vec[2] <= m_max[2];
    }

    Vector3f AABB::corner(Index i) const{
        return {i & 1 ? m_min[0] : m_max[0],
                i & 2 ? m_min[1] : m_max[1],
                i & 4 ? m_min[2] : m_max[2]};
    }

    Vector3f AABB::offset(const Vector3f &p) const {
        const Vector3f tmp = p - m_min;
        const Vector3f denom = m_max - m_min;
        return {tmp[0] / denom[0], tmp[1] / denom[1], tmp[2] / denom[2]};
    }

    Float AABB::surface_area() const {
        const Vector3f diag = m_max - m_min;
        return ((diag[0] * diag[1]) + (diag[1] * diag[2]) + (diag[2] * diag[0])) * Float2;
    }

    std::pair<bool, Float> AABB::ray_intersect(const Ray &ray, Float maxt) const{
        Float tmin = Float0;
        Float tmax = INF;

        for(Index i=0;i<3;i++){
            const Float o = ray.m_o[i];
            const Float d = ray.m_d[i];
            const Float min = m_min[i];
            const Float max = m_max[i];
            if (Peanut::is_zero(d)) {
                if (o < min || o > max) {
                    return {false, INF};
                }
                continue;
            }
            const Float invd = ray.m_d_recip[i];
            Float t1 = (min - o) * invd;
            Float t2 = (max - o) * invd;

            if (t1 > t2) {
                std::swap(t1, t2);
            }
            tmin = std::max(t1, tmin);
            tmax = std::min(t2, tmax);

            if (tmin > tmax || tmin > maxt) {
                return {false, INF};
            }
        }
        return {tmax >= Float0, tmin};
    }

    int AABB::longest_axis() const {
        const Float len_x = m_max[0] - m_min[0];
        const Float len_y = m_max[1] - m_min[1];
        const Float len_z = m_max[2] - m_min[2];

        return len_x > len_y && len_x > len_z ? 0 :
               len_y > len_x && len_y > len_z ? 1 :
                                                2;
    }

    Vector3f AABB::get_center() const {
        return (m_max + m_min) * Float0_5;
    }

    std::pair<AABB, AABB> AABB::split(int axis) const {
        if (axis == 0/* x */) {
            const Float x_mid = (m_min[0] + m_max[0]) * Float0_5;
            return {{m_min, {x_mid, m_max[1], m_max[2]}}, {{x_mid, m_min[1], m_min[2]}, m_max}};
        }
        else if (axis == 1/* y */) {
            const Float y_mid = (m_min[1] + m_max[1]) * Float0_5;
            return {{m_min, {m_max[0], y_mid, m_max[2]}}, {{m_min[0], y_mid, m_min[2]}, m_max}};
        }
        else if (axis == 2/* z */) {
            const Float z_mid = (m_min[2] + m_max[2]) * Float0_5;
            return {{m_min, {m_max[0], m_max[1], z_mid}}, {{m_min[0], m_min[1], z_mid}, m_max}};
        }
        return {};
    }




}
