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
#include <cmath>

#include <common.h>
#include <logger.h>
#include <ray.h>
#include <aabb.h>
#include <shape.h>

namespace Caramel{
    AABB::AABB() {}
    AABB::AABB(const Vector3f &p1, const Vector3f &p2) {
        m_min = Vector3f{std::min({p1[0], p2[0]}),
                         std::min({p1[1], p2[1]}),
                         std::min({p1[2], p2[2]})};

        m_max = Vector3f{std::max({p1[0], p2[0]}),
                         std::max({p1[1], p2[1]}),
                         std::max({p1[2], p2[2]})};
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
        return Vector3f(i & 1 ? m_min[0] : m_max[0],
                        i & 2 ? m_min[1] : m_max[1],
                        i & 4 ? m_min[2] : m_max[2]);
    }

    std::tuple<bool, Float, Float> AABB::ray_intersect(const Ray &ray) const{
        Float tmin = Float0;
        Float tmax = INF;
        for(Index i=0;i<3;i++){
            Float t1 = (m_min[i] - ray.m_o[i]) / ray.m_d[i];
            Float t2 = (m_max[i] - ray.m_o[i]) / ray.m_d[i];
            tmin = std::min(std::max(t1, tmin), std::max(t2, tmin));
            tmax = std::max(std::min(t1, tmax), std::min(t2, tmax));
        }
        return {tmin <= tmax, tmin, tmax};
    }
}
