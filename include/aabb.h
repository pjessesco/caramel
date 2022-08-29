//
// Created by Jino Park on 2022/08/29.
//

#pragma once

#include <cmath>

#include "common.h"
#include "logger.h"
#include "ray.h"

namespace Caramel{
    struct AABB {
        AABB() {}
        AABB(const Vector3f &min, const Vector3f &max) : m_min{min}, m_max{max} {}

        bool ray_intersect(const Ray &ray) const{
            Float tmin = Float0;
            Float tmax = INF;
            for(Index i=0;i<3;i++){
                Float t1 = (m_min[i] - ray.m_o[i]) / ray.m_d[i];
                Float t2 = (m_max[i] - ray.m_o[i]) / ray.m_d[i];
                tmin = std::min(std::max(t1, tmin), std::max(t2, tmin));
                tmax = std::max(std::min(t1, tmax), std::min(t2, tmax));
            }
            return tmin <= tmax;
        }

        Vector3f m_min;
        Vector3f m_max;
    };
}
