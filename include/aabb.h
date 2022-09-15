//
// Created by Jino Park on 2022/08/29.
//

#pragma once

#include <cmath>

#include <common.h>

namespace Caramel{

    class Triangle;
    class Ray;

    struct AABB {
        AABB();
        AABB(const Vector3f &p1, const Vector3f &p2);

        bool is_overlap(const AABB &aabb) const;
        bool is_overlap(const Triangle &triangle) const;

        bool is_contain(const Vector3f &vec) const;

        Vector3f corner(Index i) const;

        std::tuple<bool, Float, Float> ray_intersect(const Ray &ray) const;

        Vector3f m_min;
        Vector3f m_max;
    };
}
