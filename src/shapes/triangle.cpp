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
#include <sampler.h>
#include <rayintersectinfo.h>

namespace Caramel {
    Triangle::Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2)
        : Shape(nullptr, nullptr), m_p0{p0}, m_p1{p1}, m_p2{p2}, is_vn_exists{false} {}

    Triangle::Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2,
                       const Vector3f &n0, const Vector3f &n1, const Vector3f &n2)
        : Shape(nullptr, nullptr), m_p0{p0}, m_p1{p1}, m_p2{p2}, m_n0{n0}, m_n1{n1}, m_n2{n2}, is_vn_exists{true} {}

    AABB Triangle::get_aabb() const{
        return AABB(Vector3f{std::min({m_p0[0], m_p1[0], m_p2[0]}),
                             std::min({m_p0[1], m_p1[1], m_p2[1]}),
                             std::min({m_p0[2], m_p1[2], m_p2[2]})},
                    Vector3f{std::max({m_p0[0], m_p1[0], m_p2[0]}),
                             std::max({m_p0[1], m_p1[1], m_p2[1]}),
                             std::max({m_p0[2], m_p1[2], m_p2[2]})});
    }

    Float Triangle::get_area() const{
        return cross(m_p1 - m_p0, m_p2 - m_p0).length() * Float0_5;
    }

    std::tuple<Vector3f, Vector3f, Float> Triangle::sample_point(Sampler &sampler) const{
        const Float u = sampler.sample_1d();
        const Float v = sampler.sample_1d();
        const Float x = Float1 - std::sqrt(Float1 - u);
        const Float y = v * std::sqrt(Float1 - u);
        // z = 1 - x - y

        return {interpolate(m_p0, m_p1, m_p2, x, y),
                is_vn_exists ?
                             interpolate(normal(0), normal(1), normal(2), x, y).normalize() :
                             cross(m_p1 - m_p0, m_p2 - m_p0).normalize(),
                Float1 / get_area()};
    }

    inline Vector3f Triangle::point(Index i) const {
        return i == 0 ? m_p0 : i == 1 ? m_p1 : m_p2;
    }

    inline Vector3f Triangle::normal(Index i) const {
        return i == 0 ? m_n0 : i == 1 ? m_n1 : m_n2;
    }

    // u, v, t
    std::tuple<bool, RayIntersectInfo> Triangle::ray_intersect(const Ray &ray) const {

        auto [u, v, t] = watertight_intersection(ray, m_p0, m_p1, m_p2);

        if(u==-Float1 && v==-Float1 && t==-Float1){
            return {false, RayIntersectInfo()};
        }

        // Intersect
        Vector3f hitpos = interpolate(m_p0, m_p1, m_p2, u, v);


        RayIntersectInfo ret;
        ret.p = hitpos;
        ret.t = t;
        ret.u = u;
        ret.v = v;

        if(is_vn_exists){
            Vector3f shn = interpolate(m_n0, m_n1, m_n2, u, v);
            ret.sh_coord = Coordinate(shn.normalize());
        }
        else{
            const Vector3f E1 = m_p1 - m_p0;
            const Vector3f E2 = m_p2 - m_p0;
            ret.sh_coord = Coordinate(cross(E1, E2).normalize());
        }

        return {true, ret};
    }
}