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

#include <tuple>
#include <algorithm>

#include <shape.h>

#include <transform.h>
#include <sampler.h>
#include <rayintersectinfo.h>

namespace Caramel {
    Triangle::Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2, BSDF *bsdf)
        : Shape(bsdf, nullptr), m_p0{p0}, m_p1{p1}, m_p2{p2}, is_vn_exists{false}, is_tx_exists{false} {}

    Triangle::Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2,
                       const Vector3f &n0, const Vector3f &n1, const Vector3f &n2, BSDF *bsdf)
        : Shape(bsdf, nullptr), m_p0{p0}, m_p1{p1}, m_p2{p2}, m_n0{n0}, m_n1{n1}, m_n2{n2}, is_vn_exists{true}, is_tx_exists{false} {}

    Triangle::Triangle(const Vector3f &p0, const Vector3f &p1, const Vector3f &p2,
                       const Vector3f &n0, const Vector3f &n1, const Vector3f &n2,
                       const Vector2f &uv0, const Vector2f &uv1, const Vector2f &uv2, BSDF *bsdf)
        : Shape(bsdf, nullptr), m_p0{p0}, m_p1{p1}, m_p2{p2}, m_n0{n0}, m_n1{n1}, m_n2{n2}, is_vn_exists{true}, is_tx_exists{true},
          m_uv0{uv0}, m_uv1{uv1}, m_uv2{uv2} {}

    AABB Triangle::get_aabb() const{
        return AABB(Vector3f{std::min({m_p0[0], m_p1[0], m_p2[0]}),
                             std::min({m_p0[1], m_p1[1], m_p2[1]}),
                             std::min({m_p0[2], m_p1[2], m_p2[2]})},
                    Vector3f{std::max({m_p0[0], m_p1[0], m_p2[0]}),
                             std::max({m_p0[1], m_p1[1], m_p2[1]}),
                             std::max({m_p0[2], m_p1[2], m_p2[2]})});
    }

    Float Triangle::get_area() const{
        return Vector3f::cross(m_p1 - m_p0, m_p2 - m_p0).length() * Float0_5;
    }

    std::tuple<Vector3f, Vector3f, Float> Triangle::sample_point(Sampler &sampler) const{
        const Float u = sampler.sample_1d();
        const Float v = sampler.sample_1d();
        using std::sqrt;
        const Float x = Float1 - sqrt(Float1 - u);
        const Float y = v * sqrt(Float1 - u);
        // z = 1 - x - y

        return {interpolate(m_p0, m_p1, m_p2, x, y),
                is_vn_exists ?
                             interpolate(normal(0), normal(1), normal(2), x, y).normalize() :
                             Vector3f::cross(m_p1 - m_p0, m_p2 - m_p0).normalize(),
                Float1 / get_area()};
    }

    // Similar with `OBJMesh::pdf_solidangle()`
    // This implementation assumes that two given points are visible to each other
    Float Triangle::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const{
        const Vector3f shape_to_hitpos_world = hitpos_world - shapepos_world;
        const Float dist_squared = shape_to_hitpos_world.dot(shape_to_hitpos_world);
        using std::abs;
        const Float cos = abs(shape_normal_world.dot(shape_to_hitpos_world.normalize()));
        return dist_squared / (cos * get_area());
    }

    // u, v, t
    std::pair<bool, RayIntersectInfo> Triangle::ray_intersect(const Ray &ray, Float maxt) const {

#ifdef USE_MOLLER_TRUMBORE
        auto [u, v, t] = moller_trumbore(ray, m_p0, m_p1, m_p2, maxt);
#else
        // Default: watertight intersection (more robust at triangle edges)
        // Reference: https://jcgt.org/published/0002/01/05/paper.pdf
        auto [u, v, t] = watertight_intersection(ray, m_p0, m_p1, m_p2, maxt);
#endif

        if(u==-Float1 && v==-Float1 && t==-Float1){
            return {false, RayIntersectInfo()};
        }

        RayIntersectInfo ret;
        ret.t = t;
        ret.tex_uv = is_tx_exists ? interpolate(m_uv0, m_uv1, m_uv2, u, v) : Vector2f{u, v};

        ret.tex_uv[0] -= floor(ret.tex_uv[0]);
        ret.tex_uv[1] -= floor(ret.tex_uv[1]);

        ret.p = interpolate(m_p0, m_p1, m_p2, u, v);
        
        const Vector3f n = is_vn_exists ? interpolate(m_n0, m_n1, m_n2, u, v).normalize() :
                                          Vector3f::cross(m_p1 - m_p0, m_p2 - m_p0).normalize();
        ret.sh_coord = Coordinate(n);

        return {true, ret};
    }
}