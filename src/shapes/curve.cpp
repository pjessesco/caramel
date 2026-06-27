//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2026 Jino Park
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

#include <algorithm>
#include <cmath>

#include <shape.h>

#include <ray.h>
#include <rayintersectinfo.h>
#include <coordinate.h>
#include <aabb.h>
#include <bvh_base.h>

namespace Caramel {
    namespace {
        // pbrt convention: lerp(t, a, b) = (1-t)a + t b  (t is the FIRST argument).
        template <typename T>
        T lerp(Float t, const T &a, const T &b) { return a * (Float1 - t) + b * t; }

        // Blossom f(u0,u1,u2): de Casteljau with a different parameter per level.
        Vector3f blossom(const std::array<Vector3f, 4> &p, Float u0, Float u1, Float u2) {
            const Vector3f a0 = lerp(u0, p[0], p[1]);
            const Vector3f a1 = lerp(u0, p[1], p[2]);
            const Vector3f a2 = lerp(u0, p[2], p[3]);
            const Vector3f b0 = lerp(u1, a0, a1);
            const Vector3f b1 = lerp(u1, a1, a2);
            return lerp(u2, b0, b1);
        }

        // 4 control points of the curve restricted to [u_min, u_max].
        std::array<Vector3f, 4> sub_control_points(const std::array<Vector3f, 4> &cp, Float u_min, Float u_max) {
            return {blossom(cp, u_min, u_min, u_min), blossom(cp, u_min, u_min, u_max),
                    blossom(cp, u_min, u_max, u_max), blossom(cp, u_max, u_max, u_max)};
        }

        // Point + tangent. Zero-tangent falls back to the chord so dpdu is never the zero vector.
        Vector3f eval_bezier(const std::array<Vector3f, 4> &cp, Float u, Vector3f *deriv) {
            const Vector3f a0 = lerp(u, cp[0], cp[1]);
            const Vector3f a1 = lerp(u, cp[1], cp[2]);
            const Vector3f a2 = lerp(u, cp[2], cp[3]);
            const Vector3f b0 = lerp(u, a0, a1);
            const Vector3f b1 = lerp(u, a1, a2);
            if (deriv) {
                const Vector3f d = b1 - b0;
                *deriv = d.length() > Float0 ? Vector3f(d * static_cast<Float>(3)) : Vector3f(cp[3] - cp[0]);
            }
            return lerp(u, b0, b1);
        }

        // Split a cubic into two halves sharing the midpoint (de Casteljau at u=0.5).
        std::array<Vector3f, 7> subdivide_bezier(const std::array<Vector3f, 4> &c) {
            return {c[0],
                    (c[0] + c[1]) * Float0_5,
                    (c[0] + c[1] * Float2 + c[2]) * static_cast<Float>(0.25),
                    (c[0] + c[1] * static_cast<Float>(3) + c[2] * static_cast<Float>(3) + c[3]) * static_cast<Float>(0.125),
                    (c[1] + c[2] * Float2 + c[3]) * static_cast<Float>(0.25),
                    (c[2] + c[3]) * Float0_5,
                    c[3]};
        }

        // round-to-nearest integer log2 (mirrors pbrt Log2Int(float)); guard L0 > 0 first.
        Float int_log2_round(Float v) {
            if (v <= Float0) return Float0;
            const int lo = std::ilogb(v);
            return static_cast<Float>(v >= static_cast<Float>(1.5) * std::ldexp(Float1, lo) ? lo + 1 : lo);
        }

        AABB box_of(const std::array<Vector3f, 4> &cp) {
            return AABB::merge(AABB{cp[0], cp[1]}, AABB{cp[2], cp[3]});
        }

        // World AABB of one cubic-Bezier curve restricted to [u_min, u_max], padded by half-width.
        AABB slice_box(const CurveCommon &cc, Float u_min, Float u_max) {
            const auto cp = sub_control_points(cc.m_cp, u_min, u_max);
            const Float w0 = lerp(u_min, cc.m_width[0], cc.m_width[1]);
            const Float w1 = lerp(u_max, cc.m_width[0], cc.m_width[1]);
            const Float r = std::max(w0, w1) * Float0_5;
            const AABB b = box_of(cp);
            return AABB{b.m_min - Vector3f{r, r, r}, b.m_max + Vector3f{r, r, r}};
        }

        // ---- one curve slice [u0,u1] vs ray, recursive de Casteljau subdivision ----
        bool curve_recurse(const CurveCommon &cc, const Ray &ray, Float maxt,
                           const std::array<Vector3f, 4> &cp, const Coordinate &ray_frame,
                           Float u0, Float u1, int depth, RayIntersectInfo &best) {
            if (depth > 0) {
                const auto s = subdivide_bezier(cp);
                const Float um = (u0 + u1) * Float0_5;
                const Float ub[3] = {u0, um, u1};
                bool any = false;
                for (int seg = 0; seg < 2; ++seg) {
                    const std::array<Vector3f, 4> child{s[3 * seg], s[3 * seg + 1], s[3 * seg + 2], s[3 * seg + 3]};
                    const Float cw0 = lerp(ub[seg], cc.m_width[0], cc.m_width[1]);
                    const Float cw1 = lerp(ub[seg + 1], cc.m_width[0], cc.m_width[1]);
                    const Float r = std::max(cw0, cw1) * Float0_5;
                    AABB cb = box_of(child);
                    cb = AABB{cb.m_min - Vector3f{r, r, r}, cb.m_max + Vector3f{r, r, r}};
                    const AABB ray_box{Vector3f{Float0, Float0, Float0}, Vector3f{Float0, Float0, best.t}};
                    if (!cb.is_overlap(ray_box)) continue;
                    any = curve_recurse(cc, ray, maxt, child, ray_frame, ub[seg], ub[seg + 1], depth - 1, best) || any;
                }
                return any;
            }

            // leaf: 2D test in the ray-space (x,y) plane + a z-range check.
            if ((cp[1][1] - cp[0][1]) * (-cp[0][1]) + cp[0][0] * (cp[0][0] - cp[1][0]) < Float0) return false;
            if ((cp[2][1] - cp[3][1]) * (-cp[3][1]) + cp[3][0] * (cp[3][0] - cp[2][0]) < Float0) return false;

            const Float sx = cp[3][0] - cp[0][0], sy = cp[3][1] - cp[0][1];
            const Float denom = sx * sx + sy * sy;
            if (denom == Float0) return false;
            const Float w = ((-cp[0][0]) * sx + (-cp[0][1]) * sy) / denom;

            const Float u = std::clamp(lerp(w, u0, u1), u0, u1);
            Float hit_width = lerp(u, cc.m_width[0], cc.m_width[1]);

            Vector3f n_hit;
            if (cc.m_type == CurveType::Ribbon) {
                using std::sin;
                if (cc.m_normal_angle == Float0) n_hit = cc.m_n[0];
                else {
                    const Float A = cc.m_normal_angle, is = cc.m_inv_sin_normal_angle;
                    n_hit = cc.m_n[0] * (sin((Float1 - u) * A) * is) + cc.m_n[1] * (sin(u * A) * is);
                }
                hit_width *= std::abs(n_hit.dot(ray.m_d));
            }

            Vector3f dpcdw;
            const Vector3f pc = eval_bezier(cp, std::clamp(w, Float0, Float1), &dpcdw);
            const Float dist2 = pc[0] * pc[0] + pc[1] * pc[1];
            if (dist2 > hit_width * hit_width * static_cast<Float>(0.25)) return false;
            if (pc[2] < Float0 || pc[2] > maxt) return false;

            const Float t_hit = pc[2];
            if (t_hit > best.t) return false;

            const Float dist = std::sqrt(dist2);
            const Float edge = dpcdw[0] * (-pc[1]) + pc[0] * dpcdw[1];
            const Float v = edge > Float0 ? Float0_5 + dist / hit_width : Float0_5 - dist / hit_width;

            Vector3f dpdu;
            eval_bezier(cc.m_cp, u, &dpdu);

            Vector3f dpdv;
            if (cc.m_type == CurveType::Ribbon) {
                dpdv = Vector3f::cross(n_hit, dpdu).normalize() * hit_width;
            } else {
                const Vector3f dpdu_plane = ray_frame.to_local(dpdu);
                Vector3f dpdv_plane = Vector3f{-dpdu_plane[1], dpdu_plane[0], Float0}.normalize() * hit_width;
                if (cc.m_type == CurveType::Cylinder) {
                    const Float theta = deg_to_rad(lerp(v, static_cast<Float>(-90), static_cast<Float>(90)));
                    const Vector3f axis = dpdu_plane.normalize();
                    using std::cos;
                    using std::sin;
                    const Float c = cos(-theta), si = sin(-theta);
                    dpdv_plane = dpdv_plane * c + Vector3f::cross(axis, dpdv_plane) * si + axis * (axis.dot(dpdv_plane) * (Float1 - c));
                }
                dpdv = ray_frame.to_world(dpdv_plane);
            }

            Vector3f n = (cc.m_type == CurveType::Flat) ? Vector3f(ray.m_d * static_cast<Float>(-1))
                                                        : Vector3f(Vector3f::cross(dpdu, dpdv).normalize());
            if (n.dot(ray.m_d) > Float0) n = n * static_cast<Float>(-1);

            best.t = t_hit;
            best.p = ray.m_o + ray.m_d * t_hit;
            best.tex_uv = Vector2f{u, v};
            best.sh_coord = Coordinate{n};
            return true;
        }

        std::pair<bool, RayIntersectInfo> curve_slice_intersect(const CurveCommon &cc, Float u_min, Float u_max,
                                                                const Ray &ray, Float maxt) {
            const auto cp_obj = sub_control_points(cc.m_cp, u_min, u_max);

            Vector3f dx = Vector3f::cross(ray.m_d, cp_obj[3] - cp_obj[0]);
            if (dx.dot(dx) == Float0) dx = Coordinate{ray.m_d}.m_axis1;
            Coordinate ray_frame;
            ray_frame.m_world_n = ray.m_d;
            ray_frame.m_axis1 = Vector3f::cross(dx, ray.m_d).normalize();
            ray_frame.m_axis2 = Vector3f::cross(ray.m_d, ray_frame.m_axis1);

            std::array<Vector3f, 4> cp;
            for (int i = 0; i < 4; ++i) cp[i] = ray_frame.to_local(cp_obj[i] - ray.m_o);

            const Float w0 = lerp(u_min, cc.m_width[0], cc.m_width[1]);
            const Float w1 = lerp(u_max, cc.m_width[0], cc.m_width[1]);
            const Float r = std::max(w0, w1) * Float0_5;
            AABB cb = box_of(cp);
            cb = AABB{cb.m_min - Vector3f{r, r, r}, cb.m_max + Vector3f{r, r, r}};
            const AABB ray_box{Vector3f{Float0, Float0, Float0}, Vector3f{Float0, Float0, maxt}};
            if (!cb.is_overlap(ray_box)) return {false, RayIntersectInfo()};

            Float L0 = Float0;
            for (int i = 0; i < 2; ++i)
                for (int a = 0; a < 3; ++a)
                    L0 = std::max(L0, std::abs(cp[i][a] - Float2 * cp[i + 1][a] + cp[i + 2][a]));
            int max_depth = 0;
            if (L0 > Float0) {
                const Float eps = std::max(w0, w1) * static_cast<Float>(0.05);
                const Float d = int_log2_round(static_cast<Float>(1.41421356237) * static_cast<Float>(6) * L0 /
                                               (static_cast<Float>(8) * eps)) * Float0_5;
                max_depth = std::clamp(static_cast<int>(d), 0, 10);
            }

            RayIntersectInfo best;
            best.t = maxt;
            const bool hit = curve_recurse(cc, ray, maxt, cp, ray_frame, u_min, u_max, max_depth, best);
            return hit ? std::pair{true, best} : std::pair{false, RayIntersectInfo()};
        }

        // ---- Level A: basis -> cubic Bezier converters (parse time) ----
        std::array<Vector3f, 4> elevate_quad_to_cubic(const Vector3f &q0, const Vector3f &q1, const Vector3f &q2) {
            return {q0, lerp(static_cast<Float>(2.0 / 3.0), q0, q1), lerp(static_cast<Float>(1.0 / 3.0), q1, q2), q2};
        }
        std::array<Vector3f, 3> quad_bspline_to_bezier(const Vector3f &p01, const Vector3f &p12, const Vector3f &p23) {
            return {lerp(Float0_5, p01, p12), p12, lerp(Float0_5, p12, p23)};
        }
        std::array<Vector3f, 4> cubic_bspline_to_bezier(const Vector3f &p012, const Vector3f &p123, const Vector3f &p234, const Vector3f &p345) {
            const Vector3f p122 = lerp(static_cast<Float>(2.0 / 3.0), p012, p123);
            const Vector3f p223 = lerp(static_cast<Float>(1.0 / 3.0), p123, p234);
            const Vector3f p233 = lerp(static_cast<Float>(2.0 / 3.0), p123, p234);
            const Vector3f p334 = lerp(static_cast<Float>(1.0 / 3.0), p234, p345);
            return {lerp(Float0_5, p122, p223), p223, p233, lerp(Float0_5, p233, p334)};
        }

        // Build per-segment CurveCommons for one logical curve (basis -> cubic Bezier).
        std::vector<CurveCommon> curve_to_commons(const std::vector<Vector3f> &P, int degree, bool bspline,
                                                  CurveType type, Float width0, Float width1,
                                                  const std::vector<Vector3f> &normals) {
            std::vector<CurveCommon> out;
            const int nP = static_cast<int>(P.size());
            const int nSeg = bspline ? (nP - degree) : ((nP - 1) / degree);
            if (nSeg < 1) return out;
            const Float fnSeg = static_cast<Float>(nSeg);
            out.reserve(nSeg);
            for (int seg = 0; seg < nSeg; ++seg) {
                const int off = bspline ? seg : seg * degree;
                std::array<Vector3f, 4> cp;
                if (degree == 3 && !bspline) cp = {P[off], P[off + 1], P[off + 2], P[off + 3]};
                else if (degree == 3 && bspline) cp = cubic_bspline_to_bezier(P[off], P[off + 1], P[off + 2], P[off + 3]);
                else if (degree == 2 && !bspline) cp = elevate_quad_to_cubic(P[off], P[off + 1], P[off + 2]);
                else { const auto q = quad_bspline_to_bezier(P[off], P[off + 1], P[off + 2]); cp = elevate_quad_to_cubic(q[0], q[1], q[2]); }

                const Float sw0 = lerp(static_cast<Float>(seg) / fnSeg, width0, width1);
                const Float sw1 = lerp(static_cast<Float>(seg + 1) / fnSeg, width0, width1);
                const Vector3f *np = nullptr;
                Vector3f nn[2];
                if (type == CurveType::Ribbon && static_cast<int>(normals.size()) >= nSeg + 1) {
                    nn[0] = normals[seg];
                    nn[1] = normals[seg + 1];
                    np = nn;
                }
                out.emplace_back(cp, sw0, sw1, type, np);
            }
            return out;
        }
    } // namespace

    CurveCommon::CurveCommon(const std::array<Vector3f, 4> &cp, Float w0, Float w1, CurveType type, const Vector3f *n)
        : m_type{type}, m_cp{cp}, m_width{w0, w1} {
        if (type == CurveType::Ribbon && n) {
            m_n[0] = n[0].normalize();
            m_n[1] = n[1].normalize();
            using std::acos;
            using std::sin;
            m_normal_angle = acos(std::clamp(m_n[0].dot(m_n[1]), -Float1, Float1));
            m_inv_sin_normal_angle = Float1 / sin(m_normal_angle);
        }
    }

    // ===================== Curve (single slice, top-level Shape) =====================
    Curve::Curve(std::shared_ptr<const CurveCommon> common, Float u_min, Float u_max, BSDF *bsdf)
        : Shape{bsdf, nullptr}, m_common{std::move(common)}, m_u_min{u_min}, m_u_max{u_max} {}

    const std::vector<Vector3f> &Curve::get_polygon_vertices() const {
        static const std::vector<Vector3f> empty;
        return empty;
    }
    AABB Curve::get_aabb() const { return slice_box(*m_common, m_u_min, m_u_max); }
    Float Curve::get_area() const {
        const auto cp = sub_control_points(m_common->m_cp, m_u_min, m_u_max);
        const Float len = Vector3f(cp[1] - cp[0]).length() + Vector3f(cp[2] - cp[1]).length() + Vector3f(cp[3] - cp[2]).length();
        return len * (m_common->m_width[0] + m_common->m_width[1]) * Float0_5;
    }
    std::tuple<Vector3f, Vector3f, Float> Curve::sample_point(Sampler &) const {
        return {vec3f_zero, Vector3f{Float0, Float0, Float1}, Float0};
    }
    Float Curve::pdf_solidangle(const Vector3f &, const Vector3f &, const Vector3f &) const { return Float0; }
    std::pair<bool, RayIntersectInfo> Curve::ray_intersect(const Ray &ray, Float maxt) const {
        return curve_slice_intersect(*m_common, m_u_min, m_u_max, ray, maxt);
    }

    // ===================== CurveMesh (many curves, own inner BVH) =====================
    Index CurveMesh::slice_num() const { return m_slice_aabb.size(); }
    AABB CurveMesh::slice_aabb(Index i) const { return m_slice_aabb[i]; }
    Vector3f CurveMesh::slice_center(Index i) const { return m_slice_center[i]; }
    std::pair<bool, RayIntersectInfo> CurveMesh::slice_intersect(Index i, const Ray &ray, Float maxt) const {
        const Slice &s = m_slices[i];
        return curve_slice_intersect(m_commons[s.common_idx], s.u_min, s.u_max, ray, maxt);
    }

    CurveMesh::CurveMesh(std::vector<CurveCommon> commons, int split_depth, BSDF *bsdf)
        : Shape{bsdf, nullptr}, m_commons{std::move(commons)} {
        const int nslice = 1 << split_depth;
        const std::size_t total = m_commons.size() * static_cast<std::size_t>(nslice);
        m_slices.reserve(total);
        m_slice_aabb.reserve(total);
        m_slice_center.reserve(total);
        Vector3f mn{INF, INF, INF}, mx{-INF, -INF, -INF};
        for (std::size_t c = 0; c < m_commons.size(); ++c) {
            for (int i = 0; i < nslice; ++i) {
                const Float umin = static_cast<Float>(i) / nslice;
                const Float umax = static_cast<Float>(i + 1) / nslice;
                const AABB box = slice_box(m_commons[c], umin, umax);
                m_slices.push_back({static_cast<Index>(c), umin, umax});
                m_slice_aabb.push_back(box);
                m_slice_center.push_back((box.m_min + box.m_max) * Float0_5);
                for (int a = 0; a < 3; ++a) { mn[a] = std::min(mn[a], box.m_min[a]); mx[a] = std::max(mx[a], box.m_max[a]); }
            }
        }
        m_aabb = AABB{mn, mx};

        std::vector<Index> prims(m_slices.size());
        for (std::size_t i = 0; i < prims.size(); ++i) prims[i] = static_cast<Index>(i);
        m_bvh = std::make_unique<BVHTree<CurveMeshTraits>>(std::move(prims), CurveMeshTraits{*this},
                                                           Float1, Float1, 16, 4);
    }

    CurveMesh::~CurveMesh() = default;

    const std::vector<Vector3f> &CurveMesh::get_polygon_vertices() const {
        static const std::vector<Vector3f> empty;
        return empty;
    }
    AABB CurveMesh::get_aabb() const { return m_aabb; }
    Float CurveMesh::get_area() const { return Float0; }  // curves are never area lights
    std::tuple<Vector3f, Vector3f, Float> CurveMesh::sample_point(Sampler &) const {
        return {vec3f_zero, Vector3f{Float0, Float0, Float1}, Float0};
    }
    Float CurveMesh::pdf_solidangle(const Vector3f &, const Vector3f &, const Vector3f &) const { return Float0; }
    std::pair<bool, RayIntersectInfo> CurveMesh::ray_intersect(const Ray &ray, Float maxt) const {
        return m_bvh->ray_intersect(ray, maxt);
    }

    // ===================== factories =====================
    void create_curve(std::vector<Vector3f> P, int degree, bool bspline, CurveType type,
                      Float width0, Float width1, const std::vector<Vector3f> &normals,
                      int split_depth, BSDF *bsdf, std::vector<Shape *> &out) {
        auto commons = curve_to_commons(P, degree, bspline, type, width0, width1, normals);
        const int nslice = 1 << split_depth;
        for (auto &cc : commons) {
            auto shared = std::make_shared<const CurveCommon>(cc);
            for (int i = 0; i < nslice; ++i) {
                const Float umin = static_cast<Float>(i) / nslice;
                const Float umax = static_cast<Float>(i + 1) / nslice;
                out.push_back(Shape::Create<Curve>(shared, umin, umax, bsdf));
            }
        }
    }
}
