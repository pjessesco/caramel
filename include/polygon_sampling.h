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

// Solid angle sampling of convex polygons.
// Based on: Christoph Peters, 2021
//   "BRDF Importance Sampling for Polygonal Lights", Section 5 & Supplement C
//   https://doi.org/10.1145/3450626.3459672
//
// Given a convex polygon light and a shading point, this samples a direction
// uniformly distributed over the solid angle subtended by the polygon.
// The PDF is simply 1 / solid_angle (uniform in solid angle measure).
//
// The sampled result is a unit direction vector from the shading point toward
// the polygon. To recover the actual 3D point on the light surface, a ray-plane
// intersection is needed:
//
//   plane_n = cross(p1 - p0, p2 - p0)        // unnormalized triangle normal
//   t = dot(plane_n, p0 - origin) / dot(plane_n, dir)
//   light_pos = origin + t * dir
//
// This is necessary because the algorithm operates entirely on the unit sphere
// (spherical polygon), so it only produces directions, not positions.

#pragma once

#include <array>
#include <cmath>

#include <common.h>

namespace Caramel{

    constexpr Index MAX_POLYGON_VERTEX_COUNT = 8;

    // Precomputed data for a convex polygon projected onto the unit sphere.
    // Used by sample_solid_angle_polygon() to sample directions.
    struct SolidAnglePolygon {
        // Number of vertices of the polygon
        Index vertex_count;

        // Normalized direction vectors from the shading point to each vertex.
        // These are the polygon vertices projected onto the unit sphere.
        std::array<Vector3f, MAX_POLYGON_VERTEX_COUNT> vertex_dirs;

        // Precomputed parameters for each triangle in the fan triangulation.
        // For the i-th triangle (vertex_dirs[i+1], vertex_dirs[0], vertex_dirs[i+2]):
        //   [0] = |det(v0, v1, v2)| computed via Householder reflection (simplex volume)
        //   [1] = dot(v0, v2) + dot(v1, v2)
        //   [2] = 1 + dot(v0, v1)
        // These are used in the Van Oosterom-Strackee formula and sampling.
        std::array<Vector3f, MAX_POLYGON_VERTEX_COUNT - 2> triangle_parameters;

        // Cumulative solid angles of the fan triangles.
        // fan_solid_angles[i] = sum of solid angles of triangles 0..i
        std::array<Float, MAX_POLYGON_VERTEX_COUNT - 2> fan_solid_angles;

        // Total solid angle subtended by the polygon as seen from the shading point
        Float solid_angle;
    };

    // Compute atan(tangent) mapped to [0, pi).
    // Standard atan returns [-pi/2, pi/2], so we add pi when tangent < 0
    // to get the correct angle for the Van Oosterom-Strackee formula.
    inline Float positive_atan(Float tangent) {
        using std::atan;
        return atan(tangent) + ((tangent < Float0) ? PI : Float0);
    }

    // Numerically stable linear interpolation: x*(1-a) + y*a
    // Uses two fma (fused multiply-add) operations to minimize rounding error:
    //   fma(-a, x, x) = x - a*x = x*(1-a)
    //   fma(a, y, ...)  adds a*y
    inline Float mix_fma(Float x, Float y, Float a) {
        using std::fma;
        return fma(a, y, fma(-a, x, x));
    }

    // Precompute the solid angle polygon data for a convex polygon.
    //
    // The polygon is decomposed into a fan of spherical triangles sharing vertex_dirs[0].
    // For each triangle, we compute:
    //   - The solid angle via the Van Oosterom-Strackee formula:
    //       tan(Omega/2) = |det(v0,v1,v2)| / (1 + d01 + d02 + d12)
    //     where dij = dot(vi, vj) and vi are unit vectors from the shading point.
    //   - The |det| is computed using a Householder reflection that maps vertex_dirs[0]
    //     onto the x-axis, reducing the 3x3 determinant to a 2x2 determinant for
    //     better numerical stability (avoids catastrophic cancellation).
    //
    // Parameters:
    //   vertex_count: number of polygon vertices (3 to MAX_POLYGON_VERTEX_COUNT)
    //   vertices:     world-space vertex positions of the convex polygon
    //   shading_pos:  world-space position of the shading point
    inline SolidAnglePolygon prepare_solid_angle_polygon(Index vertex_count, const Vector3f* vertices, const Vector3f& shading_pos) {
        using std::abs;
        using std::fma;

        SolidAnglePolygon polygon;
        polygon.vertex_count = vertex_count;

        // Project each vertex onto the unit sphere centered at the shading point
        for (Index i = 0; i < vertex_count; ++i) {
            const Vector3f diff = vertices[i] - shading_pos;
            polygon.vertex_dirs[i] = diff.normalize();
        }

        // Householder reflection setup: map vertex_dirs[0] onto (+-1, 0, 0).
        // The reflection vector h = vertex_dirs[0] - sign * e_x, and we precompute
        // only the yz-components with the 2/||h||^2 scaling baked in.
        // This avoids explicitly forming the full Householder matrix.
        const Float householder_sign = (polygon.vertex_dirs[0][0] > Float0) ? -Float1 : Float1;
        const Float hh_denom = Float1 / (abs(polygon.vertex_dirs[0][0]) + Float1);
        const Vector2f householder_yz{polygon.vertex_dirs[0][1] * hh_denom,
                                      polygon.vertex_dirs[0][2] * hh_denom};

        polygon.solid_angle = Float0;

        // Cache dot product between vertex_dirs[0] and vertex_dirs[1] for reuse
        Float previous_dot_1_2 = polygon.vertex_dirs[0].dot(polygon.vertex_dirs[1]);

        // Iterate over fan triangles: (vertex_dirs[i+1], vertex_dirs[0], vertex_dirs[i+2])
        for (Index i = 0; i + 2 < vertex_count; ++i) {
            const Vector3f& v0 = polygon.vertex_dirs[i + 1];  // current edge start
            const Vector3f& v1 = polygon.vertex_dirs[0];       // fan center (shared vertex)
            const Vector3f& v2 = polygon.vertex_dirs[i + 2];   // current edge end

            // Dot products between all pairs of the triangle vertices
            const Float dot_01 = previous_dot_1_2;   // reuse from previous iteration
            const Float dot_02 = v0.dot(v2);
            const Float dot_12 = v1.dot(v2);
            previous_dot_1_2 = dot_12;  // cache for next iteration

            // Apply Householder reflection to compute |det(v0, v1, v2)| stably.
            // After reflection, v1 maps to (+-1, 0, 0), so the 3x3 determinant
            // reduces to a 2x2 determinant of the yz-components of Hv0 and Hv2.
            //
            // dot_hh_0 = dot(H*v0, H*v1) projected term = dot(v0, v1) - sign * v0.x
            const Float dot_hh_0 = fma(-householder_sign, v0[0], dot_01);
            const Float dot_hh_2 = fma(-householder_sign, v2[0], dot_12);

            // yz-components of Householder-transformed v0 and v2:
            //   (Hv)_yz = v_yz - dot_hh * householder_yz
            const Float col0_y = fma(-dot_hh_0, householder_yz[0], v0[1]);
            const Float col0_z = fma(-dot_hh_0, householder_yz[1], v0[2]);
            const Float col1_y = fma(-dot_hh_2, householder_yz[0], v2[1]);
            const Float col1_z = fma(-dot_hh_2, householder_yz[1], v2[2]);

            // 2x2 determinant = |det(v0, v1, v2)| = simplex volume
            const Float simplex_volume = abs(col0_y * col1_z - col0_z * col1_y);

            // Van Oosterom-Strackee formula for the solid angle of a spherical triangle:
            //   tan(Omega/2) = |det(v0, v1, v2)| / (1 + d01 + d02 + d12)
            //   Omega = 2 * atan(tan(Omega/2))
            // We store intermediate terms for reuse during sampling.
            const Float dot_02_plus_12 = dot_02 + dot_12;
            const Float one_plus_dot_01 = Float1 + dot_01;
            const Float tangent = simplex_volume / (one_plus_dot_01 + dot_02_plus_12);
            const Float triangle_solid_angle = Float2 * positive_atan(tangent);

            // Accumulate the cumulative solid angle for triangle selection during sampling
            polygon.solid_angle += triangle_solid_angle;
            polygon.fan_solid_angles[i] = polygon.solid_angle;

            // Store (simplex_volume, dot_02+dot_12, 1+dot_01) for use in sampling
            polygon.triangle_parameters[i] = Vector3f{simplex_volume, dot_02_plus_12, one_plus_dot_01};
        }

        return polygon;
    }

    // Sample a unit direction uniformly distributed over the solid angle of the polygon.
    //
    // Algorithm (Peters, Section 5):
    //   1. Use u0 to select a fan triangle proportional to its solid angle,
    //      and compute the residual solid angle within that triangle.
    //   2. Construct a new vertex L2' on the great arc (v0, v2) such that the
    //      sub-triangle (v0, v1, L2') has the desired sub-solid-angle.
    //      This uses the shortcut formula (Supplement C.3):
    //        offset = (P0*cos(h) - P1*sin(h)) * v0 + P2*sin(h) * v2
    //        L2' = -v0 + 2 * dot(v0, offset) / dot(offset, offset) * offset
    //      which is a reflection of v0 through the plane defined by offset.
    //   3. Use u1 to linearly interpolate along the great arc (v1, L2') via slerp.
    //
    // Parameters:
    //   polygon: precomputed polygon data from prepare_solid_angle_polygon()
    //   u0, u1:  uniform random numbers in [0, 1)
    //
    // Returns:
    //   A unit direction vector from the shading point toward the sampled point
    //   on the polygon. This direction lies within the solid angle of the polygon.
    inline Vector3f sample_solid_angle_polygon(const SolidAnglePolygon& polygon, Float u0, Float u1) {
        using std::cos;
        using std::sin;
        using std::sqrt;
        using std::fma;

        // Step 1: Select which fan triangle to sample from.
        // Scale u0 by total solid angle to get a target cumulative solid angle.
        const Float target_solid_angle = polygon.solid_angle * u0;

        // Start with the first triangle's data as default
        Float subtriangle_solid_angle = target_solid_angle;
        Vector3f parameters = polygon.triangle_parameters[0];
        Vector3f v0 = polygon.vertex_dirs[1];   // fan triangle edge start
        Vector3f v1 = polygon.vertex_dirs[0];   // fan center (shared vertex)
        Vector3f v2 = polygon.vertex_dirs[2];   // fan triangle edge end

        // Walk the cumulative solid angle array to find the right triangle
        for (Index i = 0; i + 3 < polygon.vertex_count; ++i) {
            if (polygon.fan_solid_angles[i] >= target_solid_angle) break;

            // Subtract the cumulative angle to get the residual within the next triangle
            subtriangle_solid_angle = target_solid_angle - polygon.fan_solid_angles[i];
            v0 = polygon.vertex_dirs[i + 2];
            v2 = polygon.vertex_dirs[i + 3];
            parameters = polygon.triangle_parameters[i + 1];
        }

        // Step 2: Construct new vertex L2' on the arc v0-v2.
        // half_angle = half of the residual solid angle we want the sub-triangle to have
        const Float half_angle = Float0_5 * subtriangle_solid_angle;
        const Float cos_half = cos(half_angle);
        const Float sin_half = sin(half_angle);

        // Peters' shortcut formula (Supplement C.3):
        //   offset = (P0*cos - P1*sin) * v0 + P2*sin * v2
        // where P0 = simplex_volume, P1 = dot_02+dot_12, P2 = 1+dot_01
        const Float coeff_v0 = parameters[0] * cos_half - parameters[1] * sin_half;
        const Float coeff_v2 = parameters[2] * sin_half;
        const Vector3f offset = v0 * coeff_v0 + v2 * coeff_v2;

        // Reflect v0 through the plane containing the origin with normal perpendicular to offset:
        //   L2' = -v0 + 2 * dot(v0, offset) / dot(offset, offset) * offset
        // This places L2' on the great arc from v0 to v2 at the correct solid angle.
        const Float scale = Float2 * v0.dot(offset) / offset.dot(offset);
        const Vector3f new_vertex_2 = offset * scale - v0;

        // Step 3: Sample along the great arc from v1 to new_vertex_2 using u1.
        // This is an efficient slerp parameterization.
        //   s2 = cos(angle between v1 and L2')
        //   s  = lerp(1, s2, u1) = cos of the interpolated angle
        //   t_normed = sin of the interpolated angle / sin of the full angle
        const Float s2 = v1.dot(new_vertex_2);
        const Float s = mix_fma(Float1, s2, u1);
        const Float denominator = fma(-s2, s2, Float1);   // = 1 - s2^2 = sin^2(angle)
        Float t_normed = sqrt(fma(-s, s, Float1) / denominator);

        // Fallback for degenerate case: when v1 and new_vertex_2 are nearly identical
        // (s2 -> 1), denominator -> 0, so we fall back to linear interpolation
        t_normed = (denominator > Float0) ? t_normed : u1;

        // Final sampled direction = slerp(v1, new_vertex_2, u1)
        //   = v1 * (s - t_normed * s2) + new_vertex_2 * t_normed
        return v1 * fma(-t_normed, s2, s) + new_vertex_2 * t_normed;
    }

}
