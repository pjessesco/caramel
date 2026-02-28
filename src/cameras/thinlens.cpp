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

#include <camera.h>

#include <common.h>
#include <transform.h>
#include <ray.h>
#include <warp_sample.h>

namespace Caramel{

    ThinLens::ThinLens(const Vector3f &pos, const Vector3f &dir, const Vector3f &up,
                       Index w, Index h, Float fov_x, Float lens_radius, Float focal_dist)
        : Camera(pos, dir, up, w, h, fov_x), m_lens_radius{lens_radius}, m_focal_dist{focal_dist} {}

    ThinLens::ThinLens(const Matrix44f &cam_to_world, Index w, Index h, Float fov_x, Float lens_radius, Float focal_dist)
        : Camera(cam_to_world, w, h, fov_x), m_lens_radius{lens_radius}, m_focal_dist{focal_dist} {}

    [[nodiscard]] Ray ThinLens::sample_ray(Float w, Float h, Sampler &sampler) const{
        // 1. Calculate ray direction for a pinhole camera
        // Pinhole ray direction in camera space (origin is 0,0,0)
        Vector3f local_d_pinhole = Vector3f(Block<0,0,3,1>(m_sample_to_camera * Vector4f(w / static_cast<Float>(m_w),
                                                                                         h / static_cast<Float>(m_h),
                                                                                         Float0, Float1))).normalize();

        // If lens radius is effectively zero, behave like a pinhole
        if (m_lens_radius <= Float0) {
             const Vector3f d = Block<0,0,3,1>(m_cam_to_world * Vector4f{local_d_pinhole[0], local_d_pinhole[1], local_d_pinhole[2], Float0});
             return {m_pos, d.normalize()};
        }

        // 2. Sample point on the lens
        auto [lens_sample_uv, _] = sample_unit_disk_uniformly(sampler);
        Vector3f p_lens_cam{lens_sample_uv[0] * m_lens_radius, lens_sample_uv[1] * m_lens_radius, Float0};

        // 3. Calculate point on the focal plane
        Float ft = m_focal_dist / local_d_pinhole[2];
        Vector3f p_focus_cam = local_d_pinhole * ft;

        // 4. Calculate new ray direction (from p_lens to p_focus)
        Vector3f new_ray_dir_cam = Vector3f(p_focus_cam - p_lens_cam).normalize();

        // 5. Transform to world space
        Vector3f origin_world = Block<0,0,3,1>(m_cam_to_world * Vector4f{p_lens_cam[0], p_lens_cam[1], p_lens_cam[2], Float1});
        Vector3f dir_world = Block<0,0,3,1>(m_cam_to_world * Vector4f{new_ray_dir_cam[0], new_ray_dir_cam[1], new_ray_dir_cam[2], Float0});

        return {origin_world, dir_world.normalize()};
    }

}
