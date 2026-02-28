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

namespace Caramel{

    // Perspective camera
    Pinhole::Pinhole(const Matrix44f &cam_to_world, Index w, Index h, Float fov_x)
        : Camera(cam_to_world, w, h, fov_x) {}

    Pinhole::Pinhole(const Vector3f &pos, const Vector3f &dir, const Vector3f &up, Index w, Index h, Float fov_x)
        : Camera(pos, dir, up, w, h, fov_x) {}

    [[nodiscard]] Ray Pinhole::sample_ray(Float w, Float h, Sampler &) const{
        Vector4f local_d = (m_sample_to_camera * Vector4f(w / static_cast<Float>(m_w),
                                                          h / static_cast<Float>(m_h),
                                                          Float0, Float1));
        local_d[3] = Float0;
        const Vector3f d = Block<0,0,3,1>(m_cam_to_world * local_d);

        return {m_pos, d.normalize()};
    }
}
