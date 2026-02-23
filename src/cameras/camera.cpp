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
    Camera::Camera(const Matrix44f &cam_to_world, Index w, Index h, Float fov_x)
    : m_cam_to_world{cam_to_world}, m_w{w}, m_h{h}, m_fov_x{fov_x}, m_near{1e-4}, m_far{1000} {
        m_ratio = static_cast<Float>(m_w) / static_cast<Float>(m_h);

        const Float tmp1 = Float1 / (m_far - m_near);
        const Float cot = Float1 / tan(deg_to_rad(m_fov_x * Float0_5));

        const Matrix44f perspective{   cot, Float0,       Float0,                 Float0,
                                    Float0,    cot,       Float0,                 Float0,
                                    Float0, Float0, m_far * tmp1, -m_near * m_far * tmp1,
                                    Float0, Float0,       Float1,                 Float0};

        const Matrix44f camera_to_sample = scale(-Float0_5, -Float0_5 * m_ratio, Float1) *
                                           translate(-Float1, -Float1/m_ratio, Float0) *
                                           perspective;

        m_sample_to_camera = Inverse(camera_to_sample);
        m_pos = Block<0,0,3,1>(m_cam_to_world * Vector4f{0.0f, 0.0f, 0.0f, 1.0f});
    }

    Camera::Camera(const Vector3f &pos, const Vector3f &dir, const Vector3f &up,
                   Index w, Index h, Float fov_x)
        : m_pos{pos}, m_w{w}, m_h{h}, m_fov_x{fov_x}, m_near{1e-4}, m_far{1000} {
        Vector3f _dir = dir.normalize();
        Vector3f _up = up.normalize();
        // right-handed coord
        Vector3f left = Vector3f::cross(_up, _dir);
        _up = Vector3f::cross(_dir, left);

        m_cam_to_world = Matrix44f::from_cols(
                Vector4f{left[0], left[1], left[2], Float0},
                Vector4f{ _up[0],  _up[1],  _up[2], Float0},
                Vector4f{_dir[0], _dir[1], _dir[2], Float0},
                Vector4f{ pos[0],  pos[1],  pos[2], Float1}
        );

        m_ratio = static_cast<Float>(m_w) / static_cast<Float>(m_h);

        const Float tmp1 = Float1 / (m_far - m_near);
        const Float cot = Float1 / tan(deg_to_rad(m_fov_x * Float0_5));

        const Matrix44f perspective{   cot, Float0,       Float0,                 Float0,
                                    Float0,    cot,       Float0,                 Float0,
                                    Float0, Float0, m_far * tmp1, -m_near * m_far * tmp1,
                                    Float0, Float0,       Float1,                 Float0};

        const Matrix44f camera_to_sample = scale(-Float0_5, -Float0_5 * m_ratio, Float1) *
                                           translate(-Float1, -Float1/m_ratio, Float0) *
                                           perspective;

        m_sample_to_camera = Inverse(camera_to_sample);
    }
}
