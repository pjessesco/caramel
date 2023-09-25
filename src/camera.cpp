//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
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
#include <ray.h>

namespace Caramel{

    // Perspective camera
    Camera::Camera(const Vector3f &pos, const Vector3f &dir, const Vector3f &up,
                   Index w, Index h, Float fov_x)
        : m_pos{pos}, m_dir{dir.normalize()}, m_up(up.normalize()), m_w{w}, m_h{h}, m_fov_x{fov_x} {
        // right-handed coord
        m_left = cross(m_up, m_dir);

        m_cam_to_world = Matrix44f::from_cols(
                Vector4f{m_left[0], m_left[1], m_left[2], Float0},
                Vector4f{  m_up[0],   m_up[1],   m_up[2], Float0},
                Vector4f{ m_dir[0],  m_dir[1],  m_dir[2], Float0},
                Vector4f{   pos[0],    pos[1],    pos[2], Float1}
        );

        m_ratio = static_cast<Float>(m_w) / static_cast<Float>(m_h);
        // m_ratio / tangent of half fov
        m_cam_space_dir_z = m_ratio / tan(m_fov_x * PI / (360));
    }

    [[nodiscard]] Ray Camera::sample_ray(Float w, Float h) const{
        const Vector4f local_d{-(w / static_cast<Float>(m_w) - Float0_5) * 2 * m_ratio,
                               -(h / static_cast<Float>(m_h) - Float0_5) * 2 ,
                               m_cam_space_dir_z,
                               Float0};

        const Vector3f d = Block<0,0,3,1>(m_cam_to_world * local_d);

        return {m_pos, d.normalize()};
    }
}
