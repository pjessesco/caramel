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

#pragma once

#include <common.h>
#include <ray.h>

namespace Caramel{

    // Perspective camera
    struct Camera{
        Camera(const Vector3f &pos, const Vector3f &dir, const Vector3f &up,
               Index w, Index h, Float fov_x);

        [[nodiscard]] Ray sample_ray(Float w, Float h) const;

        const Vector3f m_pos;
        const Vector3f m_dir;
        const Vector3f m_up;
        Vector3f m_left;
        const Index m_w, m_h;
        const Float m_fov_x;
        Float m_cam_space_dir_z;
        Float m_ratio;
        Matrix44f m_cam_to_world;
    };

}
