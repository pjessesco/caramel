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

#pragma once

#include <common.h>

namespace Caramel{
    class Ray;

    // Perspective camera
    class Camera{
    public:
        Camera(const Vector3f &pos, const Vector3f &dir, const Vector3f &up,
               Index w, Index h, Float fov_x);
        Camera(const Matrix44f &cam_to_world, Index w, Index h, Float fov_x);

        [[nodiscard]] virtual Ray sample_ray(Float w, Float h) const = 0;

        template <typename Type, typename ...Param>
        static Camera* Create(Param ...args){
            return dynamic_cast<Camera*>(new Type(args...));
        }

        std::pair<Index, Index> get_size() const{
            return {m_w, m_h};
        }

        // Used for test
        void set_size(Index width, Index height){
            m_w = width;
            m_h = height;
        }

    protected:
        Vector3f m_pos;
        const Float m_fov_x;
        Index m_w, m_h;
        Float m_ratio;
        Float m_near;
        Float m_far;
        Matrix44f m_sample_to_camera;
        Matrix44f m_cam_to_world;
    };

    class Pinhole : public Camera {
    public:
        Pinhole(const Vector3f &pos, const Vector3f &dir, const Vector3f &up,
               Index w, Index h, Float fov_x);
        Pinhole(const Matrix44f &cam_to_world, Index w, Index h, Float fov_x);

        [[nodiscard]] Ray sample_ray(Float w, Float h) const override;
    };
}
