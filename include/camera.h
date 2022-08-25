//
// Created by Jino Park on 2022/08/08.
//

#pragma once

#include <common.h>
#include <ray.h>

namespace Caramel{

    // Perspective camera
    struct Camera{
        Camera(const Vector3f pos, const Vector3f dir, const Vector3f up,
               Index w, Index h, Float fov_x)
        : m_pos{pos}, m_dir{dir}, m_w{w}, m_h{h}, m_fov_x{fov_x} {
            // right-handed coord
            m_left = cross(up, dir);

            m_cam_to_world = Matrix44f::from_cols(
                    Vector4f{m_left[0], m_left[1], m_left[2], Float0},
                    Vector4f{    up[0],     up[1],     up[2], Float0},
                    Vector4f{   dir[0],    dir[1],    dir[2], Float0},
                    Vector4f{   pos[0],    pos[1],    pos[2], Float1}
                    );

            m_tan = tan(m_fov_x * PI / (2 * 180));
            m_ratio = static_cast<Float>(m_w) / static_cast<Float>(m_h);
        }

        Ray sample_ray(Float w, Float h){
            Vector4f local_d{-(w/m_w - 0.5f) * 2 * m_ratio,
                             -(h/m_h - 0.5f) * 2 ,
                             m_ratio / tan(m_fov_x * PI / (2 * 180)),
                             Float0};

            Vector3f d = Block<0,0,3,1>(m_cam_to_world * local_d);

            return Ray(m_pos, d.normalize());
        }

        const Vector3f m_pos;
        const Vector3f m_dir;
        const Vector3f m_up;
        Vector3f m_left;
        const Index m_w, m_h;
        const Float m_fov_x;
        Float m_tan;
        Float m_ratio;
        Matrix44f m_cam_to_world;
    };

}
