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
               Index w, Index h, Float near, Float far, Float fov_x)
        : m_pos{pos}, m_dir{dir}, m_w{w}, m_h{h}, m_near{near}, m_far{far}, m_fov_x{fov_x} {
            // right-handed coord
            m_left = cross(up, dir);

            m_cam_to_world = Matrix44f::from_cols(
                    Vector4f{m_left[0], m_left[1], m_left[2], Float0},
                    Vector4f{    up[0],     up[1],     up[2], Float0},
                    Vector4f{   dir[0],    dir[1],    dir[2], Float0},
                    Vector4f{   pos[0],    pos[1],    pos[2], Float1}
                    );
            m_world_to_cam = Inverse(m_cam_to_world);
        }

        Ray sample_ray(Float w, Float h){
            


        }

        Vector3f m_pos;
        Vector3f m_dir;
        Vector3f m_up;
        Vector3f m_left;
        Index m_w, m_h;
        Float m_near, m_far;
        Float m_fov_x;
        Matrix44f m_cam_to_world;
        Matrix44f m_world_to_cam;
    };

}
