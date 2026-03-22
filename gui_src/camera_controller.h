#pragma once

#include <common.h>

struct GLFWwindow;

namespace Caramel{ class Camera; }

namespace Caramel::GUI{

    class CameraController{
    public:
        CameraController() = default;

        void init_from_camera(const Caramel::Camera &cam);
        void update(GLFWwindow *window, float delta_time);

        Matrix44f get_cam_to_world() const;

        bool is_dirty() const{ return m_dirty; }
        void clear_dirty(){ m_dirty = false; }

        void set_enabled(bool enabled){ m_enabled = enabled; }

    private:
        Vector3f m_position{Float0, Float0, Float0};
        float m_yaw = 0.0f;
        float m_pitch = 0.0f;

        float m_move_speed = 2.0f;
        float m_mouse_sensitivity = 0.15f;

        bool m_dirty = false;
        bool m_enabled = true;
        bool m_right_mouse_pressed = false;
        double m_last_mouse_x = 0.0;
        double m_last_mouse_y = 0.0;
    };
}
