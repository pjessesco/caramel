#include "camera_controller.h"

#include <cmath>
#include <GLFW/glfw3.h>

#include <camera.h>

namespace Caramel::GUI{

    void CameraController::init_from_camera(const Camera &cam){
        Matrix44f c2w = cam.get_cam_to_world();

        m_position = Vector3f{c2w(0,3), c2w(1,3), c2w(2,3)};

        // Caramel convention: column 2 IS the forward direction (+Z in camera space)
        Vector3f forward{c2w(0,2), c2w(1,2), c2w(2,2)};

        m_pitch = std::asin(std::clamp(forward[1], -1.0f, 1.0f));
        m_yaw = std::atan2(forward[0], -forward[2]);

        m_dirty = false;
    }

    void CameraController::update(GLFWwindow *window, float delta_time){
        if(!m_enabled) return;

        // Mouse right button for rotation
        int right_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);

        if(right_state == GLFW_PRESS){
            if(m_right_mouse_pressed){
                float dx = static_cast<float>(mx - m_last_mouse_x);
                float dy = static_cast<float>(my - m_last_mouse_y);

                m_yaw -= dx * m_mouse_sensitivity * 0.01f;
                m_pitch += dy * m_mouse_sensitivity * 0.01f;
                m_pitch = std::clamp(m_pitch, -1.5f, 1.5f); // ~86 degrees

                if(dx != 0.0f || dy != 0.0f) m_dirty = true;
            }
            m_right_mouse_pressed = true;
        }
        else{
            m_right_mouse_pressed = false;
        }
        m_last_mouse_x = mx;
        m_last_mouse_y = my;

        // Compute forward/right/up vectors from yaw/pitch
        float cy = std::cos(m_yaw), sy = std::sin(m_yaw);
        float cp = std::cos(m_pitch), sp = std::sin(m_pitch);

        Vector3f forward{sy * cp, sp, -cy * cp};
        Vector3f world_up{Float0, Float1, Float0};
        Vector3f right_vec = Vector3f::cross(forward, world_up).normalize();
        Vector3f up_vec = Vector3f::cross(right_vec, forward).normalize();

        float speed = m_move_speed * delta_time;

        // WASD movement
        if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
            m_position = m_position + forward * speed;
            m_dirty = true;
        }
        if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
            m_position = m_position - forward * speed;
            m_dirty = true;
        }
        if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
            m_position = m_position - right_vec * speed;
            m_dirty = true;
        }
        if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
            m_position = m_position + right_vec * speed;
            m_dirty = true;
        }
        if(glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS){
            m_position = m_position + up_vec * speed;
            m_dirty = true;
        }
        if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS){
            m_position = m_position - up_vec * speed;
            m_dirty = true;
        }
    }

    Matrix44f CameraController::get_cam_to_world() const{
        float cy = std::cos(m_yaw), sy = std::sin(m_yaw);
        float cp = std::cos(m_pitch), sp = std::sin(m_pitch);

        // Caramel convention: column 0 = left, column 1 = up, column 2 = forward
        Vector3f forward{sy * cp, sp, -cy * cp};
        Vector3f world_up{Float0, Float1, Float0};
        Vector3f left_vec = Vector3f::cross(world_up, forward).normalize();
        Vector3f up_vec = Vector3f::cross(forward, left_vec).normalize();

        return Matrix44f{
            left_vec[0],   up_vec[0],   forward[0],  m_position[0],
            left_vec[1],   up_vec[1],   forward[1],  m_position[1],
            left_vec[2],   up_vec[2],   forward[2],  m_position[2],
            Float0,        Float0,      Float0,      Float1
        };
    }
}
