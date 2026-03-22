#pragma once

#include <memory>
#include <filesystem>
#include <common.h>

#include "camera_controller.h"
#include "preview_renderer.h"
#include "progressive_renderer.h"

struct GLFWwindow;

namespace Caramel{
    class Scene;
    class Integrator;
    class Camera;
}

namespace Caramel::GUI{

    enum class AppState{
        Idle,
        Preview,
        InteractiveRender
    };

    class Application{
    public:
        Application(GLFWwindow *window);
        ~Application();

        void update(float delta_time);

    private:
        void render_menu_bar();
        void render_viewport();
        void load_scene_file(const std::filesystem::path &path);
        void unload_scene();
        void reset_camera();
        void toggle_interactive_render();

        GLFWwindow *m_window;
        AppState m_state = AppState::Idle;

        CameraController m_camera_controller;
        PreviewRenderer m_preview_renderer;
        ProgressiveRenderer m_progressive_renderer;

        // Scene ownership
        std::unique_ptr<Caramel::Scene> m_scene;
        std::unique_ptr<Caramel::Integrator> m_integrator;
        std::unique_ptr<Caramel::Camera> m_original_camera;

        bool m_i_key_was_pressed = false;
        bool m_r_key_was_pressed = false;
        bool m_o_key_was_pressed = false;
        bool m_v_key_was_pressed = false;
        bool m_show_verbose = false;
        int m_frames_since_camera_move = 100;
    };
}
