#include "application.h"

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuiFileDialog.h>

#include <render.h>
#include <scene.h>
#include <camera.h>
#include <integrators.h>

namespace Caramel::GUI{

    Application::Application(GLFWwindow *window) : m_window(window){}

    Application::~Application(){
        unload_scene();
    }

    void Application::update(float delta_time){
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Update camera — uses GLFW polling directly, no conflict with ImGui
        if(m_state == AppState::Preview || m_state == AppState::InteractiveRender){
            m_camera_controller.update(m_window, delta_time);
        }

        // Keyboard shortcuts
        bool cmd = glfwGetKey(m_window, GLFW_KEY_LEFT_SUPER) == GLFW_PRESS ||
                   glfwGetKey(m_window, GLFW_KEY_RIGHT_SUPER) == GLFW_PRESS;

        // Cmd+O: open file
        if(cmd && glfwGetKey(m_window, GLFW_KEY_O) == GLFW_PRESS && !m_o_key_was_pressed){
            IGFD::FileDialogConfig config;
            config.path = ".";
            ImGuiFileDialog::Instance()->OpenDialog("OpenScene", "Open Scene", ".json", config);
        }
        m_o_key_was_pressed = (glfwGetKey(m_window, GLFW_KEY_O) == GLFW_PRESS);

        // I: toggle interactive render
        if(m_state != AppState::Idle &&
           glfwGetKey(m_window, GLFW_KEY_I) == GLFW_PRESS && !m_i_key_was_pressed){
            toggle_interactive_render();
        }
        m_i_key_was_pressed = (glfwGetKey(m_window, GLFW_KEY_I) == GLFW_PRESS);

        // R: reset camera
        if(m_state != AppState::Idle &&
           glfwGetKey(m_window, GLFW_KEY_R) == GLFW_PRESS && !m_r_key_was_pressed){
            reset_camera();
        }
        m_r_key_was_pressed = (glfwGetKey(m_window, GLFW_KEY_R) == GLFW_PRESS);

        // V: toggle verbose overlay
        if(glfwGetKey(m_window, GLFW_KEY_V) == GLFW_PRESS && !m_v_key_was_pressed){
            m_show_verbose = !m_show_verbose;
        }
        m_v_key_was_pressed = (glfwGetKey(m_window, GLFW_KEY_V) == GLFW_PRESS);

        // Track camera movement for interactive render
        if(m_camera_controller.is_dirty()){
            m_frames_since_camera_move = 0;
            if(m_state == AppState::InteractiveRender){
                m_progressive_renderer.notify_camera_changed(
                    m_camera_controller.get_cam_to_world()
                );
            }
            m_camera_controller.clear_dirty();
        }
        else{
            m_frames_since_camera_move++;
        }

        render_menu_bar();
        render_viewport();

        if(m_show_verbose && m_state != AppState::Idle){
            ImGui::SetNextWindowPos(ImVec2(10, ImGui::GetFrameHeight() + 10));
            ImGui::SetNextWindowBgAlpha(0.5f);
            ImGui::Begin("##verbose", nullptr,
                          ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                          ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs);

            Matrix44f c = m_camera_controller.get_cam_to_world();
            ImGui::Text("[%.2f %.2f %.2f %.2f]", c(0,0), c(0,1), c(0,2), c(0,3));
            ImGui::Text("[%.2f %.2f %.2f %.2f]", c(1,0), c(1,1), c(1,2), c(1,3));
            ImGui::Text("[%.2f %.2f %.2f %.2f]", c(2,0), c(2,1), c(2,2), c(2,3));
            ImGui::Text("[%.2f %.2f %.2f %.2f]", c(3,0), c(3,1), c(3,2), c(3,3));

            if(m_state == AppState::InteractiveRender){
                ImGui::Text("SPP: %d (%.1f spp/s)", m_progressive_renderer.get_sample_count(),
                            m_progressive_renderer.get_spp_per_second());
            }

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    void Application::render_menu_bar(){
        if(ImGui::BeginMainMenuBar()){
            if(ImGui::BeginMenu("File")){
                if(ImGui::MenuItem("Open...", "Cmd+O")){
                    IGFD::FileDialogConfig config;
                    config.path = ".";
                    ImGuiFileDialog::Instance()->OpenDialog(
                        "OpenScene", "Open Scene", ".json", config);
                }
                if(ImGui::MenuItem("Reset Camera", "R", false, m_state != AppState::Idle)){
                    reset_camera();
                }
                ImGui::EndMenu();
            }

            if(ImGui::BeginMenu("Render")){
                bool scene_loaded = (m_state != AppState::Idle);
                if(ImGui::MenuItem("Interactive Render", "I",
                                    m_state == AppState::InteractiveRender, scene_loaded)){
                    toggle_interactive_render();
                }
                ImGui::EndMenu();
            }

            if(ImGui::BeginMenu("View")){
                ImGui::MenuItem("Verbose Info", "V", &m_show_verbose);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        // File dialog
        if(ImGuiFileDialog::Instance()->Display("OpenScene")){
            if(ImGuiFileDialog::Instance()->IsOk()){
                std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
                load_scene_file(path);
            }
            ImGuiFileDialog::Instance()->Close();
        }
    }

    void Application::render_viewport(){
        int fb_w, fb_h;
        glfwGetFramebufferSize(m_window, &fb_w, &fb_h);
        int win_w, win_h;
        glfwGetWindowSize(m_window, &win_w, &win_h);

        if(m_state == AppState::Idle){
            glClearColor(0.18f, 0.18f, 0.18f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            return;
        }

        // Show GL preview when in Preview mode, or InteractiveRender while camera is moving
        bool show_preview = (m_state == AppState::Preview) ||
                            (m_state == AppState::InteractiveRender && m_frames_since_camera_move < 5);
        auto [cam_w, cam_h] = m_original_camera->get_size();
        float scene_aspect = static_cast<float>(cam_w) / static_cast<float>(std::max(cam_h, 1u));

        if(show_preview){
            float dpi_scale = static_cast<float>(fb_h) / static_cast<float>(std::max(win_h, 1));
            int menu_fb_h = static_cast<int>(ImGui::GetFrameHeight() * dpi_scale);
            int avail_fb_w = fb_w;
            int avail_fb_h = fb_h - menu_fb_h;
            float avail_aspect = static_cast<float>(avail_fb_w) / static_cast<float>(std::max(avail_fb_h, 1));

            int vp_w, vp_h, vp_x, vp_y;
            if(avail_aspect > scene_aspect){
                vp_h = avail_fb_h;
                vp_w = static_cast<int>(avail_fb_h * scene_aspect);
                vp_x = (avail_fb_w - vp_w) / 2;
                vp_y = 0;
            }
            else{
                vp_w = avail_fb_w;
                vp_h = static_cast<int>(avail_fb_w / scene_aspect);
                vp_x = 0;
                vp_y = (avail_fb_h - vp_h) / 2;
            }

            glClearColor(0.18f, 0.18f, 0.18f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glViewport(vp_x, vp_y, vp_w, vp_h);
            m_preview_renderer.render(m_camera_controller.get_cam_to_world(),
                                       m_original_camera->get_fov_x(), scene_aspect);
            return;
        }

        glClearColor(0.18f, 0.18f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);

        ImGui::SetNextWindowPos(ImVec2(0, ImGui::GetFrameHeight()));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(win_w),
                                         static_cast<float>(win_h) - ImGui::GetFrameHeight()));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", nullptr,
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                      ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImVec2 avail = ImGui::GetContentRegionAvail();

        if(m_state == AppState::InteractiveRender){
            m_progressive_renderer.update_texture();

            float viewport_aspect = avail.x / std::max(avail.y, 1.0f);

            ImVec2 img_size;
            if(viewport_aspect > scene_aspect){
                img_size.y = avail.y;
                img_size.x = avail.y * scene_aspect;
            }
            else{
                img_size.x = avail.x;
                img_size.y = avail.x / scene_aspect;
            }

            ImVec2 cursor = ImGui::GetCursorPos();
            cursor.x += (avail.x - img_size.x) * 0.5f;
            cursor.y += (avail.y - img_size.y) * 0.5f;
            ImGui::SetCursorPos(cursor);

            ImGui::Image(static_cast<ImTextureID>(m_progressive_renderer.get_texture_id()),
                          img_size, ImVec2(0, 0), ImVec2(1, 1));
        }

        ImGui::End();
        ImGui::PopStyleVar();
    }

    void Application::load_scene_file(const std::filesystem::path &path){
        unload_scene();

        auto [scene, integrator] = Caramel::build_scene(path);
        m_scene.reset(scene);
        m_integrator.reset(integrator);

        // Clone original camera for reset/template (before progressive renderer replaces it)
        auto [w, h] = m_scene->m_cam->get_size();
        m_original_camera.reset(m_scene->m_cam->clone_with_transform(
            m_scene->m_cam->get_cam_to_world(), w, h));

        m_camera_controller.init_from_camera(*m_original_camera);
        m_camera_controller.set_enabled(true);
        m_preview_renderer.upload_scene(*m_scene);

        m_state = AppState::Preview;
    }

    void Application::unload_scene(){
        m_progressive_renderer.stop();
        m_preview_renderer.cleanup();

        m_scene.reset();
        m_integrator.reset();
        m_original_camera.reset();

        m_state = AppState::Idle;
    }

    void Application::toggle_interactive_render(){
        if(m_state == AppState::InteractiveRender){
            m_progressive_renderer.stop();
            m_state = AppState::Preview;
        }
        else if(m_state == AppState::Preview){
            auto [cw, ch] = m_original_camera->get_size();
            float scale = std::min(1.0f, 400.0f / static_cast<float>(cw));
            Index rw = std::max(1u, static_cast<Index>(cw * scale));
            Index rh = std::max(1u, static_cast<Index>(ch * scale));

            m_progressive_renderer.init_texture(rw, rh);
            m_progressive_renderer.start(
                *m_scene, m_integrator.get(),
                *m_original_camera,
                m_camera_controller.get_cam_to_world(),
                rw, rh
            );
            m_camera_controller.clear_dirty();
            m_state = AppState::InteractiveRender;
        }
    }

    void Application::reset_camera(){
        if(m_state == AppState::Idle || !m_original_camera) return;

        // Stop any render
        if(m_state == AppState::InteractiveRender){
            m_progressive_renderer.stop();
            m_camera_controller.set_enabled(true);
        }

        // Re-init camera controller from original camera
        m_camera_controller.init_from_camera(*m_original_camera);
        m_state = AppState::Preview;
    }
}
