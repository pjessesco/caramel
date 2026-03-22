#include "progressive_renderer.h"

#include <cmath>
#include <algorithm>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB
#endif
#include <GLFW/glfw3.h>

#include <camera.h>
#include <scene.h>
#include <integrators.h>
#include <image.h>

namespace Caramel::GUI{

    ProgressiveRenderer::ProgressiveRenderer() = default;

    ProgressiveRenderer::~ProgressiveRenderer(){
        stop();
        cleanup_texture();
    }

    void ProgressiveRenderer::start(Caramel::Scene &scene, Caramel::Integrator *integrator,
                                     const Caramel::Camera &cam_template,
                                     const Matrix44f &cam_to_world,
                                     Index render_w, Index render_h){
        stop();

        m_scene = &scene;
        m_integrator = integrator;
        m_cam_template = &cam_template;
        m_render_w = render_w;
        m_render_h = render_h;

        m_pending_cam_to_world = cam_to_world;
        m_accum_buffer.assign(render_w * render_h, vec3f_zero);
        m_render_image = std::make_unique<Image>(render_w, render_h);
        m_sample_count = 0;
        m_start_time = std::chrono::steady_clock::now();
        m_should_stop = false;
        m_is_dirty = false;
        m_running = true;

        m_owned_camera.reset(m_cam_template->clone_with_transform(cam_to_world, render_w, render_h));
        m_scene->set_camera(m_owned_camera.get());
        m_integrator->pre_process(*m_scene);

        m_render_thread = std::thread(&ProgressiveRenderer::render_thread_func, this);
    }

    void ProgressiveRenderer::stop(){
        m_should_stop = true;
        if(m_render_thread.joinable()){
            m_render_thread.join();
        }
        m_running = false;
        m_owned_camera.reset();
    }

    float ProgressiveRenderer::get_spp_per_second() const{
        int spp = m_sample_count.load();
        if(spp == 0) return 0.0f;
        auto elapsed = std::chrono::steady_clock::now() - m_start_time;
        float seconds = std::chrono::duration<float>(elapsed).count();
        return seconds > 0.0f ? static_cast<float>(spp) / seconds : 0.0f;
    }

    void ProgressiveRenderer::notify_camera_changed(const Matrix44f &new_cam_to_world){
        std::lock_guard<std::mutex> lock(m_cam_mutex);
        m_pending_cam_to_world = new_cam_to_world;
        m_is_dirty = true;
    }

    static constexpr int MAX_SPP = 200;

    void ProgressiveRenderer::render_thread_func(){
        while(!m_should_stop.load()){
            // Apply pending camera change
            if(m_is_dirty.load()){
                Matrix44f c2w;
                {
                    std::lock_guard<std::mutex> lock(m_cam_mutex);
                    c2w = m_pending_cam_to_world;
                }
                m_owned_camera.reset(m_cam_template->clone_with_transform(c2w, m_render_w, m_render_h));
                m_scene->set_camera(m_owned_camera.get());
                std::fill(m_accum_buffer.begin(), m_accum_buffer.end(), vec3f_zero);
                m_sample_count = 0;
                m_start_time = std::chrono::steady_clock::now();
                m_is_dirty = false;
            }

            // Check spp limit
            if(m_sample_count.load() >= MAX_SPP){
                // Wait until camera changes or stop
                while(!m_should_stop.load() && !m_is_dirty.load()){
                    std::this_thread::sleep_for(std::chrono::milliseconds(16));
                }
                continue;
            }

            // Render 1 spp using integrator's original loop structure
            RenderConfig config;
            config.spp = 1;
            config.random_seed = true;
            config.should_stop = &m_should_stop;

            m_integrator->render(*m_scene, *m_render_image, config);

            if(m_should_stop.load() || m_is_dirty.load()) continue;

            accumulate_and_upload(*m_render_image);
            m_sample_count.fetch_add(1);
        }
    }

    void ProgressiveRenderer::accumulate_and_upload(const Caramel::Image &img){
        if(!m_texture) return;

        int count = m_sample_count.load() + 1;
        float inv = 1.0f / static_cast<float>(count);

        auto to_srgb = [](float v) -> unsigned char{
            if(v <= 0.0031308f) v = 12.92f * v;
            else v = 1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f;
            return static_cast<unsigned char>(std::min(std::max(v, 0.0f), 1.0f) * 255.0f);
        };

        for(Index j = 0; j < m_render_h; j++){
            for(Index i = 0; i < m_render_w; i++){
                Vector3f pixel = img.get_pixel_value(i, j);
                Index idx = j * m_render_w + i;
                m_accum_buffer[idx] = m_accum_buffer[idx] + pixel;
                Vector3f color = m_accum_buffer[idx] * inv;
                m_display_buffer[idx*3 + 0] = to_srgb(color[0]);
                m_display_buffer[idx*3 + 1] = to_srgb(color[1]);
                m_display_buffer[idx*3 + 2] = to_srgb(color[2]);
            }
        }
    }

    void ProgressiveRenderer::init_texture(Index w, Index h){
        cleanup_texture();
        m_display_buffer.resize(w * h * 3, 0);

        glGenTextures(1, &m_texture);
        glBindTexture(GL_TEXTURE_2D, m_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void ProgressiveRenderer::cleanup_texture(){
        if(m_texture){
            glDeleteTextures(1, &m_texture);
            m_texture = 0;
        }
    }

    void ProgressiveRenderer::update_texture(){
        if(!m_texture) return;

        int current_spp = m_sample_count.load();
        if(current_spp == m_last_uploaded_spp) return;

        if(current_spp == 0){
            std::fill(m_display_buffer.begin(), m_display_buffer.end(), 0);
        }

        glBindTexture(GL_TEXTURE_2D, m_texture);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_render_w, m_render_h, GL_RGB, GL_UNSIGNED_BYTE, m_display_buffer.data());
        glBindTexture(GL_TEXTURE_2D, 0);
        m_last_uploaded_spp = current_spp;
    }
}
