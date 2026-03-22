#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <common.h>

namespace Caramel{
    class Scene;
    class Integrator;
    class Camera;
    class Image;
}

namespace Caramel::GUI{

    class ProgressiveRenderer{
    public:
        ProgressiveRenderer();
        ~ProgressiveRenderer();

        void start(Caramel::Scene &scene, Caramel::Integrator *integrator,
                   const Caramel::Camera &cam_template,
                   const Matrix44f &cam_to_world,
                   Index render_w, Index render_h);

        void stop();
        void notify_camera_changed(const Matrix44f &new_cam_to_world);

        void init_texture(Index w, Index h);
        void update_texture();
        void cleanup_texture();
        unsigned int get_texture_id() const{ return m_texture; }

        int get_sample_count() const{ return m_sample_count.load(); }
        float get_spp_per_second() const;
        bool is_running() const{ return m_running.load(); }

        Index get_render_width() const{ return m_render_w; }
        Index get_render_height() const{ return m_render_h; }

    private:
        void render_thread_func();
        void accumulate_and_upload(const Caramel::Image &img);

        Caramel::Scene *m_scene = nullptr;
        Caramel::Integrator *m_integrator = nullptr;
        const Caramel::Camera *m_cam_template = nullptr;
        std::unique_ptr<Caramel::Camera> m_owned_camera;

        Index m_render_w = 0;
        Index m_render_h = 0;
        std::unique_ptr<Caramel::Image> m_render_image;

        // Accumulation
        std::vector<Vector3f> m_accum_buffer;
        std::atomic<int> m_sample_count{0};
        int m_last_uploaded_spp = -1;
        std::chrono::steady_clock::time_point m_start_time;

        std::atomic<bool> m_should_stop{false};
        std::atomic<bool> m_is_dirty{false};
        std::atomic<bool> m_running{false};

        std::mutex m_cam_mutex;
        std::thread m_render_thread;

        Matrix44f m_pending_cam_to_world = Matrix44f::identity();

        // GL texture
        unsigned int m_texture = 0;
        std::vector<unsigned char> m_display_buffer;
    };
}
