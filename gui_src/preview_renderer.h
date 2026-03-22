#pragma once

#include <common.h>

namespace Caramel{ class Scene; }

namespace Caramel::GUI{

    class PreviewRenderer{
    public:
        PreviewRenderer() = default;
        ~PreviewRenderer();

        void upload_scene(const Caramel::Scene &scene);
        void render(const Matrix44f &cam_to_world, float fov, float aspect);
        void cleanup();

    private:
        void init_shaders();

        unsigned int m_vao = 0;
        unsigned int m_vbo = 0;
        unsigned int m_shader_program = 0;
        int m_vertex_count = 0;

        // Cached uniform locations
        int m_loc_mvp = -1;
        int m_loc_model = -1;
        int m_loc_viewPos = -1;
    };
}
