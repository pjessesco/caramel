#include "preview_renderer.h"

#include <cmath>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB
#endif
#include <GLFW/glfw3.h>

#include <scene.h>
#include <shape.h>

namespace Caramel::GUI{

    static const char* vertex_shader_src = R"(#version 330 core
    layout(location = 0) in vec3 aPos;
    layout(location = 1) in vec3 aNormal;
    uniform mat4 mvp;
    uniform mat4 model;
    out vec3 vNormal;
    out vec3 vWorldPos;
    void main(){
    gl_Position = mvp * vec4(aPos, 1.0);
    vNormal = mat3(model) * aNormal;
    vWorldPos = vec3(model * vec4(aPos, 1.0));
}
)";

    static const char* fragment_shader_src = R"(#version 330 core
    in vec3 vNormal;
    in vec3 vWorldPos;
    uniform vec3 viewPos;
    out vec4 FragColor;
    void main(){
    vec3 normal = normalize(vNormal);
    vec3 viewDir = normalize(viewPos - vWorldPos);
    float diffuse = max(dot(normal, viewDir), 0.1);
    FragColor = vec4(vec3(0.7) * diffuse, 1.0);
}
)";

    static unsigned int compile_shader(unsigned int type, const char* src){
        unsigned int shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);

        int success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if(!success){
            char log[512];
            glGetShaderInfoLog(shader, 512, nullptr, log);
            fprintf(stderr, "Shader compile error: %s\n", log);
        }
        return shader;
    }

    void PreviewRenderer::init_shaders(){
        unsigned int vs = compile_shader(GL_VERTEX_SHADER, vertex_shader_src);
        unsigned int fs = compile_shader(GL_FRAGMENT_SHADER, fragment_shader_src);

        m_shader_program = glCreateProgram();
        glAttachShader(m_shader_program, vs);
        glAttachShader(m_shader_program, fs);
        glLinkProgram(m_shader_program);

        int success;
        glGetProgramiv(m_shader_program, GL_LINK_STATUS, &success);
        if(!success){
            char log[512];
            glGetProgramInfoLog(m_shader_program, 512, nullptr, log);
            fprintf(stderr, "Shader link error: %s\n", log);
        }

        glDeleteShader(vs);
        glDeleteShader(fs);

        m_loc_mvp = glGetUniformLocation(m_shader_program, "mvp");
        m_loc_model = glGetUniformLocation(m_shader_program, "model");
        m_loc_viewPos = glGetUniformLocation(m_shader_program, "viewPos");
    }

    PreviewRenderer::~PreviewRenderer(){
        cleanup();
    }

    void PreviewRenderer::cleanup(){
        if(m_vao){ glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
        if(m_vbo){ glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
        if(m_shader_program){ glDeleteProgram(m_shader_program); m_shader_program = 0; }
        m_vertex_count = 0;
    }

    void PreviewRenderer::upload_scene(const Scene &scene){
        cleanup();
        init_shaders();

        // Collect all triangle vertices + normals
        std::vector<float> vertex_data; // pos(3) + normal(3) interleaved

        for(const auto *shape : scene.m_meshes){
            const auto *mesh = dynamic_cast<const TriangleMesh*>(shape);
            if(mesh){
                Index tri_count = mesh->get_triangle_num();
                for(Index i = 0; i < tri_count; i++){
                    auto [v0, v1, v2] = mesh->get_triangle_vertices(i);
                    Vector3f edge1 = v1 - v0;
                    Vector3f edge2 = v2 - v0;
                    Vector3f normal = Vector3f::cross(edge1, edge2).normalize();

                    // v0
                    vertex_data.push_back(v0[0]); vertex_data.push_back(v0[1]); vertex_data.push_back(v0[2]);
                    vertex_data.push_back(normal[0]); vertex_data.push_back(normal[1]); vertex_data.push_back(normal[2]);
                    // v1
                    vertex_data.push_back(v1[0]); vertex_data.push_back(v1[1]); vertex_data.push_back(v1[2]);
                    vertex_data.push_back(normal[0]); vertex_data.push_back(normal[1]); vertex_data.push_back(normal[2]);
                    // v2
                    vertex_data.push_back(v2[0]); vertex_data.push_back(v2[1]); vertex_data.push_back(v2[2]);
                    vertex_data.push_back(normal[0]); vertex_data.push_back(normal[1]); vertex_data.push_back(normal[2]);
                }
            }
            else{
                // Standalone Triangle
                const auto &verts = shape->get_polygon_vertices();
                if(verts.size() >= 3){
                    Vector3f edge1 = verts[1] - verts[0];
                    Vector3f edge2 = verts[2] - verts[0];
                    Vector3f normal = Vector3f::cross(edge1, edge2).normalize();

                    for(int j = 0; j < 3; j++){
                        vertex_data.push_back(verts[j][0]); vertex_data.push_back(verts[j][1]); vertex_data.push_back(verts[j][2]);
                        vertex_data.push_back(normal[0]); vertex_data.push_back(normal[1]); vertex_data.push_back(normal[2]);
                    }
                }
            }
        }

        m_vertex_count = static_cast<int>(vertex_data.size()) / 6;

        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);

        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER, vertex_data.size() * sizeof(float), vertex_data.data(), GL_STATIC_DRAW);

        // position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // normal
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

    void PreviewRenderer::render(const Matrix44f &cam_to_world, float fov, float aspect){
        if(!m_vao) return;

        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);

        float near_val = 0.01f;
        float far_val = 1000.0f;
        float fov_rad = deg_to_rad(fov);
        float tan_half = std::tan(fov_rad * 0.5f);
        // fov is horizontal fov, so tan_half is for horizontal
        float right = near_val * tan_half;
        float top = right / aspect;

        float proj[16] = {};
        proj[0] = near_val / right;
        proj[5] = near_val / top;
        proj[10] = -(far_val + near_val) / (far_val - near_val);
        proj[11] = -1.0f;
        proj[14] = -2.0f * far_val * near_val / (far_val - near_val);

        // Build OpenGL view matrix from Caramel cam_to_world
        // Caramel convention: col0=left, col1=up, col2=forward (+Z = look direction)
        // OpenGL convention:  col0=right, col1=up, col2=-forward (-Z = look direction)
        // So: right = -left (negate col0), -forward = -col2 (negate col2)
        float r00 = cam_to_world(0,0), r01 = cam_to_world(0,1), r02 = cam_to_world(0,2);
        float r10 = cam_to_world(1,0), r11 = cam_to_world(1,1), r12 = cam_to_world(1,2);
        float r20 = cam_to_world(2,0), r21 = cam_to_world(2,1), r22 = cam_to_world(2,2);
        float tx = cam_to_world(0,3), ty = cam_to_world(1,3), tz = cam_to_world(2,3);

        // OpenGL view = inverse of OpenGL-convention cam_to_world
        // R_gl = [-left | up | -forward], view = [R_gl^T | -R_gl^T * t]
        float view[16] = {};
        view[0] = -r00; view[4] = -r10; view[8]  = -r20;   // right = -left
        view[1] =  r01; view[5] =  r11; view[9]  =  r21;   // up
        view[2] = -r02; view[6] = -r12; view[10] = -r22;    // -forward
        view[12] =  (r00*tx + r10*ty + r20*tz);              // -dot(right, pos) = dot(left, pos)
        view[13] = -(r01*tx + r11*ty + r21*tz);              // -dot(up, pos)
        view[14] =  (r02*tx + r12*ty + r22*tz);              // -dot(-fwd, pos) = dot(fwd, pos)
        view[15] = 1.0f;

        // MVP = proj * view (model = identity)
        float mvp[16] = {};
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                for(int k = 0; k < 4; k++){
                    mvp[i + j*4] += proj[i + k*4] * view[k + j*4];
                }
            }
        }

        float identity[16] = {};
        identity[0] = identity[5] = identity[10] = identity[15] = 1.0f;

        glUseProgram(m_shader_program);
        glUniformMatrix4fv(m_loc_mvp, 1, GL_FALSE, mvp);
        glUniformMatrix4fv(m_loc_model, 1, GL_FALSE, identity);
        glUniform3f(m_loc_viewPos, tx, ty, tz);

        glBindVertexArray(m_vao);
        glDrawArrays(GL_TRIANGLES, 0, m_vertex_count);
        glBindVertexArray(0);
        glUseProgram(0);
    }
}
