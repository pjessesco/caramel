//
// Created by Jino Park on 2022/09/26.
//

#include <scene.h>
#include <shape.h>
#include <camera.h>
#include <light.h>

// ************************* WORK IN PROGRESS *************************

namespace Caramel{
    Scene scene_logo1(){
        std::string test_scene_path = "/Users/jino/caramel/test_scenes/";

        Scene scene(Camera({278.0f, 273.0f, -800.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           1000, 1000, 39.3077));

        auto floor = std::make_shared<OBJMesh>(test_scene_path + "logo/meshes/cbox_floor.obj");
        floor->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.2f, 0.1f, 0.6f});
        scene.add_mesh(floor);

        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/a_large.obj");
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/a_small.obj");
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/c_large.obj");
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/c_small.obj");
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/e_large.obj");
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/e_small.obj");
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/l_large.obj");
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/l_small.obj");
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/m_large.obj");
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/m_small.obj");
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/r_large.obj");
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/r_small.obj");
        }


        return scene;
    }
}