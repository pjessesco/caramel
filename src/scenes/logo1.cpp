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

//        Camera({278.0f, 273.0f, -800.0f},
//               {0.0f, 0.0f, 1.0f},
//               {0.0f, 1.0f, 0.0f},
//               1000, 1000, 39.3077)

        Scene scene;

        auto floor = std::make_shared<OBJMesh>(test_scene_path + "logo/meshes/cbox_floor.obj", new Diffuse(Vector3f{0.2f, 0.1f, 0.6f}));
        scene.add_mesh(floor);

        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/a_large.obj", new Diffuse());
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/a_small.obj", new Diffuse());
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/c_large.obj", new Diffuse());
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/c_small.obj", new Diffuse());
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/e_large.obj", new Diffuse());
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/e_small.obj", new Diffuse());
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/l_large.obj", new Diffuse());
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/l_small.obj", new Diffuse());
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/m_large.obj", new Diffuse());
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/m_small.obj", new Diffuse());
        }
        {
            auto ch1 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/r_large.obj", new Diffuse());
            auto ch2 = std::make_shared<OBJMesh>(test_scene_path + "logo/font1/r_small.obj", new Diffuse());
        }


        return scene;
    }
}