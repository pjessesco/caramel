//
// Created by Jino Park on 2022/09/26.
//

#include <string>

#include <scene.h>
#include <shape.h>
#include <camera.h>
#include <light.h>

namespace Caramel{
    Scene scene_ajax(){
        std::string test_scene_path = "/Users/jino/caramel/test_scenes/";
        Scene scene(Camera({-65.6055f, 47.5762f, 24.3583f},
                           {0.7894f, -0.3551f, -0.500699f},
                           {0.299858f, 0.934836f, -0.190177f},
                           1000, 1000, 30));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case5/ajax.obj"));

        return scene;
    }
}