//
// Created by Jino Park on 2022/09/26.
//

#include <scene.h>
#include <shape.h>
#include <camera.h>
#include <light.h>

namespace Caramel{
    Scene scene_bunny(){
        std::string test_scene_path = "/Users/jino/caramel/test_scenes/";

        Scene scene(Camera({-0.0315182f, 0.284011f, 0.7331f},
                           {0.0191411f, -0.2299197f, -0.973022f},
                           {0.00717446f, 0.973206f, -0.229822f},
                           200, 200, 16));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "bunny/bunny.obj"));
        return scene;
    }
}