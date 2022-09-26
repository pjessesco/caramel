//
// Created by Jino Park on 2022/09/26.
//

#include <scene.h>
#include <shape.h>
#include <camera.h>
#include <light.h>

namespace Caramel{
    Scene scene_cbox(){
        std::string test_scene_path = "/Users/jino/caramel/test_scenes/";

        Scene scene(Camera({278.0f, 273.0f, -800.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           300, 300, 39.3077));

        auto back = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_back.obj");
        scene.add_mesh(back);

        auto ceiling = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_ceiling.obj");
        scene.add_mesh(ceiling);

        auto floor = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_floor.obj");
        scene.add_mesh(floor);

        auto greenwall = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_greenwall.obj");
        greenwall->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.0f, 1.0f, 0.0f});
        scene.add_mesh(greenwall);

        auto largebox = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_largebox.obj");
//        largebox->m_bsdf = std::make_unique<Mirror>();
        scene.add_mesh(largebox);

        auto luminaire_obj = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_luminaire.obj");
        auto luminaire_light = std::make_shared<AreaLight>(scene, Vector3f(30.0f, 30.0f, 30.0f));
        scene.add_mesh(luminaire_obj);
        scene.add_light(luminaire_light);
        scene.enroll_arealight(luminaire_obj, luminaire_light);


        auto redwall = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_redwall.obj");
        redwall->m_bsdf = std::make_unique<Diffuse>(Vector3f{1.0f, 0.0f, 0.0f});
        scene.add_mesh(redwall);

        auto smallbox = std::make_shared<OBJMesh>(test_scene_path + "case6/meshes/cbox_smallbox.obj");
        smallbox->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.6f, 0.3f, 0.4f});
        scene.add_mesh(smallbox);

        return scene;
    }
}