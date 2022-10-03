//
// Created by Jino Park on 2022/09/26.
//

#include <scene.h>
#include <shape.h>
#include <camera.h>
#include <light.h>

namespace Caramel{
    Scene scene_cbox_complex(){
        std::string test_scene_path = "/Users/jino/caramel/test_scenes/";

        Scene scene(Camera({278.0f, 273.0f, -800.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           1000, 1000, 39.3077));

        auto back = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_back.obj");
        scene.add_mesh(back);

        auto ceiling = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_ceiling.obj");
        scene.add_mesh(ceiling);

        auto floor = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_floor.obj");
        floor->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.2f, 0.1f, 0.6f});
        scene.add_mesh(floor);

        auto greenwall = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_greenwall.obj");
        greenwall->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.0f, 1.0f, 0.0f});
        scene.add_mesh(greenwall);

        auto largebox = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_largebox.obj", translate(40, 0, 40));
        largebox->m_bsdf = std::make_unique<Mirror>();
        scene.add_mesh(largebox);

        auto luminaire_obj = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_luminaire.obj");
        auto luminaire_light = std::make_shared<AreaLight>(scene, Vector3f(30.0f, 30.0f, 30.0f));
        scene.add_mesh(luminaire_obj);
        scene.add_light(luminaire_light);
        scene.enroll_arealight(luminaire_obj, luminaire_light);

        auto redwall = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_redwall.obj");
        redwall->m_bsdf = std::make_unique<Diffuse>(Vector3f{1.0f, 0.0f, 0.0f});
        scene.add_mesh(redwall);


        auto smallbox = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_smallbox.obj", translate(200, 0, 0) * scale(0.5, 0.5, 0.5));
        smallbox->m_bsdf = std::make_unique<Dielectric>();
        scene.add_mesh(smallbox);

        auto ajax = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/ajax.obj", translate(350, 80, 117) * scale(3, 3, 3) * rotate_y(90));
        ajax->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.2f, 0.2f, 0.7f});
        scene.add_mesh(ajax);

//        auto avatar = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cloth_resize.obj");
//        avatar->m_bsdf = std::make_unique<Diffuse>(Vector3f{0.9f, 0.3f, 0.2f});
//        scene.add_mesh(avatar);

        return scene;
    }
}