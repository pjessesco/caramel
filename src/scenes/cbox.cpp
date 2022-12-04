//
// Created by Jino Park on 2022/09/26.
//

#include <scene.h>
#include <shape.h>
#include <camera.h>
#include <light.h>

namespace Caramel{
    Scene scene_cbox_complex(){
        Scene scene;
        std::string test_scene_path = "../test_scenes/";

        auto back = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_back.obj",
                                              new Diffuse());
        scene.add_mesh(back);

        auto ceiling = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_ceiling.obj",
                                                 new Diffuse());
        scene.add_mesh(ceiling);

        auto floor = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_floor.obj",
                                               new Diffuse(Vector3f{0.2f, 0.1f, 0.6f}));
        scene.add_mesh(floor);

        auto greenwall = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_greenwall.obj",
                                                   new Diffuse(Vector3f{0.0f, 1.0f, 0.0f}));
        scene.add_mesh(greenwall);

        auto largebox = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_largebox.obj",
                                                  new Mirror(),
                                                  translate(40, 0, 40));
        scene.add_mesh(largebox);

        auto luminaire_obj = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_luminaire.obj",
                                                       new Diffuse());
        auto luminaire_light = std::make_shared<AreaLight>(scene, Vector3f(30.0f, 30.0f, 30.0f));
        scene.add_mesh(luminaire_obj);
        scene.add_light(luminaire_light);
        scene.enroll_arealight(luminaire_obj, luminaire_light);

        auto redwall = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_redwall.obj",
                                                 new Diffuse(Vector3f{1.0f, 0.0f, 0.0f}));
        scene.add_mesh(redwall);


        auto smallbox = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/cbox_smallbox.obj",
                                                  new Dielectric(),
                                                  translate(200, 0, 0) * scale(0.5, 0.5, 0.5));
        scene.add_mesh(smallbox);

        auto ajax = std::make_shared<OBJMesh>(test_scene_path + "cbox/meshes/ajax.obj",
                                              new Diffuse(Vector3f{0.2f, 0.2f, 0.7f}),
                                              translate(350, 80, 117) * scale(3, 3, 3) * rotate_y(90));
        scene.add_mesh(ajax);

        return scene;
    }
}