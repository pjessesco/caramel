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

        auto luminaire_obj = std::make_shared<OBJMesh>("meshes/cbox_luminaire.obj",
                                                       new Diffuse());
        auto luminaire_light = std::make_shared<AreaLight>(scene, Vector3f(30.0f, 30.0f, 30.0f));
        scene.add_mesh(luminaire_obj);
        scene.add_light(luminaire_light);
        scene.enroll_arealight(luminaire_obj, luminaire_light);

        return scene;
    }
}