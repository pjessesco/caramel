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
                                                       new Diffuse(),
                                                       new AreaLight(Vector3f(30.0f, 30.0f, 30.0f)));

        scene.add_mesh(luminaire_obj);

        return scene;
    }
}