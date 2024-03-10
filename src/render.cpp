//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <scene_parser.h>
#include <shape.h>
#include <scene.h>
#include <image.h>
#include <integrators.h>
#include <logger.h>
#include <camera.h>
#include <image.h>

namespace Caramel{

    Image render(const std::filesystem::path &scene_path){
        // Set current path
        std::filesystem::current_path(scene_path.parent_path());

        SceneParser parser(scene_path);
        Integrator *integrator = parser.parse_integrator();
        Camera *cam = parser.parse_camera();
        std::vector<Shape*> shapes = parser.parse_shapes();
        std::vector<Light*> lights = parser.parse_lights();

        Scene scene;
        for(auto s : shapes){
            scene.add_mesh_and_arealight(s);
        }
        for(auto l : lights){
            scene.add_light(l);
        }
        scene.set_camera(cam);

        integrator->pre_process(scene);
        Image img = integrator->render(scene);

        return img;
    }
}
