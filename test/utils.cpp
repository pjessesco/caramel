//
// This software is released under the MIT license.
//
// Copyright (c) 2022 Jino Park
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

#pragma once

#include <vector>
#include <string>

#include <common.h>
#include <image.h>
#include <logger.h>
#include <scene_parser.h>
#include <shape.h>
#include <scene.h>
#include <image.h>
#include <integrators.h>
#include <camera.h>

namespace Caramel {

    Image render_for_test(const std::filesystem::path &scene_path, Index w, Index h){
        // Set current path
        std::filesystem::current_path(scene_path.parent_path());

        SceneParser parser(scene_path);
        Integrator *integrator = parser.parse_integrator();
        Camera *cam = parser.parse_camera();
        if(w!=0 && h!=0){
            CRM_LOG("Set resolution manually");
            cam->set_size(w, h);
        }
        std::vector<Shape*> shapes = parser.parse_shapes();

        Scene scene;
        for(auto s : shapes){
            scene.add_mesh(s);
        }
        scene.set_camera(cam);

        integrator->pre_process(scene);
        Image img = integrator->render(scene);

        return img;
    }

    // abs
    Image diff(const Image &img1, const Image &img2){
        if((img1.size()[0] != img2.size()[0]) || (img1.size()[1] != img2.size()[1])){
            CRM_ERROR("size diff");
        }

        const Index w = img1.size()[0];
        const Index h = img1.size()[1];
        Image img(w, h);

        for(int i=0;i<w;i++){
            for(int j=0;j<h;j++){
                const Vector3f val1 = img1.get_pixel_value(i, j);
                const Vector3f val2 = img2.get_pixel_value(i, j);
                const Vector3f diff = val1 - val2;
                img.set_pixel_value(i, j, abs(diff[0]), abs(diff[1]), abs(diff[2]));
            }
        }
        return img;
    }

    Image square(const Image &img){
        const Index w = img.size()[0];
        const Index h = img.size()[1];
        Image sq_img(w, h);

        for(int i=0;i<w;i++){
            for(int j=0;j<h;j++){
                const Vector3f val = img.get_pixel_value(i, j);

                sq_img.set_pixel_value(i, j, val[0]*val[0], val[1]*val[1], val[2]*val[2]);
            }
        }
        return sq_img;
    }

    Float avg(const Image &img){
        Float sum = Float0;
        const Index w = img.size()[0];
        const Index h = img.size()[1];
        for(int i=0;i<w;i++){
            for(int j=0;j<h;j++){
                const Vector3f val = img.get_pixel_value(i, j);
                sum += (val[0] + val[1] + val[2]);
            }
        }
        return sum / (w * h * 3);
    }

    Float mse(const Image &img1, const Image &img2){
        return avg(square(diff(img1, img2)));
    }

    Float rmse(const Image &img1, const Image &img2){
        return Float0;
    }

}