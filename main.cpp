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

#include <iostream>
#include <memory>

#include <shape.h>
#include <camera.h>
#include <image.h>
#include <scene.h>
#include <integrators.h>

using namespace Caramel;

int main() {

    std::string test_scene_path = "/Users/jino/caramel/test_scenes/";

    bool test4 = false;
    bool test5 = false;
    bool test6 = true;
    bool test7 = true;

    // Test 4
    if(test4){
        Scene scene = scene_bunny();

        {
            DepthIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"bunny/caramel_test4.exr");
        }
        {
            NormalIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"bunny/caramel_test_normal.exr");
        }
        {
            HitPosIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"bunny/caramel_test_hitpos.exr");
        }
    }

    // Test 5
    if(test5){
        Scene scene = scene_ajax();

        {
            NormalIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"ajax/caramel_test_normal.exr");
        }
        {
            HitPosIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"ajax/caramel_test_hitpos.exr");
        }
    }

    // Test 6
    if(test6){
        Scene scene = scene_cbox_complex();
        {
            PathIntegrator integrator(scene, 5, SamplingType::BSDF);
            Image img = integrator.render(10);
            img.write_exr(test_scene_path+"cbox/caramel_test_path_brdf.exr");
        }
        {
            PathIntegrator integrator(scene, 5, SamplingType::LIGHT);
            Image img = integrator.render(10);
            img.write_exr(test_scene_path+"cbox/caramel_test_path_em.exr");
        }
    }


    return 0;
}
