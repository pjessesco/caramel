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

    bool test1 = false;
    bool test2 = false;
    bool test3 = false;
    bool test4 = false;
    bool test5 = false;
    bool test6 = true;

    // Test 1
    if(test1){
        Scene scene(Camera({0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           500, 500, 50));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case1/object.obj"));

        {
            DepthIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"case1/caramel_test1.exr");
        }
        {
            UVIntegrator integrator2{scene};
            Image img2 = integrator2.render(1);
            img2.write_exr(test_scene_path+"case1/caramel_test1_uv.exr");
        }
        {
            NormalIntegrator integrator2{scene};
            Image img2 = integrator2.render(1);
            img2.write_exr(test_scene_path+"case1/caramel_test1_normal.exr");
        }
    }

    std::cout<<"===================================="<<std::endl;

    // Test 2
    if(test2){
        Scene scene(Camera({0.0f, 0.0f, -2.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           500, 500, 50));
        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case2/object.obj"));

        DepthIntegrator integrator{scene};

        Image img = integrator.render(1);
        img.write_exr(test_scene_path+"case2/caramel_test2.exr");
    }

    std::cout<<"===================================="<<std::endl;

    // Test 3
    if(test3){
        Scene scene(Camera({0.536078f, -0.536043f, -2.50343f},
                           {-0.008999f, 0.009001f, 0.99992f},
                           {8.1006e-05f, 0.99996f, -0.00900047f},
                           500, 500, 50));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case3/object.obj"));

        DepthIntegrator integrator{scene};

        Image img = integrator.render(1);
        img.write_exr(test_scene_path+"case3/caramel_test3.exr");
    }

    std::cout<<"===================================="<<std::endl;

    // Test 4
    if(test4){
        Scene scene = scene_bunny();

        {
            DepthIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"case4/caramel_test4.exr");
        }
        {
            NormalIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"case4/caramel_test_normal.exr");
        }
        {
            HitPosIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"case4/caramel_test_hitpos.exr");
        }
    }

    std::cout<<"===================================="<<std::endl;

    // Test 5
    if(test5){
        Scene scene = scene_ajax();

        {
            NormalIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"case5/caramel_test_normal.exr");
        }
        {
            HitPosIntegrator integrator{scene};
            Image img = integrator.render(1);
            img.write_exr(test_scene_path+"case5/caramel_test_hitpos.exr");
        }
    }

    std::cout<<"===================================="<<std::endl;

    // Test 6
    if(test6){
        Scene scene = scene_cbox();

        {
            DirectIntegrator integrator{scene};
            Image img = integrator.render(20);
            img.write_exr(test_scene_path+"case6/caramel_test_brdf.exr");
        }

        {
            PathIntegrator integrator(scene, 5, PathIntegrator::SamplingType::BSDF);
            Image img = integrator.render(100);
            img.write_exr(test_scene_path+"case6/caramel_test_path_brdf.exr");
        }
        {
            PathIntegrator integrator(scene, 5, PathIntegrator::SamplingType::LIGHT);
            Image img = integrator.render(100);
            img.write_exr(test_scene_path+"case6/caramel_test_path_em.exr");
        }

        {
            PathIntegrator integrator(scene, 5, PathIntegrator::SamplingType::MIS);
            Image img = integrator.render(100);
            img.write_exr(test_scene_path+"case6/caramel_test_path_mis.exr");
        }
    }

    return 0;
}
