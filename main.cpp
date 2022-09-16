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
#include <chrono>
#include <memory>

#include <logger.h>
#include <shape.h>
#include <camera.h>
#include <image.h>
#include <scene.h>
#include <integrators.h>

#include <Peanut/Peanut.h>

using namespace Caramel;

int main() {
    std::string test_scene_path = "/Users/jino/caramel/test_scenes/";

    // Test 1
    {
        Scene scene(Camera({0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           500, 500, 50));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case1/object.obj"));

        {
            DepthIntegrator integrator{scene};
            Image img = integrator.render();
            img.write_exr(test_scene_path+"case1/caramel_test1.exr");
        }
        {
            UVIntegrator integrator2{scene};
            Image img2 = integrator2.render();
            img2.write_exr(test_scene_path+"case1/caramel_test1_uv.exr");
        }
        {
            NormalIntegrator integrator2{scene};
            Image img2 = integrator2.render();
            img2.write_exr(test_scene_path+"case1/caramel_test1_normal.exr");
        }
    }

    std::cout<<"===================================="<<std::endl;

    // Test 2
    {
        Scene scene(Camera({0.0f, 0.0f, -2.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           500, 500, 50));
        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case2/object.obj"));

        DepthIntegrator integrator{scene};

        Image img = integrator.render();
        img.write_exr(test_scene_path+"case2/caramel_test2.exr");
    }

    std::cout<<"===================================="<<std::endl;

    // Test 3
    {
        Scene scene(Camera({0.536078f, -0.536043f, -2.50343f},
                           {-0.008999f, 0.009001f, 0.99992f},
                           {8.1006e-05f, 0.99996f, -0.00900047f},
                           500, 500, 50));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case3/object.obj"));

        DepthIntegrator integrator{scene};

        Image img = integrator.render();
        img.write_exr(test_scene_path+"case3/caramel_test3.exr");
    }

    std::cout<<"===================================="<<std::endl;

    // Test 4
    {
        Scene scene(Camera({-0.0315182f, 0.284011f, 0.7331f},
                           {0.0191411f, -0.2299197f, -0.973022f},
                           {0.00717446f, 0.973206f, -0.229822f},
                           200, 200, 16));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case4/bunny.obj"));

        {
            DepthIntegrator integrator{scene};
            Image img = integrator.render();
            img.write_exr(test_scene_path+"case4/caramel_test4.exr");
        }
        {
            NormalIntegrator integrator{scene};
            Image img = integrator.render();
            img.write_exr(test_scene_path+"case4/caramel_test_normal.exr");
        }
        {
            HitPosIntegrator integrator{scene};
            Image img = integrator.render();
            img.write_exr(test_scene_path+"case4/caramel_test_hitpos.exr");
        }
    }

    // Test 5
    {
        Scene scene(Camera({-65.6055f, 47.5762f, 24.3583f},
                           {0.7894f, -0.3551f, -0.500699f},
                           {0.299858f, 0.934836f, -0.190177f},
                           1000, 1000, 30));

        scene.add_mesh(std::make_shared<OBJMesh>(test_scene_path + "case5/ajax.obj"));

        {
            NormalIntegrator integrator{scene};
            Image img = integrator.render();
            img.write_exr(test_scene_path+"case5/caramel_test_normal.exr");
        }
        {
            HitPosIntegrator integrator{scene};
            Image img = integrator.render();
            img.write_exr(test_scene_path+"case5/caramel_test_hitpos.exr");
        }
    }

    return 0;
}
