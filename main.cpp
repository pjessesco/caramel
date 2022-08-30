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

        scene.add_mesh(new OBJMesh(test_scene_path + "case1/object.obj"));

        DepthIntegrator integrator{scene};

        Image img = integrator.render();
        img.write_exr(test_scene_path+"case1/caramel_test1.exr");
    }

    std::cout<<"===================================="<<std::endl;

    // Test 2
    {
        Scene scene(Camera({0.0f, 0.0f, -2.0f},
                           {0.0f, 0.0f, 1.0f},
                           {0.0f, 1.0f, 0.0f},
                           500, 500, 50));
        scene.add_mesh(new OBJMesh(test_scene_path + "case2/object.obj"));

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

        scene.add_mesh(new OBJMesh(test_scene_path + "case3/object.obj"));

        DepthIntegrator integrator{scene};

        Image img = integrator.render();
        img.write_exr(test_scene_path+"case3/caramel_test3.exr");
    }


    // Test 4
    {
        constexpr int TEST_W = 100;
        constexpr int TEST_H = 100;

        auto tri = Caramel::OBJMesh(test_scene_path + "case4/bunny.obj");

        Camera cam({-0.0315182f, 0.284011f, 0.7331f},
                   {0.0191411f, -0.2299197f, -0.973022f},
                   {0.00717446f, 0.973206f, -0.229822f},
                   TEST_W, TEST_H, 16);

        Image img(TEST_W, TEST_H);

        auto time1 = std::chrono::high_resolution_clock::now();

        for(int i=0;i<TEST_W;i++){
            for(int j=0;j<TEST_H;j++){
                auto ray = cam.sample_ray(i, j);
                auto [intersect, u, v, t] = tri.ray_intersect(ray);
                img.set_pixel_value(i, j, t, t, t);
            }
        }
        
        auto time2 = std::chrono::high_resolution_clock::now();
        std::cout<<"Test4 : "<<std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count() << " [micro-s]"<<std::endl;

        img.write_exr(test_scene_path+"case4/caramel_test4.exr");
    }

    return 0;
}
