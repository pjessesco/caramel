#include <iostream>

#include <logger.h>
#include <shape.h>
#include <camera.h>
#include <image.h>

#include <Peanut/Peanut.h>

using namespace Caramel;

int main() {

    std::string test_scene_path = "/Users/jino/test_scenes/";

    // Test 1
    {
        constexpr int TEST_W = 300;
        constexpr int TEST_H = 300;

        auto tri = Caramel::OBJMesh(test_scene_path + "case1/object.obj");

        Camera cam({0.0f, 0.0f, 0.0f},
                   {0.0f, 0.0f, 1.0f},
                   {0.0f, 1.0f, 0.0f},
                   300, 300, 50);

        Image img(TEST_W, TEST_H);

        for(int i=0;i<TEST_W;i++){
            for(int j=0;j<TEST_H;j++){
                auto ray = cam.sample_ray(i, j);
                auto [intersect, u, v, t] = tri.ray_intersect(ray);
                img.set_pixel_value(i, j, t, t, t); // FIXME : negative depth
            }
        }

        img.write_exr(test_scene_path+"case1/caramel_test1.exr");
    }

    // Test 2
    {
        constexpr int TEST_W = 300;
        constexpr int TEST_H = 300;

        auto tri = Caramel::OBJMesh(test_scene_path + "case2/object.obj");

        Camera cam({0.0f, 0.0f, -2.0f},
                   {0.0f, 0.0f, 1.0f},
                   {0.0f, 1.0f, 0.0f},
                   300, 300, 50);

        Image img(TEST_W, TEST_H);

        for(int i=0;i<TEST_W;i++){
            for(int j=0;j<TEST_H;j++){
                auto ray = cam.sample_ray(i, j);
                auto [intersect, u, v, t] = tri.ray_intersect(ray);
                img.set_pixel_value(i, j, -t, -t, -t); // FIXME : negative depth
            }
        }

        img.write_exr(test_scene_path+"case2/caramel_test2.exr");
    }


    {
        // Test 3
        {
            constexpr int TEST_W = 300;
            constexpr int TEST_H = 300;

            auto tri = Caramel::OBJMesh(test_scene_path + "case3/object.obj");

            Camera cam({0.527079f, -0.527042f, -1.50351f},
                       {-0.008999f, 0.009001f, 0.99992f},
                       {8.1006e-05f, 0.99996f, -0.00900047f},
                       300, 300, 50);

            Image img(TEST_W, TEST_H);

            for(int i=0;i<TEST_W;i++){
                for(int j=0;j<TEST_H;j++){
                    auto ray = cam.sample_ray(i, j);
                    auto [intersect, u, v, t] = tri.ray_intersect(ray);
                    img.set_pixel_value(i, j, -t, -t, -t); // FIXME : negative depth
                }
            }

            img.write_exr(test_scene_path+"case3/caramel_test2.exr");
        }
    }

    return 0;
}
