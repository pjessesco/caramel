#include <iostream>

#include <logger.h>
#include <shape.h>
#include <camera.h>
#include <image.h>

#include <Peanut/Peanut.h>

using namespace Caramel;

int main() {
    constexpr int TEST_W = 768;
    constexpr int TEST_H = 768;

    std::cout << "Hello, World!" << std::endl;

    auto bunny = Caramel::OBJMesh("/Users/jino/caramel/bunny.obj");

    Camera cam({-0.0315182f, 0.284011f, 0.7331f},
               {-0.0191411f,  0.2299197f,  0.973022f},
               {0.00717446f, 0.973206f, -0.229822f},
               TEST_W, TEST_H, 16);

    Image img(TEST_W, TEST_H);

    for(int i=0;i<TEST_W;i++){
        for(int j=0;j<TEST_H;j++){
            auto ray = cam.sample_ray(i, j);
            auto [intersect, u, v, t] = bunny.ray_intersect(ray);
            img.set_pixel_value(i, j, -t, -t, -t); // FIXME : negative depth
        }
    }

    img.write_exr("bunny_depth.exr");

    return 0;
}
