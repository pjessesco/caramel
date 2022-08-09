#include <iostream>

#include <logger.h>
#include <shape.h>
#include <camera.h>

#include <Peanut/Peanut.h>

using namespace Caramel;

int main() {
    std::cout << "Hello, World!" << std::endl;

//    Caramel::OBJMesh("/Users/jino/caramel/bunny.obj");
//    Caramel::OBJMesh("/Users/jino/caramel/ajax.obj");

    Camera cam({-0.0315182f, 0.284011f, 0.7331f},
               {-0.0191411f,  0.2299197f,  0.973022f,},
               {0.00717446f, 0.973206f, -0.229822f},
               40, 30, 45);


    for(int i=0;i<40;i+=5){
        for(int j=0;j<30;j+=5){
            auto ray = cam.sample_ray(i, j);
            std::cout<<"["<<ray.m_d[0]<<","<<ray.m_d[1]<<","<<ray.m_d[2]<<"],"<<std::endl;
        }
    }


    return 0;
}
