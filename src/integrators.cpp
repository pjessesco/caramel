//
// Created by Jino Park on 2022/08/30.
//

#include <chrono>

#include <integrators.h>
#include <image.h>
#include <parallel_for.h>
#include <rayintersectinfo.h>
#include <scene.h>
#include <progress.h>

namespace Caramel{

    Integrator::Integrator(const Scene &scene) : m_scene{scene} {}

    Image Integrator::render(){
        const Index width = m_scene.m_cam.m_w;
        const Index height = m_scene.m_cam.m_h;

        Image img(width, height);

        ProgressBar progress_bar(width);

        LOG("Render start...");

        auto time1 = std::chrono::high_resolution_clock::now();

        parallel_for(0, width, std::function([&](int i){
                         for(int j=0;j<height;j++){
                             auto rgb = get_pixel_value(i, j);
                             img.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
                         }
                         progress_bar.increase();
                     }));

        auto time2 = std::chrono::high_resolution_clock::now();
        LOG("Render done in " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() / 1000.0f) + " seconds");


        return img;
    }


    DepthIntegrator::DepthIntegrator(const Scene &scene) : Integrator(scene) {}

    Vector3f DepthIntegrator::get_pixel_value(Caramel::Float i, Caramel::Float j) {
        const Ray ray = m_scene.m_cam.sample_ray(i, j);
        auto [is_hit, info] = m_scene.ray_intersect(ray);
        return is_hit ? Vector3f(info.t, info.t, info.t) : Vector3f();
    }



    UVIntegrator::UVIntegrator(const Scene &scene) : Integrator(scene) {}

    Vector3f UVIntegrator::get_pixel_value(Caramel::Float i, Caramel::Float j) {
        const Ray ray = m_scene.m_cam.sample_ray(i, j);
        auto [is_hit, info] = m_scene.ray_intersect(ray);
        return is_hit ? Vector3f(info.u, info.v, Float1 - info.u - info.v) : Vector3f();
    }



    HitPosIntegrator::HitPosIntegrator(const Scene &scene) : Integrator(scene) {}

    Vector3f HitPosIntegrator::get_pixel_value(Caramel::Float i, Caramel::Float j) {
        const Ray ray = m_scene.m_cam.sample_ray(i, j);
        auto [is_hit, info] = m_scene.ray_intersect(ray);
        return is_hit ? Vector3f(info.p[0], info.p[1], info.p[2]) : Vector3f();
    }



    NormalIntegrator::NormalIntegrator(const Scene &scene) : Integrator(scene) {}

    Vector3f NormalIntegrator::get_pixel_value(Caramel::Float i, Caramel::Float j) {
        const Ray ray = m_scene.m_cam.sample_ray(i, j);
        auto [is_hit, info] = m_scene.ray_intersect(ray);
        return is_hit ? Vector3f(info.sh_n[0], info.sh_n[1], info.sh_n[2]) : Vector3f();
    }

}
