//
// Created by Jino Park on 2022/08/30.
//

#pragma once

#include <image.h>
#include <parallel_for.h>
#include <rayintersectinfo.h>
#include <scene.h>

namespace Caramel{
    class Integrator{
    public:
        Integrator(const Scene &scene) : m_scene{scene} {}

        Image render(){
            const Index width = m_scene.m_cam.m_w;
            const Index height = m_scene.m_cam.m_h;

            Image img(width, height);

            parallel_for(0, width, std::function([&](int i){
                for(int j=0;j<width;j++){
                    auto rgb = get_pixel_value(i, j);
                    img.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
                }
            }));

//            for(int i=0;i<width;i++){
//                for(int j=0;j<width;j++){
//                    auto rgb = get_pixel_value(i, j);
//                    img.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
//                }
//            }
            return img;
        }

    protected:
        virtual Vector3f get_pixel_value(Float i, Float j) = 0;
        Scene m_scene;
    };

    class DepthIntegrator : public Integrator{
    public:
        DepthIntegrator(const Scene &scene) : Integrator(scene) {}

        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override{
            const Ray ray = m_scene.m_cam.sample_ray(i, j);
            auto [is_hit, info] = m_scene.ray_intersect(ray);
            return is_hit ? Vector3f(info.t, info.t, info.t) : Vector3f();
        }
    };

    class UVIntegrator : public Integrator{
    public:
        UVIntegrator(const Scene &scene) : Integrator(scene) {}

        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override{
            const Ray ray = m_scene.m_cam.sample_ray(i, j);
            auto [is_hit, info] = m_scene.ray_intersect(ray);
            return is_hit ? Vector3f(info.u, info.v, Float1 - info.u - info.v) : Vector3f();
        }
    };

    class NormalIntegrator : public Integrator{
    public:
        NormalIntegrator(const Scene &scene) : Integrator(scene) {}

        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override{
            const Ray ray = m_scene.m_cam.sample_ray(i, j);
            auto [is_hit, info] = m_scene.ray_intersect(ray);
            return is_hit ? Vector3f(info.sh_n[0], info.sh_n[1], info.sh_n[2]) : Vector3f();
        }
    };
}
