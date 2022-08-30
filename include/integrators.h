//
// Created by Jino Park on 2022/08/30.
//

#pragma once

#include <scene.h>
#include <image.h>
#include <rayintersectinfo.h>

namespace Caramel{
    class Integrator{
    public:
        Integrator(const Scene &scene) : m_scene{scene} {}
        virtual Image render() = 0;

    protected:
        Scene m_scene;
    };


    class DepthIntegrator : public Integrator{
    public:
        DepthIntegrator(const Scene &scene) : Integrator(scene) {}

        Image render() override{
            const Index width = m_scene.m_cam.m_w;
            const Index height = m_scene.m_cam.m_h;

            Image img(width, height);

            for(int i=0;i<width;i++){
                for(int j=0;j<width;j++){
                    const Ray ray = m_scene.m_cam.sample_ray(i, j);
                    auto [is_hit, info] = m_scene.ray_intersect(ray);
                    if(is_hit){
                        img.set_pixel_value(i, j, info.t, info.t, info.t);
                    }
                }
            }

            return img;
        }
    };

    class UVIntegrator : public Integrator{
    public:
        UVIntegrator(const Scene &scene) : Integrator(scene) {}

        Image render() override{
            const Index width = m_scene.m_cam.m_w;
            const Index height = m_scene.m_cam.m_h;

            Image img(width, height);

            for(int i=0;i<width;i++){
                for(int j=0;j<width;j++){
                    const Ray ray = m_scene.m_cam.sample_ray(i, j);
                    auto [is_hit, info] = m_scene.ray_intersect(ray);
                    if(is_hit){
                        img.set_pixel_value(i, j, info.u, info.v, Float1 - info.u - info.v);
                    }
                }
            }

            return img;
        }
    };
}
