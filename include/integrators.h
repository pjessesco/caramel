//
// Created by Jino Park on 2022/08/30.
//

#pragma once

#include <chrono>

#include <image.h>
#include <scene.h>

namespace Caramel{
    class Integrator{
    public:
        Integrator(const Scene &scene);
        Image render();

    protected:
        virtual Vector3f get_pixel_value(Float i, Float j) = 0;
        Scene m_scene;
    };

    class DepthIntegrator : public Integrator{
    public:
        DepthIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override;
    };

    class UVIntegrator : public Integrator{
    public:
        UVIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override;
    };

    class HitPosIntegrator : public Integrator{
    public:
        HitPosIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override;
    };

    class NormalIntegrator : public Integrator{
    public:
        NormalIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Caramel::Float i, Caramel::Float j) override;
    };
}
