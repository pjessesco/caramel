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

#pragma once

#include <chrono>

#include <image.h>
#include <scene.h>

namespace Caramel{
    class Integrator{
    public:
        explicit Integrator(Scene scene);
        Image render();

    protected:
        virtual Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) = 0;
        Scene m_scene;
    };

    class DepthIntegrator : public Integrator{
    public:
        explicit DepthIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) override;
    };

    class UVIntegrator : public Integrator{
    public:
        explicit UVIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) override;
    };

    class HitPosIntegrator : public Integrator{
    public:
        explicit HitPosIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) override;
    };

    class NormalIntegrator : public Integrator{
    public:
        explicit NormalIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) override;
    };

    class AlbedoIntegrator : public Integrator{
    public:
        explicit AlbedoIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) override;
    };

    class DirectIntegrator : public Integrator{
    public:
        explicit DirectIntegrator(const Scene &scene);
        Vector3f get_pixel_value(Float i, Float j, Sampler &sampler) override;
    };
}

