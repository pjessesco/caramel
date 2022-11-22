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

    enum class SamplingType{
        BSDF,
        LIGHT,
        MIS
    };

    class Integrator{
    public:
        Integrator();
        Image render(const Scene &scene, Index spp);

    protected:
        virtual Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) = 0;
    };

    class DepthIntegrator final : public Integrator{
    public:
        explicit DepthIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    class UVIntegrator final : public Integrator{
    public:
        explicit UVIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    class HitPosIntegrator final : public Integrator{
    public:
        explicit HitPosIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    class NormalIntegrator final : public Integrator{
    public:
        explicit NormalIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    class DirectIntegrator final : public Integrator{
    public:
        DirectIntegrator(SamplingType sampling_type);
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;

    private:
        Vector3f brdf_sampling_direct(const Scene &scene, Float i, Float j, Sampler &sampler);
        Vector3f emitter_sampling_direct(const Scene &scene, Float i, Float j, Sampler &sampler);

        SamplingType m_sampling_type;
    };

    class PathIntegrator final : public Integrator{
    public:
        PathIntegrator(Index max_depth, SamplingType sampling_type);
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;

    private:
        Vector3f brdf_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler);
        Vector3f emitter_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler);
        Vector3f mis_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler);

        Index m_max_depth;
        bool m_brdf_sampling;
        SamplingType m_sampling_type;
    };
}

