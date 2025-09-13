//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2025 Jino Park
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

#include <common.h>

namespace Caramel{
    class Image;
    class Scene;
    class Sampler;

    inline Float balance_heuristic(Float a, Float b){
        return a / (a + b);
    }

    class Integrator{
    public:
        Integrator() = default;
        virtual void pre_process(const Scene &scene) = 0;
        virtual Image render(const Scene &scene) = 0;

        template <typename Type, typename ...Param>
        static Integrator* Create(Param ...args){
            return dynamic_cast<Integrator*>(new Type(args...));
        }
    };

    class MCIntegrator : public Integrator{
    public:
        explicit MCIntegrator(Index m_spp = 1);
        void pre_process(const Scene &scene) override;
        Image render(const Scene &scene) override;

    protected:
        virtual Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) = 0;
        Index m_spp;
    };

    class DepthIntegrator final : public MCIntegrator{
    public:
        DepthIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;

    };

    class UVIntegrator final : public MCIntegrator{
    public:
        UVIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    class HitPosIntegrator final : public MCIntegrator{
    public:
        HitPosIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    class NormalIntegrator final : public MCIntegrator{
    public:
        NormalIntegrator();
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    };

    // class DirectIntegrator final : public MCIntegrator{
    // public:
    //     DirectIntegrator(Index spp);
    //     Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;
    //
    // private:
    //     Vector3f mis_sampling_direct(const Scene &scene, Float i, Float j, Sampler &sampler);
    // };

    class PathIntegrator final : public MCIntegrator{
    public:
        PathIntegrator(Index rr_depth, Index max_depth, Index spp);
        Vector3f get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) override;

    private:
        Vector3f mis_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler) const;

        Index m_rr_depth;
        Index m_max_depth;
    };


}

