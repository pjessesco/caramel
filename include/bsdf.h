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

#include <tuple>

#include <common.h>
#include <ray.h>
#include <scene.h>
#include <warp_sample.h>

namespace Caramel{

    class BSDF{
    public:
        virtual std::tuple<Vector3f, Float> sample_recursive_info(const Vector3f &world_incoming_dir, Sampler &sampler) = 0;
        virtual Vector3f get_reflection(const Vector3f &world_incoming_dir) = 0;

    protected:
        explicit BSDF(const Scene &scene) : m_scene{scene} {}
        const Scene &m_scene;
    };

    class Diffuse : public BSDF{
        explicit Diffuse(const Scene &scene, const Vector3f &albedo = Vector3f{Float0_5, Float0_5, Float0_5})
            : BSDF(scene), m_albedo{albedo} {}

        std::tuple<Vector3f, Float> sample_recursive_info(const Vector3f &world_incoming_dir, Sampler &sampler){

        }


 
        const Vector3f &m_albedo;
    };


}