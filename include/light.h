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

namespace Caramel{
    struct Scene;
    struct Shape;
    class Sampler;
    class RayIntersectInfo;

    class Light{
    public:
        Light() {}
        virtual Vector3f radiance() const = 0;
        // returns emitted radiance, sampled point, sampled normal, pdf
        virtual std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> sample_contribution(const Scene &scene,
                                                                                                      const Vector3f &pos,
                                                                                                      Sampler &sampler) const = 0;

        // Arealight is handled in AreaLight::Create
        template <typename Type, typename ...Param>
        static Light* Create(Param ...args){
            return dynamic_cast<Light*>(new Type(args...));
        }
    };

    class AreaLight final : public Light{
    public:
        AreaLight(const Vector3f &radiance);
        ~AreaLight();
        
        Vector3f radiance() const override;
        std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> sample_contribution(const Scene &scene,
                                                                                              const Vector3f &hitpos,
                                                                                              Sampler &sampler) const override;

        template <typename ...Param>
        static AreaLight* Create(Param ...args){
            return new AreaLight(args...);
        }

        Shape *m_shape;
        Vector3f m_radiance;
    };

}
