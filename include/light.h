//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
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
    class Shape;
    class Sampler;
    class Image;
    class RayIntersectInfo;

    class Light{
    public:
        Light() {}

        // Sample a point on the light from given pos
        // returns emitted radiance, sampled point on the light, normal at the sampled point, and its pdf
        virtual std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> sample_direct_contribution(const Scene &scene,
                                                                                                             const Vector3f &pos,
                                                                                                             Sampler &sampler) const = 0;

        // Probability of hitpos_world sampled from hitpos_world respect to solid angle
        // This function should not be used from delta lights since they don't have to be sampled
        virtual Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const = 0;

        // Returns emitted radiance
        // This function should not be used from delta lights, since they can't be captured by recursive ray from brdf sampling
        virtual Vector3f radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &light_normal_world) const = 0;

        virtual bool is_delta() const = 0;
        virtual bool is_envlight() const = 0;

        // Arealight is handled in AreaLight::Create
        template <typename Type, typename ...Param>
        static Light* Create(Param ...args){
            return dynamic_cast<Light*>(new Type(args...));
        }
    };

    class PointLight final : public Light{
    public:
        PointLight(const Vector3f &pos, const Vector3f &radiance);
        ~PointLight();

        Vector3f radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &) const override;
        std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> sample_direct_contribution(const Scene &scene,
                                                                                                     const Vector3f &hitpos,
                                                                                                     Sampler &) const override;

        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const override;

        bool is_delta() const override;
        bool is_envlight() const override;

        Vector3f m_pos;
        Vector3f m_radiance;
    };

    class AreaLight final : public Light{
    public:
        AreaLight(const Vector3f &radiance);
        ~AreaLight();
        
        Vector3f radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &light_normal_world) const override;
        std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> sample_direct_contribution(const Scene &scene,
                                                                                                     const Vector3f &hitpos,
                                                                                                     Sampler &sampler) const override;

        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const override;

        bool is_delta() const override;
        bool is_envlight() const override;

        template <typename ...Param>
        static AreaLight* Create(Param ...args){
            return new AreaLight(args...);
        }

        Shape *m_shape;
        Vector3f m_radiance;
    };

    class ConstantEnvLight final : public Light {
    public:
        ConstantEnvLight(const Vector3f &radiance, Float scale);

        Vector3f radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &light_normal_world) const override;
        std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> sample_direct_contribution(const Scene &scene,
                                                                                                     const Vector3f &hitpos,
                                                                                                     Sampler &sampler) const override;

        Float pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const override;

        bool is_delta() const override;
        bool is_envlight() const override;

        const Float m_scale;
        const Vector3f m_radiance;
        Image *m_image;
    };

}
