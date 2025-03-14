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

#include <tuple>

#include <light.h>

#include <ray.h>
#include <common.h>
#include <sampler.h>
#include <scene.h>
#include <transform.h>
#include <image.h>
#include <rayintersectinfo.h>
#include <warp_sample.h>


namespace Caramel{
    EnvmapLight::EnvmapLight(const Vector3f &radiance,
                             Float scale,
                             const Vector3f &sceneCenter,
                             Float sceneRadius)
        : m_radiance(radiance), m_scale(scale), m_sceneCenter(sceneCenter), m_sceneRadius(sceneRadius) {}

    Vector3f EnvmapLight::radiance(const Vector3f &hitpos, const Vector3f &lightpos, const Vector3f &light_normal_world) const{
        return m_radiance;
    }

    std::tuple<Vector3f, Vector3f, Vector3f, Float, RayIntersectInfo> EnvmapLight::sample_direct_contribution(const Scene &scene, const Vector3f &hitpos, Sampler &sampler) const{
        auto [light_dir, pos_pdf] = sample_unit_sphere_uniformly(sampler);
        Vector3f light_pos = (light_dir * scene.m_sceneRadius) + scene.m_sceneCenterPos;
            ???
    }

    Float EnvmapLight::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &lightpos_world, const Vector3f &light_normal_world) const{
        return PI_4_INV;
    }

    bool EnvmapLight::is_delta() const {
        return false;
    }

    bool PointLight::is_envlight() const {
        return true;
    }

}
