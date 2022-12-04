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

#include <memory>
#include <tuple>

#include <common.h>
#include <ray.h>
#include <shape.h>
#include <transform.h>
#include <camera.h>
#include <light.h>

namespace Caramel{
    struct Scene{
        Scene();

        void set_camera(Camera *camera);

        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) const;
        void add_mesh(const std::shared_ptr<Shape>& shape);
        void add_light(const std::shared_ptr<Light>& light);
        bool is_visible(const Vector3f &pos1, const Vector3f &pos2) const;
        std::tuple<std::shared_ptr<Light>, Float> sample_light(Sampler &sampler) const;

        void enroll_arealight(const std::shared_ptr<Shape>& shape, const std::shared_ptr<AreaLight>& arealight);

        std::vector<std::shared_ptr<Light>> m_lights;
        std::vector<std::shared_ptr<Shape>> m_meshes;
        std::unique_ptr<Camera> m_cam;
    };

    // Will be removed after scene file implemented
    Scene scene_cbox();
    Scene scene_cbox2();
    Scene scene_cbox_complex();
    Scene scene_ajax();
    Scene scene_bunny();
    Scene scene_logo1();


}
