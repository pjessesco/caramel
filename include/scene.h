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

#include <vector>

#include <distribution.h>
#include <common.h>
#include <aabb.h>

namespace Caramel{
    class Camera;
    class Ray;
    class RayIntersectInfo;
    class Shape;
    class Light;
    class Sampler;
    class ConstantEnvLight;
    class SceneAccel;

    class Scene{
    public:
        Scene();

        void set_camera(Camera *camera);

        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt=INF) const;
        void add_mesh_and_arealight(const Shape *shape);
        void add_light(Light *light);
        bool is_visible(const Vector3f &pos1, const Vector3f &pos2) const;
        std::pair<const Light*, Float> sample_light(Sampler &sampler) const;
        Float pdf_light(const Light *light) const;
        void build_accel();
        void build_light_pdf();

        std::vector<const Light*> m_lights;
        Distrib1D m_lights_pdf;
        std::unordered_map<const Light*, Index> m_light_idx_map;

        Light* m_envmap_light;
        std::vector<const Shape*> m_meshes;
        Vector3f m_sceneCenterPos;
        Float m_sceneRadius;
        AABB m_aabb;
        const Camera *m_cam;
        SceneAccel *m_accel;

    };
}
