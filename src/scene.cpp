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

#include <tuple>

#include <scene.h>

#include <aabb.h>
#include <bvh_base.h>
#include <camera.h>
#include <common.h>
#include <light.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <sampler.h>
#include <shape.h>
#include <scene_accel.h>

namespace Caramel{

    Scene::Scene()
    : m_cam{nullptr}, m_aabb{vec3f_zero, vec3f_zero}, m_sceneRadius{0.0f}, m_sceneCenterPos{vec3f_zero}, m_envmap_light{nullptr} {}

    void Scene::set_camera(Camera *camera) {
        m_cam = camera;
    }

    std::pair<bool, RayIntersectInfo> Scene::ray_intersect(const Ray &ray, Float maxt) const{
        return m_accel->ray_intersect(ray, maxt);
    }

    void Scene::add_mesh_and_arealight(const Shape *shape){
        m_meshes.emplace_back(shape);
        if(shape->is_light()){
            m_lights.push_back(shape->get_arealight());
        }

        m_aabb = m_meshes.empty() ? shape->get_aabb() :
                                    AABB::merge(m_aabb, shape->get_aabb());
        m_sceneCenterPos = (m_aabb.m_max + m_aabb.m_min) * 0.5f;
        m_sceneRadius = Vector3f::L2(m_sceneCenterPos, m_aabb.m_max);
    }

    void Scene::add_light(Light *light){
        m_lights.push_back(light);
        if (light->is_envlight()) {
            m_envmap_light = light;
            m_envmap_light->set_scene_radius(m_sceneRadius);
        }
    }

    // Similar with `RayIntersectInfo::recursive_ray_to()`
    bool Scene::is_visible(const Vector3f &pos1, const Vector3f &pos2) const{
        const Vector3f vec3_1_to_2(pos2 - pos1);
        const Vector3f dir = vec3_1_to_2.normalize();
        const Float len = vec3_1_to_2.length();
        const Ray ray{pos1 + (dir * EPSILON), dir};

        const auto [is_hit, info] = ray_intersect(ray, len);
        return !is_hit || std::abs(len - info.t) <= (EPSILON * static_cast<Float>(1.1));
    }

    std::pair<const Light*, Float> Scene::sample_light(Sampler &sampler) const{
        const Index idx = m_lights_pdf.sample(sampler.sample_1d());
        return {m_lights[idx], m_lights_pdf.pdf(idx)};
    }

    Float Scene::pdf_light(const Light *light) const{
        return m_lights_pdf.pdf(m_light_idx_map.at(light));
    }

    void Scene::build_accel() {
        m_accel = new BVHScene();
        m_accel->build(m_meshes);
    }

    void Scene::build_light_pdf() {
        std::vector<Float> light_power;
        light_power.reserve(m_lights.size());

        int i = 0;
        for (auto p : m_lights) {
            m_light_idx_map[p] = i;
            i++;
            light_power.push_back(p->power());
        }
        m_lights_pdf = Distrib1D(light_power);
    }
}
