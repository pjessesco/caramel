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

#include <scene.h>

#include <common.h>
#include <ray.h>
#include <shape.h>
#include <camera.h>
#include <light.h>
#include <rayintersectinfo.h>
#include <sampler.h>

namespace Caramel{

    Scene::Scene() : m_cam{nullptr} {}

    void Scene::set_camera(Camera *camera) {
        m_cam = camera;
    }

    std::tuple<bool, RayIntersectInfo> Scene::ray_intersect(const Ray &ray) const{

        bool is_hit = false;
        RayIntersectInfo info = RayIntersectInfo();

        for(int i=0;i<m_meshes.size();i++){
            if(get<1>(m_meshes[i]->get_aabb().ray_intersect(ray)) <= info.t){
                auto [hit, tmp_info] = m_meshes[i]->ray_intersect(ray);
                if(hit){
                    is_hit = true;
                    if(info.t >= tmp_info.t){
                        info = tmp_info;
                        info.idx = i;
                    }
                }
            }
        }

        return {is_hit, info};
    }

    void Scene::add_mesh_and_arealight(const Shape *shape){
        m_meshes.emplace_back(shape);
        if(shape->is_light()){
            m_lights.push_back(shape->get_arealight());
        }
    }

    void Scene::add_light(const Light *light){
        m_lights.push_back(light);
    }

    // Similar with `RayIntersectInfo::recursive_ray_to()`
    std::pair<bool, RayIntersectInfo> Scene::is_visible(const Vector3f &pos1, const Vector3f &pos2) const{
        const Vector3f vec3_1_to_2(pos2 - pos1);
        const Vector3f dir = vec3_1_to_2.normalize();
        const Ray ray{pos1 + (dir * 1e-3), dir};

        const auto [is_hit, info] = ray_intersect(ray);

        if(!is_hit){
            return {false, RayIntersectInfo()};
        }

        return {std::abs(vec3_1_to_2.length() - info.t) <= 1.1e-3, info};
    }

    std::tuple<const Light*, Float> Scene::sample_light(Sampler &sampler) const{
        const Index idx = static_cast<Index>(sampler.sample_1d() * static_cast<Float>(m_lights.size()));
        return {m_lights[idx], Float1 / static_cast<Float>(m_lights.size())};
    }

}
