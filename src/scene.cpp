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

#include <memory>
#include <tuple>

#include <common.h>
#include <ray.h>
#include <shape.h>
#include <camera.h>
#include <light.h>
#include <scene.h>

namespace Caramel{

    Scene::Scene(const Camera &cam) : m_cam{cam} {}

    std::tuple<bool, RayIntersectInfo> Scene::ray_intersect(const Ray &ray) const{

        bool is_hit = false;
        RayIntersectInfo info = RayIntersectInfo();

        for(int i=0;i<m_meshes.size();i++){
            auto [hit, tmp_info] = m_meshes[i]->ray_intersect(ray);
            if(hit){
                is_hit = true;
                if(info.t > tmp_info.t){
                    info = tmp_info;
                    info.idx = i;
                }
            }
        }

        return {is_hit, info};
    }

    void Scene::add_mesh(const std::shared_ptr<Shape>& shape){
        m_meshes.emplace_back(shape);
    }

    void Scene::add_light(const std::shared_ptr<Light>& light){
        m_lights.emplace_back(light);
    }

    bool Scene::is_visible(const Vector3f pos1, const Vector3f &pos2) const{
        const Ray ray{pos1, pos2 - pos1};
        const auto [is_hit, info] = ray_intersect(ray);

        if(!is_hit){
            return false;
        }

        const Vector3f dist = pos2 - pos1;

        return dist.length() - info.t <= EPSILON;
    }

    std::tuple<std::shared_ptr<Light>, Float> Scene::sample_light(Sampler &sampler) const{
        const Index idx = static_cast<Index>(sampler.sample_1d() * static_cast<Float>(m_lights.size()));
        return {m_lights[idx], Float1 / static_cast<Float>(m_lights.size())};
    }

    void Scene::enroll_arealight(const std::shared_ptr<Shape>& shape, const std::shared_ptr<AreaLight>& arealight){
        arealight->m_shape = shape;
        shape->m_arealight = arealight;
    }

}