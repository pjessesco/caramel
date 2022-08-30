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

namespace Caramel{
    struct Scene{
        Scene(const Camera &cam) : m_cam{cam} {}

        //       is_hit,     u,     v,     t,   idx
        std::tuple<bool, Float, Float, Float, Index> ray_intersect(const Ray &ray) const{

            //       is_hit,     u,     v,     t
            std::tuple<bool, Float, Float, Float> result = {false, 0, 0, 0};
            Index mesh_idx = 999999;

            for(int i=0;i<m_meshes.size();i++){
                auto tmp = m_meshes[i]->ray_intersect(ray);
                if(get<0>(tmp)){
                    mesh_idx = i;
                    result = tmp;
                }
            }

            return std::tuple_cat(result, std::make_tuple(mesh_idx));
        }

        void add_mesh(Shape *shape){
            m_meshes.emplace_back(std::shared_ptr<Shape>(shape));
        }


        std::vector<std::shared_ptr<Shape>> m_meshes;
        Camera m_cam;
    };
}
