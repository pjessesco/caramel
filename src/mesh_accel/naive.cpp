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

#include <vector>

#include <acceleration.h>

#include <aabb.h>
#include <common.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

namespace Caramel{
    Naive::Naive(const OBJMesh &shape) : AccelerationMesh(shape) {}

    void Naive::build() {}

    std::tuple<bool, RayIntersectInfo> Naive::ray_intersect(const Ray &ray) {
        if(!(std::get<0>(m_shape.get_aabb().ray_intersect(ray)))){
            return {false, RayIntersectInfo()};
        }

        RayIntersectInfo info = RayIntersectInfo();
        bool is_hit = false;

        for (int i = 0; i < m_shape.get_triangle_num(); i++) {
            auto [is_intersect, tmp_info] = m_shape.get_triangle(i).ray_intersect(ray);
            if (is_intersect) {
                is_hit = true;
                if (info.t > tmp_info.t) {
                    info = tmp_info;
                }
            }
        }

        return {is_hit, info};
    }
}

