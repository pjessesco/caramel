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

#include <vector>

#include <acceleration.h>

#include <aabb.h>
#include <common.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

#include <bvh.h>

namespace Caramel{

     BVHNode::BVHNode(const std::vector<const Shape*> &shapes) {
        m_shapes = shapes;
        m_left = nullptr;
        m_right = nullptr;
        m_aabb = shapes[0]->get_aabb();
        for (int i=1;i<shapes.size();i++) {
            m_aabb = AABB::merge(m_aabb, shapes[i]->get_aabb());
        }
    }

    bool BVHNode::is_leaf() const {
         return !m_shapes.empty();
     }

    std::pair<bool, RayIntersectInfo> BVHNode::ray_intersect(const Ray &ray) {
        if (is_leaf()) {
            bool is_hit = false;
            RayIntersectInfo info = RayIntersectInfo();

            for(int i=0;i<m_shapes.size();i++){
                if(get<1>(m_shapes[i]->get_aabb().ray_intersect(ray)) <= info.t){
                    auto [hit, tmp_info] = m_shapes[i]->ray_intersect(ray);
                    if(hit){
                        is_hit = true;
                        if(info.t >= tmp_info.t){
                            info = tmp_info;
                            info.shape = m_shapes[i];
                        }
                    }
                }
            }

            return {is_hit, info};
        }
        else {
            if (!std::get<0>(m_aabb.ray_intersect(ray))) {
                return {false, {}};
            }
            std::pair<bool, RayIntersectInfo> ret = {false, {}};

            const auto left_hit = m_left->ray_intersect(ray);
            if (left_hit.first) {
                ret = left_hit;
            }

            const auto right_hit = m_right->ray_intersect(ray);
            if (right_hit.first && right_hit.second.t < ret.second.t) {
                ret = right_hit;
            }
            return ret;
        }
    }

    void BVHNode::create_child() {
        if (m_shapes.size() <= 2) {
            return;
        }

        const int longest_axis = m_aabb.longest_axis();
        const Float longest_axis_mid = (m_aabb.m_min[longest_axis] + m_aabb.m_max[longest_axis]) * Float0_5;

        std::vector<const Shape*> left;
        std::vector<const Shape*> right;

        for (auto &s : m_shapes) {
            ((s->get_center()[longest_axis] < longest_axis_mid) ? left : right).push_back(s);
        }

         if (left.empty() || right.empty()) {
             return;
         }
        m_left = std::make_unique<BVHNode>(left);
        m_right = std::make_unique<BVHNode>(right);
        m_shapes.clear();
        m_left->create_child();
        m_right->create_child();
    }

}

