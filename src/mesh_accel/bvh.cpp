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

     BVHNode::BVHNode(const std::vector<Shape*> &shapes) {
        m_shapes = shapes;
        m_left = nullptr;
        m_right = nullptr;
        m_aabb = shapes[0]->get_aabb();
        for (int i=1;i<shapes.size();i++) {
            m_aabb = AABB::merge(m_aabb, shapes[i]->get_aabb());
        }
    }

    std::pair<bool, RayIntersectInfo> BVHNode::ray_intersect(const Ray &ray) {
        if (std::get<0>(m_aabb.ray_intersect(ray))) {
            return {false, {}};
        }
        std::pair<bool, RayIntersectInfo> ret = {false, {}};

        if (auto tmp = m_left->m_aabb.ray_intersect(ray); std::get<0>(tmp)) {
            if (std::get<1>(ret).t > std::get<1>(tmp)) {
                ret = {true, }
            }
        }
        if (std::get<0>(m_right->m_aabb.ray_intersect(ray))) {

        }

    }

    void BVHNode::create_child() {
        if (m_shapes.size() < 4) {
            return;
        }

        const int longest_axis = m_aabb.longest_axis();
        const Float longest_axis_mid = (m_aabb.m_min[longest_axis] + m_aabb.m_max[longest_axis]) * Float0_5;

        std::vector<Shape*> left;
        std::vector<Shape*> right;

        AABB left_aabb = m_aabb;
        AABB right_aabb = m_aabb;

        for (auto &s : m_shapes) {
            ((s->get_center()[longest_axis] < longest_axis_mid) ? left : right).push_back(s);
        }

        m_left = new BVHNode(left);
        m_right = new BVHNode(right);
        m_shapes.clear();
        m_left->create_child();
        m_right->create_child();
    }

}

