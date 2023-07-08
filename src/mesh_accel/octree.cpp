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
#include <parallel_for.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

namespace Caramel{

    // ================= Octree::Node implementation ====================

    Octree::Node::Node(const AABB &aabb) : m_aabb{aabb} {}

    void Octree::Node::construct_children(const OBJMesh &shape){
        const Vector3f center = (m_aabb.m_max + m_aabb.m_min) * Float0_5;

        for(int i=0;i<8;i++){
            m_childs.emplace_back(AABB(center, m_aabb.corner(i)));
        }

        for(auto ti : m_triangle_indices){
            const Triangle tri = shape.get_triangle(ti);
            for(int i=0;i<8;i++){
                // can be replaced to tri.is_overlab, not using aabb
                if(m_childs[i].m_aabb.is_overlap(tri.get_aabb())){
                    m_childs[i].m_triangle_indices.push_back(ti);
                }
            }
        }

        m_triangle_indices.clear();
    }

    void Octree::Node::construct_children_recursively(const OBJMesh &shape, int depth){
        if(depth > MAX_DEPTH){
            return;
        }

        construct_children(shape);

        if(depth == 0 && m_childs.size() == 8){
            parallel_for(0, 8, [&](int i){
                m_childs[i].construct_children_recursively(shape, depth + 1);
            });
        }
        else{
            for(auto &c : m_childs){
                if(c.m_triangle_indices.size() > MAX_TRIANGLE_NUM){
                    c.construct_children_recursively(shape, depth + 1);
                }
            }
        }
    }

    std::tuple<bool, RayIntersectInfo> Octree::Node::ray_intersect_leaf(const Ray &ray, const OBJMesh &shape){
        RayIntersectInfo info;
        bool is_hit = false;
        for(Index i:m_triangle_indices){
            auto [is_intersect, tmp_info] = shape.get_triangle(i).ray_intersect(ray);
            if (is_intersect) {
                is_hit = true;
                if (info.t > tmp_info.t) {
                    info = tmp_info;
                }
            }
        }
        return {is_hit, info};
    }

    std::tuple<bool, RayIntersectInfo> Octree::Node::ray_intersect_branch(const Ray &ray, const OBJMesh &shape){
        RayIntersectInfo info;
        bool is_hit = false;

        for(int i=0;i<8;i++){
            auto [is_intersect, tmp_info] = m_childs[i].ray_intersect(ray, shape);
            if (is_intersect) {
                is_hit = true;
                if (info.t > tmp_info.t) {
                    info = tmp_info;
                }
            }
        }

        return {is_hit, info};
    }

    std::tuple<bool, RayIntersectInfo> Octree::Node::ray_intersect(const Ray &ray, const OBJMesh &shape){
        if(std::get<0>(m_aabb.ray_intersect(ray))){
            if(is_leaf()){
                return ray_intersect_leaf(ray, shape);
            }
            else{
                return ray_intersect_branch(ray, shape);
            }
        }
        else{
            return {false, RayIntersectInfo()};
        }
    }

    // ================= Octree implementation ====================

    Octree::Octree(const OBJMesh &shape) : AccelerationMesh(shape) {}

    bool Octree::Node::is_leaf() const{
        return m_childs.empty();
    }

    void Octree::build(){
        // Construct head
        m_head = Node(m_shape.get_aabb());
        for(int i=0;i<m_shape.get_triangle_num();i++){
            m_head.m_triangle_indices.push_back(i);
        }

        // Construct childs recursively
        m_head.construct_children_recursively(m_shape, 0);
    }

    std::tuple<bool, RayIntersectInfo> Octree::ray_intersect(const Ray &ray) {
        return m_head.ray_intersect(ray, m_shape);
    }

}

