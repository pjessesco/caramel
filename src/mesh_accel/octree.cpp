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
#include <parallel_for.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

namespace Caramel{

    // ================= Octree::Node implementation ====================

    Octree::Node::Node(const AABB &aabb) : m_aabb{aabb} {}

    void Octree::Node::construct_children(const OBJMesh &shape){
        const Vector3f &center = (m_aabb.m_max + m_aabb.m_min) * Float0_5;

        m_childs.reserve(8);
        for(int i=0;i<8;i++){
            m_childs.emplace_back(AABB(center, m_aabb.corner(i)));
        }

        for(auto ti : m_triangle_indices){
            const Triangle &tri = shape.get_triangle(ti);
            for(auto &child : m_childs){
                if(child.m_aabb.is_contain(tri.get_center())){
                    child.m_triangle_indices.emplace_back(ti);
                    break;
                }
            }
        }

        std::erase_if(m_childs, [](const Octree::Node &node){return node.m_triangle_indices.empty();});
        m_triangle_indices.clear();

        // Shrink aabb as possible
        for (auto &child : m_childs) {
            AABB shrinked_aabb = shape.get_triangle(child.m_triangle_indices[0]).get_aabb();
            for (size_t i = 1; i < child.m_triangle_indices.size(); ++i) {
                shrinked_aabb = AABB::merge(shrinked_aabb, shape.get_triangle(child.m_triangle_indices[i]).get_aabb());
            }
            child.m_aabb = shrinked_aabb;
        }
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

    std::pair<bool, RayIntersectInfo> Octree::Node::ray_intersect_leaf(const Ray &ray, Float maxt, const OBJMesh &shape) const{
        RayIntersectInfo info;
        info.t = maxt;
        bool is_hit = false;
        for(const Index i:m_triangle_indices){
            const auto &[is_intersect, tmp_info] = shape.get_triangle(i).ray_intersect(ray, info.t);
            if (is_intersect) {
                is_hit = true;
                info = tmp_info;
            }
        }
        return {is_hit, info};
    }

    std::pair<bool, RayIntersectInfo> Octree::Node::ray_intersect_branch(const Ray &ray, Float maxt, const OBJMesh &shape) const{
        RayIntersectInfo info;
        info.t = maxt;
        bool is_hit = false;

        std::array<std::pair<Index, Float>, 8> idx_mint_pair;
        Index hit_count = 0;
        for(Index i=0;i<m_childs.size();i++){
            const auto [is_child_aabb_hit, child_aabb_t] = m_childs[i].m_aabb.ray_intersect(ray, maxt);
            if(is_child_aabb_hit) {
                idx_mint_pair[hit_count++] = {i, child_aabb_t};
            }
        }

        std::sort(idx_mint_pair.begin(), idx_mint_pair.begin() + hit_count, [&](const auto &a, const auto &b){return a.second < b.second;});

        for(int i=0;i<hit_count;i++){
            const auto &[idx, mint] = idx_mint_pair[i];
            if(mint > info.t) {
                break;
            }

            const auto &[is_intersect, tmp_info] = m_childs[idx].ray_intersect(ray, info.t, shape, true);
            if (is_intersect) {
                is_hit = true;
                info = tmp_info;
            }
        }

        return {is_hit, info};
    }

    std::pair<bool, RayIntersectInfo> Octree::Node::ray_intersect(const Ray &ray, Float maxt, const OBJMesh &shape, std::optional<bool> is_intersect) const{
        const bool intersect = is_intersect.has_value() ? is_intersect.value() : m_aabb.ray_intersect(ray, maxt).first;

        if(intersect){
            if(is_leaf()){
                return ray_intersect_leaf(ray, maxt, shape);
            }
            else{
                return ray_intersect_branch(ray, maxt, shape);
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
        m_head.m_triangle_indices.reserve(m_shape.get_triangle_num());
        for(int i=0;i<m_shape.get_triangle_num();i++){
            m_head.m_triangle_indices.push_back(i);
        }

        // Construct childs recursively
        m_head.construct_children_recursively(m_shape, 0);
    }

    std::pair<bool, RayIntersectInfo> Octree::ray_intersect(const Ray &ray, Float maxt) {
        return m_head.ray_intersect(ray, maxt, m_shape, std::nullopt);
    }

}

