//
// Created by Jino Park on 2022/08/31.
//

#include <vector>

#include <common.h>
#include <shape.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <aabb.h>

namespace Caramel{

    // ================= Naive implementation ====================

    Naive::Naive(const OBJMesh &shape) : AccelerationMesh(shape) {}

    void Naive::build() {}

    std::tuple<bool, RayIntersectInfo> Naive::ray_intersect(const Ray &ray) {
        if(!(m_shape.get_aabb().ray_intersect(ray))){
            return {false, RayIntersectInfo()};
        }

        RayIntersectInfo info = RayIntersectInfo();
        info.t = INF;
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

    // ================= Octree::Node implementation ====================

    Octree::Node::Node(const AABB &aabb) : m_aabb{aabb} {}

    void Octree::Node::construct_children(const OBJMesh &shape){
        const Vector3f center = (m_aabb.m_max + m_aabb.m_min) * 0.5f;

        for(int i=0;i<8;i++){
            m_childs.emplace_back(AABB(center, m_aabb.corner(i)));
        }

        for(auto ti : m_triangle_indices){
            const Triangle tri = shape.get_triangle(ti);
            for(int i=0;i<8;i++){
                // can be replaced to tri.is_overlab, not using aabb
                if(m_childs[i].m_aabb.is_overlap(tri)){
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

        if(m_triangle_indices.size() > MAX_TRIANGLE_NUM){
            construct_children(shape);
        }
        for(auto c : m_childs){
            construct_children_recursively(shape, depth + 1);
        }
    }

    std::tuple<bool, RayIntersectInfo> Octree::Node::ray_intersect_leaf(const Ray &ray, const OBJMesh &shape){
        RayIntersectInfo info;
        info.t = INF;
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
        info.t = INF;
        bool is_hit = false;
        for(auto c : m_childs){
            auto [is_intersect, tmp_info] = c.ray_intersect(ray, shape);
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
        if(m_aabb.ray_intersect(ray)){
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
        return !m_triangle_indices.empty() && m_childs.empty();
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
