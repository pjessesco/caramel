//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2026 Jino Park
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

#include <aabb.h>
#include <common.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

#include <bvh_base.h>

namespace Caramel{

    // ---- BVHNode ----

    template<typename Traits>
    BVHNode<Traits>::BVHNode(std::vector<Primitive> primitives, const Traits &traits) {
        m_primitives = std::move(primitives);
        m_left = nullptr;
        m_right = nullptr;
        m_aabb = traits.get_aabb(m_primitives[0]);
        for (int i=1;i<m_primitives.size();i++) {
            m_aabb = AABB::merge(m_aabb, traits.get_aabb(m_primitives[i]));
        }
    }

    template<typename Traits>
    bool BVHNode<Traits>::is_leaf() const {
         return !m_primitives.empty();
     }


    template<typename Traits>
    void BVHNode<Traits>::create_child(const Traits &traits,
                                       Float cost_traversal,
                                       Float cost_intersection,
                                       int subspace_count,
                                       int max_primitive_num) {
        if (m_primitives.size() <= 2) {
            return;
        }

        const int longest_axis = m_aabb.longest_axis();
        const int cut_count = subspace_count - 1;

        std::vector<std::pair<int/*primitive count*/, AABB>> slices(subspace_count);
        std::vector<Float> costs(cut_count, Float0);

        // Divide aabb and initialize
        for (const auto &prim : m_primitives) {
            const int slice_idx = std::min(static_cast<int>(m_aabb.offset(traits.get_center(prim))[longest_axis] * subspace_count),
                                           subspace_count - 1);
            const AABB prim_aabb = traits.get_aabb(prim);

            if (slices[slice_idx].first == 0) {
                slices[slice_idx].second = prim_aabb;
            }
            else {
                slices[slice_idx].second = AABB::merge(slices[slice_idx].second, prim_aabb);
            }
            slices[slice_idx].first++;
        }

        // https://pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies#

        Index lower_count = 0;
        AABB lower_aabb = slices[0].second;
        for (int i=0;i<cut_count;i++) {
            lower_count += slices[i].first;
            if (lower_count == 0) {
                continue;
            }
            lower_aabb = AABB::merge(lower_aabb, slices[i].second);
            costs[i] = lower_aabb.surface_area() * lower_count;
        }

        Index upper_count = 0;
        AABB upper_aabb = slices[cut_count].second;
        for (int i=cut_count;i>=1;i--) {
            upper_count += slices[i].first;
            if (upper_count == 0) {
                continue;
            }
            upper_aabb = AABB::merge(upper_aabb, slices[i].second);
            costs[i-1] += upper_aabb.surface_area() * upper_count;
        }

        // Find cut index with the lowest cost
        Index lowest_cost_cut_index = 0;
        Float lowest_cost = INF;
        for (int i=0;i<cut_count;i++) {
            if (lowest_cost > costs[i]) {
                lowest_cost_cut_index = i;
                lowest_cost = costs[i];
            }
        }

        const Float total_cost = cost_traversal + lowest_cost / m_aabb.surface_area();

        if (static_cast<int>(m_primitives.size()) > max_primitive_num || total_cost < m_primitives.size() * cost_intersection) {
            const auto mid = std::partition(m_primitives.begin(), m_primitives.end(),
                                      [=, this](const auto &prim) {
                                          const int slice_idx = std::min(static_cast<int>(m_aabb.offset(traits.get_center(prim))[longest_axis] * subspace_count),
                                                                         subspace_count - 1);
                                          return slice_idx <= lowest_cost_cut_index;
                                      });

            if (mid == m_primitives.begin() || mid == m_primitives.end()) {
                return;
            }

            std::vector<Primitive> left(mid - m_primitives.begin());
            std::vector<Primitive> right(m_primitives.end() - mid);
            std::move(m_primitives.begin(), mid, left.begin());
            std::move(mid, m_primitives.end(), right.begin());
            m_primitives.clear();

            m_split_axis = longest_axis;
            m_left = std::make_unique<BVHNode>(std::move(left), traits);
            m_right = std::make_unique<BVHNode>(std::move(right), traits);
            m_left->create_child(traits, cost_traversal, cost_intersection, subspace_count, max_primitive_num);
            m_right->create_child(traits, cost_traversal, cost_intersection, subspace_count, max_primitive_num);
        }
    }

    template<typename Traits>
    void BVHNode<Traits>::flatten_recursive(std::vector<LinearBVHNode> &nodes,
                                            std::vector<Primitive> &ordered_prims,
                                            int &offset) const {
        const int my_offset = offset;
        offset += 1;
        nodes.emplace_back();

        if (is_leaf()) {
            nodes[my_offset].aabb = m_aabb;
            nodes[my_offset].n_primitives = static_cast<int>(m_primitives.size());
            nodes[my_offset].offset = static_cast<int>(ordered_prims.size());
            nodes[my_offset].split_axis = -1;
            for (const auto &p : m_primitives) {
                ordered_prims.emplace_back(p);
            }
        }
        else {
            nodes[my_offset].aabb = m_aabb;
            nodes[my_offset].split_axis = m_split_axis;
            nodes[my_offset].n_primitives = 0;
            m_left->flatten_recursive(nodes, ordered_prims, offset);
            nodes[my_offset].offset = offset;
            m_right->flatten_recursive(nodes, ordered_prims, offset);
        }
    }

    // ---- BVHTree ----

    template<typename Traits>
    BVHTree<Traits>::BVHTree(std::vector<Primitive> primitives, const Traits &traits,
                             Float cost_traversal, Float cost_intersection, int subspace_count, int max_primitive_num)
    : m_traits{traits} {
        BVHNode<Traits> root(std::move(primitives), traits);
        root.create_child(traits, cost_traversal, cost_intersection, subspace_count, max_primitive_num);
        int offset = 0;
        root.flatten_recursive(m_nodes, m_ordered_primitives, offset);
    }

    template<typename Traits>
    std::pair<bool, RayIntersectInfo> BVHTree<Traits>::ray_intersect(const Ray &ray, Float maxt) const{
        bool is_hit = false;
        RayIntersectInfo info;
        info.t = maxt;

        int to_visit_stack[64];
        int to_visit_offset = 0;
        int current = 0;

        while (true) {
            const LinearBVHNode &node = m_nodes[current];

            if (node.aabb.ray_intersect(ray, info.t).first) {
                if (node.n_primitives > 0) {
                    // Leaf: test primitives
                    for (int i = 0; i < node.n_primitives; i++) {
                        auto [hit, tmp_info] = m_traits.ray_intersect(m_ordered_primitives[node.offset + i], ray, info.t);
                        if (hit) {
                            is_hit = true;
                            info = tmp_info;
                        }
                    }
                    if (to_visit_offset == 0) {
                        break;
                    }
                    to_visit_offset -= 1;
                    current = to_visit_stack[to_visit_offset];
                }
                else {
                    // Inner: visit children in order
                    if (ray.m_d[node.split_axis] > 0) {
                        to_visit_stack[to_visit_offset] = node.offset;
                        to_visit_offset += 1;
                        current = current + 1;
                    }
                    else {
                        to_visit_stack[to_visit_offset] = current + 1;
                        to_visit_offset += 1;
                        current = node.offset;
                    }
                }
            }
            else {
                if (to_visit_offset == 0) {
                    break;
                }
                to_visit_offset -= 1;
                current = to_visit_stack[to_visit_offset];
            }
        }

        return {is_hit, info};
    }

    // ---- Traits ----

    AABB BVHSceneTraits::get_aabb(const Shape *s) const {
        return s->get_aabb();
    }

    Vector3f BVHSceneTraits::get_center(const Shape *s) const {
        return s->get_center();
    }

    BVHMeshTraits::BVHMeshTraits(const TriangleMesh &m) : mesh(m) {}

    AABB BVHMeshTraits::get_aabb(Index i) const {
        return mesh.get_triangle_aabb(i);
    }

    Vector3f BVHMeshTraits::get_center(Index i) const {
        return mesh.get_triangle_aabb(i).get_center();
    }

    std::pair<bool, RayIntersectInfo> BVHMeshTraits::ray_intersect(Index i, const Ray &ray, Float maxt) const {
        return mesh.get_triangle_ray_intersect(i, ray, maxt);
    }

    template struct BVHNode<BVHSceneTraits>;
    template struct BVHNode<BVHMeshTraits>;
    template class BVHTree<BVHSceneTraits>;
    template class BVHTree<BVHMeshTraits>;
}
