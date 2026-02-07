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
#include <iostream>
#include <iomanip>

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

    std::pair<bool, RayIntersectInfo> BVHNode::ray_intersect(const Ray &ray, Float maxt) const{
        if (is_leaf()) {
            bool is_hit = false;
            RayIntersectInfo info = RayIntersectInfo();
            info.t = maxt;

            for(int i=0;i<m_shapes.size();i++){
                auto [hit, tmp_info] = m_shapes[i]->ray_intersect(ray, info.t);
                if(hit){
                    is_hit = true;
                    info = tmp_info;
                    info.shape = m_shapes[i];
                }
            }

            return {is_hit, info};
        }
        else {
            if (!m_aabb.ray_intersect(ray, maxt).first) {
                return {false, {}};
            }
            std::pair<bool, RayIntersectInfo> ret = {false, {}};
            ret.second.t = maxt;

            const BVHNode *first  = (ray.m_d[m_split_axis] > 0) ? m_left.get() : m_right.get();
            const BVHNode *second = (ray.m_d[m_split_axis] > 0) ? m_right.get() : m_left.get();

            const auto first_hit = first->ray_intersect(ray, ret.second.t);
            if (first_hit.first) {
                ret = first_hit;
            }

            const auto second_hit = second->ray_intersect(ray, ret.second.t);
            if (second_hit.first) {
                ret = second_hit;
            }
            return ret;
        }
    }

    void BVHNode::create_child() {
        if (m_shapes.size() <= 2) {
            return;
        }

        const int longest_axis = m_aabb.longest_axis();

        // const auto get_cost = [&](int shapeCount, const auto AABB &aabb) {
        //     return shapeCount * COST_INTERSECTION +
        // }

        if constexpr(USE_SAH){
            std::array<std::pair<int/*shape count*/, AABB>, SUBSPACE_COUNT> slices;
            std::array<Float, CUT_COUNT/* = SUBSPACE_COUNT - 1*/> costs{0};

            // Divide aabb and initialize
            for (const auto &shape : m_shapes) {
                const int slice_idx = m_aabb.offset(shape->get_center())[longest_axis] * SUBSPACE_COUNT;
                const AABB shape_aabb = shape->get_aabb();

                if (slices[slice_idx].first == 0) {
                    slices[slice_idx].second = shape_aabb;
                }
                else {
                    slices[slice_idx].second = AABB::merge(slices[slice_idx].second, shape_aabb);
                }
                slices[slice_idx].first++;
            }

            // https://pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies#

            Index lower_shape_count = 0;
            AABB lower_aabb = slices[0].second;
            for (int i=0;i<CUT_COUNT;i++) {
                lower_shape_count += slices[i].first;
                if (lower_shape_count == 0) {
                    continue;
                }
                lower_aabb = AABB::merge(lower_aabb, slices[i].second);
                costs[i] = lower_aabb.surface_area() * lower_shape_count;
            }

            Index upper_shape_count = 0;
            AABB upper_aabb = slices[CUT_COUNT].second;
            for (int i=CUT_COUNT;i>=1;i--) {
                upper_shape_count += slices[i].first;
                if (upper_shape_count == 0) {
                    continue;
                }
                upper_aabb = AABB::merge(upper_aabb, slices[i].second);
                costs[i-1] += upper_aabb.surface_area() * upper_shape_count;
            }

            // Find cut index with the lowest cost
            Index lowest_cost_cut_index = 0;
            Float lowest_cost = INF;
            for (int i=0;i<CUT_COUNT;i++) {
                if (lowest_cost > costs[i]) {
                    lowest_cost_cut_index = i;
                    lowest_cost = costs[i];
                }
            }

            const Float total_cost = COST_TRAVERSAL + lowest_cost / m_aabb.surface_area();

            if (m_shapes.size() > MAX_SHAPE_NUM || total_cost < m_shapes.size() * COST_INTERSECTION) {
                const auto mid = std::partition(m_shapes.begin(), m_shapes.end(),
                                          [=, this](const auto &shape) {
                                              const int slice_idx = m_aabb.offset(shape->get_center())[longest_axis] * SUBSPACE_COUNT;
                                              return slice_idx <= lowest_cost_cut_index;
                                          });

                if (mid == m_shapes.begin() || mid == m_shapes.end()) {
                    return;
                }

                std::vector<const Shape*> left(mid - m_shapes.begin());
                std::vector<const Shape*> right(m_shapes.end() - mid);
                std::move(m_shapes.begin(), mid, left.begin());
                std::move(mid, m_shapes.end(), right.begin());
                m_shapes.clear();

                m_split_axis = longest_axis;
                m_left = std::make_unique<BVHNode>(left);
                m_right = std::make_unique<BVHNode>(right);
                m_left->create_child();
                m_right->create_child();
            }
        }

        else {
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
            m_split_axis = longest_axis;
            m_shapes.clear();
            m_left->create_child();
            m_right->create_child();
        }

    }

    // void BVHNode::collect_stats(Stats &stats, int depth) const {
    //     stats.total_nodes++;
    //     if (is_leaf()) {
    //         stats.leaf_nodes++;
    //         const int n = static_cast<int>(m_shapes.size());
    //         stats.min_shapes_per_leaf = std::min(stats.min_shapes_per_leaf, n);
    //         stats.max_shapes_per_leaf = std::max(stats.max_shapes_per_leaf, n);
    //         stats.total_shapes_in_leaves += n;
    //         stats.max_depth = std::max(stats.max_depth, depth);
    //     } else {
    //         stats.inner_nodes++;
    //         m_left->collect_stats(stats, depth + 1);
    //         m_right->collect_stats(stats, depth + 1);
    //     }
    // }

    // void BVHNode::print_tree(std::string prefix, bool is_left, int depth, int max_depth) const {
    //     if (depth > max_depth) return;
    //
    //     std::cout << prefix;
    //     std::cout << (depth == 0 ? "" : (is_left ? "├─L " : "└─R "));
    //
    //     if (is_leaf()) {
    //         std::cout << "Leaf [" << m_shapes.size() << " shapes] ";
    //     } else {
    //         std::cout << "Inner ";
    //     }
    //     std::cout << "AABB("
    //               << std::fixed << std::setprecision(2)
    //               << m_aabb.m_min[0] << "," << m_aabb.m_min[1] << "," << m_aabb.m_min[2]
    //               << " ~ "
    //               << m_aabb.m_max[0] << "," << m_aabb.m_max[1] << "," << m_aabb.m_max[2]
    //               << ") SA=" << std::setprecision(2) << m_aabb.surface_area()
    //               << "\n";
    //
    //     if (!is_leaf() && depth < max_depth) {
    //         std::string child_prefix = prefix + (depth == 0 ? "" : (is_left ? "│   " : "    "));
    //         m_left->print_tree(child_prefix, true, depth + 1, max_depth);
    //         m_right->print_tree(child_prefix, false, depth + 1, max_depth);
    //     }
    // }
    //
    // void BVHNode::print_stats() const {
    //     Stats stats;
    //     collect_stats(stats, 0);
    //
    //     std::cout << "\n====== BVH Statistics ======\n";
    //     std::cout << "Total nodes:          " << stats.total_nodes << "\n";
    //     std::cout << "  Inner nodes:        " << stats.inner_nodes << "\n";
    //     std::cout << "  Leaf nodes:         " << stats.leaf_nodes << "\n";
    //     std::cout << "Max depth:            " << stats.max_depth << "\n";
    //     std::cout << "Shapes per leaf:\n";
    //     std::cout << "  Min:                " << stats.min_shapes_per_leaf << "\n";
    //     std::cout << "  Max:                " << stats.max_shapes_per_leaf << "\n";
    //     std::cout << "  Avg:                " << std::fixed << std::setprecision(1)
    //               << static_cast<float>(stats.total_shapes_in_leaves) / stats.leaf_nodes << "\n";
    //     std::cout << "Root AABB:            ("
    //               << std::setprecision(3)
    //               << m_aabb.m_min[0] << ", " << m_aabb.m_min[1] << ", " << m_aabb.m_min[2]
    //               << ") ~ ("
    //               << m_aabb.m_max[0] << ", " << m_aabb.m_max[1] << ", " << m_aabb.m_max[2]
    //               << ")\n";
    //     std::cout << "Root surface area:    " << std::setprecision(2) << m_aabb.surface_area() << "\n";
    //
    //     constexpr int PRINT_DEPTH = 4;
    //     std::cout << "\n--- Tree (depth <= " << PRINT_DEPTH << ") ---\n";
    //     print_tree("", true, 0, PRINT_DEPTH);
    //     std::cout << "============================\n\n";
    // }

}

