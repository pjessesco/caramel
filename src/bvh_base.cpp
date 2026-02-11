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

#include <aabb.h>
#include <common.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

#include <bvh_base.h>

namespace Caramel{
    template<typename Traits>
    BVHBase<Traits>::BVHBase(std::vector<Primitive> primitives, const Traits &traits)
    : m_traits{traits} {
        m_primitives = std::move(primitives);
        m_left = nullptr;
        m_right = nullptr;
        m_aabb = m_traits.get_aabb(m_primitives[0]);
        for (int i=1;i<m_primitives.size();i++) {
            m_aabb = AABB::merge(m_aabb, m_traits.get_aabb(m_primitives[i]));
        }
    }

    template<typename Traits>
    bool BVHBase<Traits>::is_leaf() const {
         return !m_primitives.empty();
     }

    template<typename Traits>
    std::pair<bool, RayIntersectInfo> BVHBase<Traits>::ray_intersect(const Ray &ray, Float maxt) const{
        if (is_leaf()) {
            bool is_hit = false;
            RayIntersectInfo info = RayIntersectInfo();
            info.t = maxt;

            for(int i=0;i<m_primitives.size();i++){
                auto [hit, tmp_info] = m_traits.ray_intersect(m_primitives[i], ray, info.t);
                if(hit){
                    is_hit = true;
                    info = tmp_info;
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

            const BVHBase *first  = (ray.m_d[m_split_axis] > 0) ? m_left.get() : m_right.get();
            const BVHBase *second = (ray.m_d[m_split_axis] > 0) ? m_right.get() : m_left.get();

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

    template<typename Traits>
    void BVHBase<Traits>::create_child() {
        if (m_primitives.size() <= 2) {
            return;
        }

        const int longest_axis = m_aabb.longest_axis();
        const int subspace_count = BVH_SUBSPACE_COUNT;
        const int cut_count = subspace_count - 1;

        if constexpr(USE_SAH) {
            std::vector<std::pair<int/*primitive count*/, AABB>> slices(subspace_count);
            std::vector<Float> costs(cut_count, Float0);

            // Divide aabb and initialize
            for (const auto &prim : m_primitives) {
                const int slice_idx = std::min(static_cast<int>(m_aabb.offset(m_traits.get_center(prim))[longest_axis] * subspace_count),
                                               subspace_count - 1);
                const AABB prim_aabb = m_traits.get_aabb(prim);

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

            const Float total_cost = COST_TRAVERSAL + lowest_cost / m_aabb.surface_area();

            if (m_primitives.size() > BVH_MAX_PRIMITIVE_NUM || total_cost < m_primitives.size() * COST_INTERSECTION) {
                const auto mid = std::partition(m_primitives.begin(), m_primitives.end(),
                                          [=, this](const auto &prim) {
                                              const int slice_idx = std::min(static_cast<int>(m_aabb.offset(m_traits.get_center(prim))[longest_axis] * subspace_count),
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
                m_left = std::make_unique<BVHBase>(std::move(left), m_traits);
                m_right = std::make_unique<BVHBase>(std::move(right), m_traits);
                m_left->create_child();
                m_right->create_child();
            }
        }

        else {
            const Float longest_axis_mid = (m_aabb.m_min[longest_axis] + m_aabb.m_max[longest_axis]) * Float0_5;

            std::vector<Primitive> left;
            std::vector<Primitive> right;

            for (auto &p : m_primitives) {
                ((m_traits.get_center(p)[longest_axis] < longest_axis_mid) ? left : right).push_back(p);
            }

            if (left.empty() || right.empty()) {
                return;
            }

            m_left = std::make_unique<BVHBase>(std::move(left), m_traits);
            m_right = std::make_unique<BVHBase>(std::move(right), m_traits);
            m_split_axis = longest_axis;
            m_primitives.clear();
            m_left->create_child();
            m_right->create_child();
        }
    }
}

