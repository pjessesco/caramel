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

#pragma once

#include <vector>
#include <memory>

#include <common.h>
#include <aabb.h>

namespace Caramel{

    class Shape;
    class TriangleMesh;
    class RayIntersectInfo;
    class Ray;

    struct LinearBVHNode {
        AABB aabb;
        int offset;        // primitives_offset (leaf) or second_child_offset (inner)
        int n_primitives;  // 0 = inner node
        int split_axis;
    };

    template<typename Traits>
    struct BVHNode {
        using Primitive = typename Traits::Primitive;

        BVHNode(std::vector<Primitive> primitives, const Traits &traits);
        bool is_leaf() const;
        void create_child(const Traits &traits,
                          Float cost_traversal, Float cost_intersection, int subspace_count, int max_primitive_num);
        void flatten_recursive(std::vector<LinearBVHNode> &nodes,
                               std::vector<Primitive> &ordered_prims,
                               int &offset) const;

        AABB m_aabb;
        std::vector<Primitive> m_primitives;
        std::unique_ptr<BVHNode> m_left;
        std::unique_ptr<BVHNode> m_right;
        int m_split_axis = -1;
    };

    template<typename Traits>
    class BVHBase {
    public:
        using Primitive = typename Traits::Primitive;

        BVHBase(std::vector<Primitive> primitives, const Traits &traits,
                Float cost_traversal, Float cost_intersection, int subspace_count, int max_primitive_num);
        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const;

    private:
        Traits m_traits;
        std::vector<LinearBVHNode> m_nodes;
        std::vector<Primitive> m_ordered_primitives;
    };

    // Traits for scene-level BVH (Shape pointers)
    struct BVHSceneTraits {
        using Primitive = const Shape *;

        AABB get_aabb(Primitive p) const;
        Vector3f get_center(Primitive p) const;
        std::pair<bool, RayIntersectInfo> ray_intersect(Primitive p, const Ray &ray, Float maxt) const;
    };

    // Traits for mesh-level BVH (triangle indices)
    struct BVHMeshTraits {
        using Primitive = Index;
        explicit BVHMeshTraits(const TriangleMesh &m);

        AABB get_aabb(Primitive p) const;
        Vector3f get_center(Primitive p) const;
        std::pair<bool, RayIntersectInfo> ray_intersect(Primitive p, const Ray &ray, Float maxt) const;

        const TriangleMesh &mesh;
    };
}
