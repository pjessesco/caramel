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
#include <optional>

#include <common.h>
#include <aabb.h>

namespace Caramel{

    class TriangleMesh;
    class RayIntersectInfo;
    class Ray;


    // Divide a single mesh
    struct AccelerationMesh{
        friend class TriangleMesh;

        explicit AccelerationMesh(const TriangleMesh &shape) : m_shape{shape} {}
        virtual ~AccelerationMesh() = default;

        virtual void build() = 0;

        // Trace ray
        virtual std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) = 0;

        const TriangleMesh &m_shape;
    };

    struct Naive final : public AccelerationMesh{
        explicit Naive(const TriangleMesh &shape);

        void build() override;

        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) override;
    };

    // Octree for triangle meshes
    struct Octree final : public AccelerationMesh{
        explicit Octree(const TriangleMesh &shape);

        struct Node{
            Node() = default;
            explicit Node(const AABB &aabb);

            void construct_children(const TriangleMesh &shape);
            void construct_children_recursively(const TriangleMesh &shape, int depth);

            std::pair<bool, RayIntersectInfo> ray_intersect_leaf(const Ray &ray, Float maxt, const TriangleMesh &shape) const;
            std::pair<bool, RayIntersectInfo> ray_intersect_branch(const Ray &ray, Float maxt, const TriangleMesh &shape) const;
            std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt, const TriangleMesh &shape, std::optional<bool> is_intersect) const;
            bool is_leaf() const;

            AABB m_aabb;
            std::vector<Node> m_childs;
            std::vector<Index> m_triangle_indices;

        };

        void build() override;
        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) override;

        static constexpr Index MAX_DEPTH = 7;
        static constexpr Index MAX_TRIANGLE_NUM = 30;
        Node m_head;
    };

}

