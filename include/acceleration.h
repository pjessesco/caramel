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

#pragma once

#include <vector>

#include <common.h>
#include <aabb.h>

namespace Caramel{

    class OBJMesh;
    class RayIntersectInfo;
    class Ray;


    // Divide a single mesh
    struct AccelerationMesh{
        friend struct OBJMesh;

        explicit AccelerationMesh(const OBJMesh &shape) : m_shape{shape} {}
        virtual ~AccelerationMesh() = default;

        virtual void build() = 0;

        // Trace ray
        virtual std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) = 0;

        const OBJMesh &m_shape;
    };

    struct Naive final : public AccelerationMesh{
        explicit Naive(const OBJMesh &shape);

        void build() override;

        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) override;
    };

    // Octree for OBJMesh
    struct Octree final : public AccelerationMesh{
        explicit Octree(const OBJMesh &shape);

        struct Node{
            Node() {};
            explicit Node(const AABB &aabb);

            void construct_children(const OBJMesh &shape);
            void construct_children_recursively(const OBJMesh &shape, int depth);

            std::tuple<bool, RayIntersectInfo> ray_intersect_leaf(const Ray &ray, const OBJMesh &shape) const;
            std::tuple<bool, RayIntersectInfo> ray_intersect_branch(const Ray &ray, const OBJMesh &shape) const;
            std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray, const OBJMesh &shape) const;
            bool is_leaf() const;

            AABB m_aabb;
            std::vector<Node> m_childs;
            std::vector<Index> m_triangle_indices;

        };

        void build() override;
        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) override;

        static constexpr Index MAX_DEPTH = 10;
        static constexpr Index MAX_TRIANGLE_NUM = 30;
        Node m_head;
    };

}

