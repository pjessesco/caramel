//
// Created by Jino Park on 2022/08/31.
//

#pragma once

#include <vector>

#include <common.h>
#include <shape.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <aabb.h>

namespace Caramel{

    class OBJMesh;

    // Divide a single mesh
    struct AccelerationMesh{
        friend class OBJMesh;

        explicit AccelerationMesh(const OBJMesh &shape) : m_shape{shape} {}
        virtual ~AccelerationMesh() = default;

        virtual void build() = 0;

        // Trace ray
        virtual std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) = 0;

        const OBJMesh &m_shape;
    };

    struct Naive : public AccelerationMesh{
        explicit Naive(const OBJMesh &shape);

        void build() override;

        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) override;
    };

    // Octree for OBJMesh
    struct Octree : public AccelerationMesh{
        explicit Octree(const OBJMesh &shape);

        struct Node{
            Node() {};
            explicit Node(const AABB &aabb);

            void construct_children(const OBJMesh &shape);
            void construct_children_recursively(const OBJMesh &shape, int depth);

            std::tuple<bool, RayIntersectInfo> ray_intersect_leaf(const Ray &ray, const OBJMesh &shape);
            std::tuple<bool, RayIntersectInfo> ray_intersect_branch(const Ray &ray, const OBJMesh &shape);
            std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray, const OBJMesh &shape);
            bool is_leaf() const;

            AABB m_aabb;
            std::vector<Node> m_childs;
            std::vector<Index> m_triangle_indices;

        };

        void build() override;
        std::tuple<bool, RayIntersectInfo> ray_intersect(const Ray &ray) override;

        static constexpr Index MAX_DEPTH = 4;
        static constexpr Index MAX_TRIANGLE_NUM = 30;
        Node m_head;
    };

}

