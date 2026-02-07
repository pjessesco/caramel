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
#include <string>

#include <common.h>
#include <aabb.h>

namespace Caramel{

    class Shape;

    class BVHNode {
    public:
        explicit BVHNode(const std::vector<const Shape*> &shapes);
        void create_child();
        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const;
        bool is_leaf() const;

        // void print_stats() const;

    private:
        // struct Stats {
        //     int total_nodes = 0;
        //     int leaf_nodes = 0;
        //     int inner_nodes = 0;
        //     int max_depth = 0;
        //     int min_shapes_per_leaf = std::numeric_limits<int>::max();
        //     int max_shapes_per_leaf = 0;
        //     int total_shapes_in_leaves = 0;
        // };
        // void collect_stats(Stats &stats, int depth) const;
        // void print_tree(std::string prefix, bool is_left, int depth, int max_depth) const;

        std::vector<const Shape*> m_shapes;

        std::unique_ptr<BVHNode> m_left;
        std::unique_ptr<BVHNode> m_right;

        AABB m_aabb;


        /*
         *      +----------------------------------+
         *      |      |      |      |      |      |  :  SUBSPACE_COUNT = 5
         *      +----------------------------------+     CUT_COUNT = 4
         */
        static constexpr int SUBSPACE_COUNT = 12;
        static constexpr int CUT_COUNT = SUBSPACE_COUNT - 1;
        static constexpr int MAX_SHAPE_NUM = 4;
        // as pbrt says so...
        static constexpr int COST_TRAVERSAL = 1;
        static constexpr int COST_INTERSECTION = 2;

        static constexpr bool USE_SAH = true;
    };


}

