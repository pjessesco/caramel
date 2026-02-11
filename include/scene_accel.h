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

#include <aabb.h>
#include <bvh_base.h>
#include <common.h>

namespace Caramel{

    class TriangleMesh;
    class RayIntersectInfo;
    class Ray;


    // Divide a single mesh
    class SceneAccel{
    public:
        virtual void build(std::vector<const Shape*> shapes) = 0;
        virtual std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const = 0;
    };

    // Traits for scene-level BVH (Shape pointers)
    struct SceneBVHTraits {
        using Primitive = const Shape *;

        AABB get_aabb(Primitive p) const;
        Vector3f get_center(Primitive p) const;
        std::pair<bool, RayIntersectInfo> ray_intersect(Primitive p, const Ray &ray, Float maxt) const;
    };

    class BVHScene final : public SceneAccel {
    public:
        void build(std::vector<const Shape*> shapes) override;
        std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;

    public:
        BVHBase<SceneBVHTraits> *m_bvh_root;
    };



}

