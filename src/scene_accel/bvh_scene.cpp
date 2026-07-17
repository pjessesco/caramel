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

#include <scene_accel.h>

#include <bvh_base.h>
#include <common.h>
#include <ray.h>
#include <rayintersectinfo.h>
#include <shape.h>

namespace Caramel{
    std::optional<RayIntersectInfo> BVHSceneTraits::intersect(const Shape *s, const Ray &ray, Float maxt) const {
        auto [hit, info] = s->ray_intersect(ray, maxt);
        if (!hit) {
            return std::nullopt;
        }
        info.shape = s;
        return info;
    }

    RayIntersectInfo BVHSceneTraits::finalize(const RayIntersectInfo &hit, const Ray &) const {
        return hit;
    }

    bool BVHSceneTraits::occluded(const Shape *s, const Ray &ray, Float maxt) const {
        return s->ray_occluded(ray, maxt);
    }


    void BVHScene::build(const std::vector<const Shape*> &shapes) {
        m_bvh_root = new BVHTree<BVHSceneTraits>(shapes, BVHSceneTraits{}, Float1, Float2, 12, 4);
    }

    std::pair<bool, RayIntersectInfo> BVHScene::ray_intersect(const Ray &ray, Float maxt) const {
        return m_bvh_root->ray_intersect(ray, maxt);
    }

    bool BVHScene::ray_occluded(const Ray &ray, Float maxt) const {
        return m_bvh_root->ray_occluded(ray, maxt);
    }

}// namespace Caramel
