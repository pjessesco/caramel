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

#include <common.h>
#include <rayintersectinfo.h>
#include <ray.h>

namespace Caramel{

    RayIntersectInfo::RayIntersectInfo() : p{Float0, Float0, Float0}, sh_coord(), t(INF), tex_uv{INF, INF}, shape(nullptr) {}

    // See also `Scene::is_visible()`
    Ray RayIntersectInfo::recursive_ray_to(const Vector3f &local_next_dir) const{
        const Vector3f world_d = sh_coord.to_world(local_next_dir);

        // TODO : add offset using geometry normal
        return {p + (world_d * EPSILON), world_d};
    }

}
