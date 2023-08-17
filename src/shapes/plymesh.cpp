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

#include <filesystem>
#include <tuple>

#include <shape.h>

#include <logger.h>
#include <transform.h>
#include <acceleration.h>
#include <rayintersectinfo.h>
#include <sampler.h>

#include "happly.h"

namespace Caramel {

    PLYMesh::PLYMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight, const Matrix44f &transform)
        : Shape(bsdf, arealight){
        if (!std::filesystem::exists(path)) {
            CRM_ERROR(path.string() + " is not exists");
        }

    }

    std::tuple<bool, RayIntersectInfo> PLYMesh::ray_intersect(const Ray &ray) const {
        return {true, {}};
    }

    AABB PLYMesh::get_aabb() const {
        return {};
    }

    Float PLYMesh::get_area() const{
        return 0;
    }

    std::tuple<Vector3f, Vector3f, Float> PLYMesh::sample_point(Sampler &sampler) const{

    }

    // Similar with `Triangle::pdf_solidangle()`
    // This implementation assumes that two given points are visible to each other
    Float OBJMesh::pdf_solidangle(const Vector3f &hitpos_world, const Vector3f &shapepos_world, const Vector3f &shape_normal_world) const{

    }

}
