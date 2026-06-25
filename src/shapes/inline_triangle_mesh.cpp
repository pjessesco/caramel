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

#include <utility>

#include <shape.h>

#include <mesh_accel.h>
#include <transform.h>

namespace Caramel {
    InlineTriangleMesh::InlineTriangleMesh(std::vector<Vector3f> positions,
                                           std::vector<Vector3i> indices,
                                           std::vector<Vector3f> normals,
                                           BSDF *bsdf, AreaLight *arealight,
                                           const Matrix44f &transform)
    : TriangleMesh(bsdf, arealight) {
        is_vn_exists = !normals.empty();

        m_vertices.reserve(positions.size());
        for (const auto &p : positions) {
            m_vertices.emplace_back(transform_point(p, transform));
        }

        if (is_vn_exists) {
            m_normals.reserve(normals.size());
            for (const auto &n : normals) {
                m_normals.emplace_back(transform_normal(n, transform));
            }
        }

        m_face_indices = std::move(indices);

        finalize(arealight, "trianglemesh");
    }
}
