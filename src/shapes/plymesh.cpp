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

#include <filesystem>

#include <shape.h>

#include <logger.h>
#include <mesh_accel.h>
#include <transform.h>

#include "happly.h"

namespace Caramel {
    PLYMesh::PLYMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight, const Matrix44f &transform)
    : TriangleMesh(bsdf, arealight) {
        if (!std::filesystem::exists(path)) {
            CRM_ERROR(path.string() + " does not exist");
        }

        CRM_LOG("Loading ply : " + path.string());

        happly::PLYData plyData(path.string());

        // Get vertex positions
        std::vector<std::array<double, 3>> vertices = plyData.getVertexPositions();

        // Check if normals exist
        is_vn_exists = plyData.getElement("vertex").hasProperty("nx") &&
                       plyData.getElement("vertex").hasProperty("ny") &&
                       plyData.getElement("vertex").hasProperty("nz");

        std::vector<float> nx, ny, nz;
        if (is_vn_exists) {
            nx = plyData.getElement("vertex").getProperty<float>("nx");
            ny = plyData.getElement("vertex").getProperty<float>("ny");
            nz = plyData.getElement("vertex").getProperty<float>("nz");
        }

        // Get face indices
        std::vector<std::vector<size_t>> faces = plyData.getFaceIndices<size_t>();

        // Process vertices
        m_vertices.reserve(vertices.size());
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Vector3f transformed_point = transform_point(
                {static_cast<Float>(vertices[i][0]),
                 static_cast<Float>(vertices[i][1]),
                 static_cast<Float>(vertices[i][2])},
                transform);
            m_vertices.emplace_back(transformed_point[0],
                                    transformed_point[1],
                                    transformed_point[2]);
        }

        // Process normals
        if (is_vn_exists) {
            m_normals.reserve(vertices.size());
            for (size_t i = 0; i < vertices.size(); ++i) {
                const Vector3f transformed_normal = transform_normal(
                    {static_cast<Float>(nx[i]),
                     static_cast<Float>(ny[i]),
                     static_cast<Float>(nz[i])},
                    transform);
                m_normals.emplace_back(transformed_normal[0],
                                       transformed_normal[1],
                                       transformed_normal[2]);
            }
        }

        // Process faces - triangulate polygons via fan triangulation
        for (const auto& face : faces) {
            if (face.size() < 3) {
                CRM_WARNING("Skipping degenerate face with less than 3 vertices");
                continue;
            }
            for (size_t i = 1; i + 1 < face.size(); ++i) {
                m_face_indices.emplace_back(
                    static_cast<Int>(face[0]),
                    static_cast<Int>(face[i]),
                    static_cast<Int>(face[i + 1]));
            }
        }

        finalize(arealight, path.string());
    }
}
