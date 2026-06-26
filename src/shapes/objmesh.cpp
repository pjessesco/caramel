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
#include <map>
#include <tuple>

#include <shape.h>

#include <logger.h>
#include <mesh_accel.h>
#include <transform.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace Caramel {
    OBJMesh::OBJMesh(const std::filesystem::path &path, BSDF *bsdf, AreaLight *arealight, const Matrix44f &transform)
    : TriangleMesh(bsdf, arealight) {
        if (!std::filesystem::exists(path)) {
            CRM_ERROR(path.string() + " is not exists");
        }

        tinyobj::ObjReader reader;
        // `triangulate` option is true by default
        if (!reader.ParseFromFile(path.string(), tinyobj::ObjReaderConfig())) {
            if (!reader.Error().empty()) {
                CRM_ERROR("TinyObjReader error : " + reader.Error());
            }
            CRM_ERROR("Cannot read obj file");
        }

        const auto &attrib = reader.GetAttrib();
        const auto &shapes = reader.GetShapes();

        CRM_LOG("Loading obj : " + path.string());
        if (shapes.size() != 1) {
            CRM_ERROR("We do not support obj file with several shapes");
        }

        is_vn_exists = !attrib.normals.empty();
        is_tx_exists = !attrib.texcoords.empty();

        const auto &indices = shapes[0].mesh.indices;
        for(const auto &idx : indices){
            if(idx.normal_index < 0)   is_vn_exists = false;
            if(idx.texcoord_index < 0) is_tx_exists = false;
        }

        std::map<std::tuple<int, int, int>, Int> welded;
        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            Int tri[3];
            for (int k = 0; k < 3; ++k) {
                const auto &idx = indices[i + k];
                const auto key = std::make_tuple(idx.vertex_index,
                                                 is_vn_exists ? idx.normal_index : 0,
                                                 is_tx_exists ? idx.texcoord_index : 0);
                auto [it, inserted] = welded.try_emplace(key, static_cast<Int>(m_vertices.size()));
                if (inserted) {
                    const int v = idx.vertex_index;
                    m_vertices.emplace_back(transform_point(
                        Vector3f{attrib.vertices[3 * v], attrib.vertices[3 * v + 1], attrib.vertices[3 * v + 2]}, transform));
                    if (is_vn_exists) {
                        const int n = idx.normal_index;
                        m_normals.emplace_back(transform_normal(
                            Vector3f{attrib.normals[3 * n], attrib.normals[3 * n + 1], attrib.normals[3 * n + 2]}, transform));
                    }
                    if (is_tx_exists) {
                        const int t = idx.texcoord_index;
                        m_tex_coords.emplace_back(attrib.texcoords[2 * t], attrib.texcoords[2 * t + 1]);
                    }
                }
                tri[k] = it->second;
            }
            m_face_indices.emplace_back(tri[0], tri[1], tri[2]);
        }

        finalize(arealight, path.string());
    }
}
