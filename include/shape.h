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
#include <filesystem>
#include <tuple>

#include <common.h>
#include <logger.h>
#include <ray.h>
#include <rayintersectinfo.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

namespace Caramel{

    struct Shape{
        Shape() {}

        virtual void transform(const Matrix44f &transform) = 0;
        // u, v, t
        virtual std::tuple<bool, Float, Float, Float> ray_intersect(const Ray &ray) const = 0;

    };

    struct Triangle : Shape{
        Triangle() {}
        Triangle(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3,
                 const Vector3f &n1, const Vector3f &n2, const Vector3f &n3)
        : m_p(Matrix33f::from_cols(p1, p2, p3)), m_n(Matrix33f::from_cols(p1, p2, p3)) {}

        void transform(const Matrix44f &transform) override{
            ERROR("Not implemented");
        }

        inline Vector3f point(Index i) const{
            return m_p.get_col(i);
        }

        inline Vector3f normal(Index i) const{
            return m_n.get_col(i);
        }

        // u, v, t
        std::tuple<bool, Float, Float, Float> ray_intersect(const Ray &ray) const override{
            const Vector3f A = m_p.get_col(0);
            const Vector3f B = m_p.get_col(1);
            const Vector3f C = m_p.get_col(2);
            const Vector3f D = ray.m_d;

            const Vector3f T = ray.m_o - A;
            const Vector3f E1 = B - A;
            const Vector3f E2 = C - A;

            const Vector3f DE2 = cross(D, E2);
            const Vector3f TE1 = cross(T, E1);

            const Float denom = dot(DE2, E1);
            if(denom < EPSILON){
                return {false, 0, 0, 0};
            }
            const Float denom_inv = static_cast<Float>(1) / denom;

            const Float u = dot(DE2, T) * denom_inv;
            if(u < 0 || 1 < u){
                return {false, 0, 0, 0};
            }

            const Float v = dot(TE1, D) * denom_inv;
            if(v < 0 || 1 < v){
                return {false, 0, 0, 0};
            }

            const Float t = dot(TE1, E2) * denom_inv;
            return {true, u, v, t};
        }

        Matrix33f m_p;
        Matrix33f m_n;
    };

    struct OBJMesh : Shape{
        OBJMesh(const std::filesystem::path &path){
            if(!m_vertices.empty()){
                ERROR("This mesh already loaded obj file");
            }
            if(!std::filesystem::exists(path)){
                ERROR(path.string() + " is not exists");
            }
            std::string err;

            tinyobj::ObjReader reader;
            // `triangulate` option is true by default
            if(!reader.ParseFromFile(path, tinyobj::ObjReaderConfig())){
                if(!reader.Error().empty()){
                    ERROR("TinyObjReader error : " + reader.Error());
                }
                ERROR("Cannot read obj file");
            }

            const auto& attrib = reader.GetAttrib();
            const auto& shapes = reader.GetShapes();
            const auto& mats = reader.GetMaterials();

            LOG("Loading obj in "+path.string());
            if(shapes.size() != 1){
                ERROR("We do not support obj file with several shapes");
            }
            LOG(" - Parsed # of vertices : "+std::to_string(attrib.vertices.size() / 3));
            LOG(" - Parsed # of normals : "+std::to_string(attrib.normals.size()));
            LOG(" - Parsed # of faces : "+std::to_string(shapes[0].mesh.indices.size() / 3));
            LOG(" - Parsed # of texture coordinates : "+std::to_string(attrib.texcoords.size()));

            for(int i=0;i<attrib.vertices.size();i+=3){
                m_vertices.emplace_back(Vector3f(attrib.vertices[i],
                                                 attrib.vertices[i+1],
                                                 attrib.vertices[i+2]));
            }

            for(int i=0;i<attrib.normals.size();i+=3){
                m_normals.emplace_back(Vector3f(attrib.normals[i],
                                                attrib.normals[i+1],
                                                attrib.normals[i+2]));
            }

            for(int i=0;i<attrib.texcoords.size();i+=2){
                m_tex_coords.emplace_back(Vector2f(attrib.texcoords[i],
                                                   attrib.texcoords[i+1]));
            }

            const auto& indices = shapes[0].mesh.indices;
            for(int i=0;i<indices.size();i+=3){
                m_vertex_indices.emplace_back(Vector3i(indices[i].vertex_index,
                                                       indices[i+1].vertex_index,
                                                       indices[i+2].vertex_index));

                m_normal_indices.emplace_back(Vector3i(indices[i].normal_index,
                                                       indices[i+1].normal_index,
                                                       indices[i+2].normal_index));

                m_tex_coord_indices.emplace_back(Vector3i(indices[i].texcoord_index,
                                                          indices[i+1].texcoord_index,
                                                          indices[i+2].texcoord_index));
            }

            LOG(" - Loaded # of vertices : "+std::to_string(m_vertices.size()));
            LOG(" - Loaded # of normals : "+std::to_string(m_normals.size()));
            LOG(" - Loaded # of tex coords : "+std::to_string(m_tex_coords.size()));
            LOG(" - Loaded # of vertex indices : "+std::to_string(m_vertex_indices.size()));
            LOG(" - Loaded # of normal indices : "+std::to_string(m_normal_indices.size()));
            LOG(" - Loaded # of tex coord indices : "+std::to_string(m_tex_coord_indices.size()));
        }

        std::tuple<bool, Float, Float, Float> ray_intersect(const Ray &ray) const override{
            ERROR("Not implemented");
        }

        void transform(const Matrix44f &transform) override{
            ERROR("Not implemented");
        }

        Triangle get_triangle(Index i) const{
            return Triangle(m_vertices[m_vertex_indices[i].m_data.d1[0]],
                            m_vertices[m_vertex_indices[i].m_data.d1[1]],
                            m_vertices[m_vertex_indices[i].m_data.d1[2]],
                            m_normals[m_normal_indices[i].m_data.d1[0]],
                            m_normals[m_normal_indices[i].m_data.d1[1]],
                            m_normals[m_normal_indices[i].m_data.d1[2]]);
        }

        std::vector<Vector3f> m_vertices;
        std::vector<Vector3f> m_normals;
        std::vector<Vector2f> m_tex_coords;
        std::vector<Vector3i> m_vertex_indices;
        std::vector<Vector3i> m_normal_indices;
        std::vector<Vector3i> m_tex_coord_indices;
    };

}
