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

#include <common.h>

namespace Caramel{

    //         Local coord                    World coord
    //
    //           1  0  0             axis1[0]  axis2[0]  world_n[0]
    //           0  1  0       ->    axis1[1]  axis2[1]  world_n[1]
    //           0  0  1             axis1[2]  axis2[2]  world_n[2]
    //
    //              l0,        ->    axis1[0]  axis2[0]  world_n[0]   l0,
    //              l1               axis1[1]  axis2[1]  world_n[1] * l1,
    //              l2               axis1[2]  axis2[2]  world_n[2]   l2
    //

    struct Coordinate{
        Coordinate() : m_world_n{Float0, Float0, Float1}, m_axis1{Float1, Float0, Float0}, m_axis2{Float0, Float1, Float0} {}

        explicit Coordinate(const Vector3f &world_normal){
            m_world_n = world_normal.normalize();
            if(Peanut::is_zero(m_world_n[0]) && Peanut::is_zero(m_world_n[2])){
                m_axis1 = Vector3f(m_world_n[1], -m_world_n[0], Float0).normalize();
                m_axis2 = Vector3f::cross(m_world_n, m_axis1);
            }
            else{
                m_axis1 = Vector3f(m_world_n[2], Float0, -m_world_n[0]).normalize();
                m_axis2 = Vector3f::cross(m_world_n, m_axis1);
            }
        }

        Vector3f to_world(const Vector3f &local_vec) const{
            return m_axis1 * local_vec[0] + m_axis2 * local_vec[1] + m_world_n * local_vec[2];
        }

        Vector3f to_local(const Vector3f &world_vec) const{
            return {m_axis1.dot(world_vec), m_axis2.dot(world_vec), m_world_n.dot(world_vec)};
        }


        Vector3f m_world_n;
        Vector3f m_axis1;
        Vector3f m_axis2;
    };




}