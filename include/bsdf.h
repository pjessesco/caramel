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

#include <tuple>

#include <common.h>
#include <ray.h>
#include <warp_sample.h>
#include <coordinate.h>

namespace Caramel{

    class BSDF{
    public:
        virtual ~BSDF() = default;
        // Given incoming dir, returns sampled recursive ray direction, and reflectance * cos / pdf if non-discrete.
        virtual std::tuple<Vector3f, Vector3f> sample_recursive_dir(const Vector3f &world_incoming_dir, Sampler &sampler, const Coordinate &coord) = 0;

        // Given incoming & outgoing dir, returns reflectance
        virtual Vector3f get_reflection(const Vector3f &world_incoming_dir, const Vector3f &world_outgoing_dir, const Coordinate &coord) = 0;

        // Returns whether bsdf is discrete (mirror, etc) or continuous (diffuse, etc).
        virtual bool is_discrete() const = 0;
    };

    class Diffuse : public BSDF{
    public:
        explicit Diffuse(const Vector3f &albedo = Vector3f{Float0_5, Float0_5, Float0_5});
        std::tuple<Vector3f, Vector3f> sample_recursive_dir(const Vector3f &, Sampler &sampler, const Coordinate &coord) override;
        Vector3f get_reflection(const Vector3f &world_incoming_dir, const Vector3f &world_outgoing_dir, const Coordinate &coord) override;
        bool is_discrete() const override;
    private:
        Vector3f m_albedo;
    };

    class Mirror : public BSDF{
    public:
        explicit Mirror();
        std::tuple<Vector3f, Vector3f> sample_recursive_dir(const Vector3f &world_incoming_dir, Sampler &, const Coordinate &coord) override;
        Vector3f get_reflection(const Vector3f &world_incoming_dir, const Vector3f &world_outgoing_dir, const Coordinate &coord) override;
        bool is_discrete() const override;
    };

}