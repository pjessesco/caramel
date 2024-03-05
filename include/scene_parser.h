//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <sstream>
#include <string>
#include <vector>

#include <common.h>

#include <json.hpp>

namespace Caramel{

    class BSDF;
    class Integrator;
    class Shape;
    class Camera;
    class Light;
    class AreaLight;
    class Texture;

    class SceneParser{
        using Json = nlohmann::json;

    public:
        explicit SceneParser(const std::filesystem::path &path);

        Integrator* parse_integrator() const;

        Camera* parse_camera() const;

        std::vector<Shape*> parse_shapes() const;

        std::vector<Light*> parse_lights() const;

        Light* parse_light() const;

    private:
        Shape* parse_shape(const Json &shape_json) const;

        Light* parse_light(const Json &light_json) const;

        // Other lights are handled in `parse_light()`
        AreaLight* parse_arealight(const Json &shape_json) const;

        BSDF* parse_bsdf(const Json &bsdf_json) const;

        Texture* parse_texture(const Json &texture_json) const;

        Json get_unique_first_elem(const Json &parent, const std::string &key) const;

        Vector3f parse_vector3f(const Json &parent, const std::string &key) const;

        std::string parse_string(const Json &parent, const std::string &key) const;

        Float parse_positive_float(const Json &parent, const std::string &key) const;

        Float parse_float(const Json &parent, const std::string &key) const;

        Index parse_positive_int(const Json &parent, const std::string &key) const;

        Index parse_nonnegative_int(const Json &parent, const std::string &key) const;

        Matrix44f parse_matrix44f(const Json &parent, const std::string &key) const;

        Json m_scene_json;
    };

}
