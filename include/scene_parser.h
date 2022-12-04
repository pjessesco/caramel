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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <sstream>
#include <string>
#include <vector>

#include <common.h>
#include <scene.h>
#include <integrators.h>
#include <camera.h>
#include <shape.h>

#include <json.hpp>

namespace Caramel{

    class BSDF;
    class Integrator;
    struct Shape;
    struct Camera;

    class SceneParser{
        using Json = nlohmann::json;

    public:
        explicit SceneParser(const std::filesystem::path &path) {
            CRM_LOG("Parsing " + path.string());
            if(!std::filesystem::exists(path)){
                CRM_ERROR("Scene file does not exists : " + path.string());
            }

            std::ifstream stream(path);
            m_scene_json = Json::parse(stream);
        }

        Integrator* parse_integrator() const{
            const Json child = get_unique_first_elem(m_scene_json, "integrator");
            const std::string type = parse_string(child, "type");
            if(type=="depth"){
                return Integrator::Create<DepthIntegrator>();
            }
            else if(type=="uv"){
                return Integrator::Create<UVIntegrator>();
            }
            else if(type=="hitpos"){
                return Integrator::Create<HitPosIntegrator>();
            }
            else if(type=="normal"){
                return Integrator::Create<NormalIntegrator>();
            }
            else if(type=="direct"){
                return Integrator::Create<DirectIntegrator>(parse_positive_int(child, "spp"),
                                                            SamplingType::LIGHT);
            }
            else if(type=="path"){
                return Integrator::Create<PathIntegrator>(parse_positive_int(child, "depth_rr"),
                                                          parse_positive_int(child, "depth_max"),
                                                          parse_positive_int(child, "spp"),
                                                          SamplingType::LIGHT);
            }
            else{
                CRM_ERROR("Can not found " + type + " in json : "+ to_string(child));
            }
        }

        Camera* parse_camera() const{
            const Json child = get_unique_first_elem(m_scene_json, "camera");
            const std::string type = parse_string(child, "type");
            if(true /* perspective */){
                return Camera::Create<Camera>(parse_vector3f(child, "pos"),
                                              parse_vector3f(child, "dir"),
                                              parse_vector3f(child, "up"),
                                              parse_positive_int(child, "width"),
                                              parse_positive_int(child, "height"),
                                              parse_positive_float(child, "fov"));
            }
        }

    private:

        Json get_unique_first_elem(const Json &parent, const std::string &key) const{
            if(!parent.contains(key)){
                CRM_ERROR("Can not found " + key + " in json : "+ to_string(parent));
            }
            if(parent.count(key) > 1){
                CRM_WARNING("Duplicated key " + key + "is found in json : " + to_string(parent));
            }
            return parent[key];
        }

        Vector3f parse_vector3f(const Json &parent, const std::string &key) const{
            const Json child = get_unique_first_elem(parent, key);

            if(child.is_array() && child.size()==3 &&
               child[0].is_number() && child[1].is_number() && child[2].is_number()){
                return Vector3f{static_cast<Float>(child[0]),
                                static_cast<Float>(child[1]),
                                static_cast<Float>(child[2])};
            }
            CRM_ERROR("Can not parse vector3f : " + to_string(child));
        }

        std::string parse_string(const Json &parent, const std::string &key) const{
            const Json child = get_unique_first_elem(parent, key);

            if(child.is_string()){
                return child;
            }
            // ???
            CRM_ERROR("Can not parse string : " + to_string(child));
        }

        Float parse_positive_float(const Json &parent, const std::string &key) const{
            const Json child = get_unique_first_elem(parent, key);
            if(child.is_number() && child > Float0){
                return static_cast<Float>(child);
            }
            CRM_ERROR("Can not parse float : " + to_string(child));
        }

        Index parse_positive_int(const Json &parent, const std::string &key) const{
            const Json child = get_unique_first_elem(parent, key);
            if(child.is_number_integer() && child > 0){
                return static_cast<Index>(child);
            }
            CRM_ERROR("Can not parse non-negative int : " + to_string(child));
        }


        Json m_scene_json;

    };

}
