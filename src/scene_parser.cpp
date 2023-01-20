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

#include <scene_parser.h>

#include <scene.h>
#include <integrators.h>
#include <camera.h>
#include <bsdf.h>
#include <shape.h>

namespace Caramel{
    SceneParser::SceneParser(const std::filesystem::path &path) {
        CRM_LOG("Parsing " + path.string());
        if(!std::filesystem::exists(path)){
            CRM_ERROR("Scene file does not exists : " + path.string());
        }
        
        std::ifstream stream(path);
        m_scene_json = Json::parse(stream);
    }

    Integrator* SceneParser::parse_integrator() const {
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
            CRM_ERROR(type + "integrator is not supported : "+ to_string(child));
        }
    }

    Camera* SceneParser::parse_camera() const {
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
        else{
            CRM_ERROR(type + "camera is not supported : "+ to_string(child));
        }
    }

    std::vector<Shape*> SceneParser::parse_shapes() const {
        std::vector<Shape*> shapes;

        const Json child = get_unique_first_elem(m_scene_json, "shape");
        if(child.is_array()){
            for(const auto &ch : child){
                shapes.push_back(parse_shape(ch));
            }
        }
        return shapes;
    }

    Light* SceneParser::parse_light() const {
        return nullptr;
    }

    Shape* SceneParser::parse_shape(const SceneParser::Json &shape_json) const {
        const std::string type = parse_string(shape_json, "type");
        if(type=="obj"){
            return Shape::Create<OBJMesh>(parse_string(shape_json, "path"),
                                          parse_bsdf(shape_json),
                                          shape_json.contains("arealight") ?
                                                                           parse_arealight(shape_json) :
                                                                           nullptr,
                                          shape_json.contains("to_world") ?
                                                                          parse_matrix44f(shape_json, "to_world") :
                                                                          Matrix44f::identity());
        }
        else if(type=="triangle"){
            if(shape_json.contains("n0") || shape_json.contains("n1") || shape_json.contains("n2")){
                return Shape::Create<Triangle>(parse_vector3f(shape_json, "p0"),
                                               parse_vector3f(shape_json, "p1"),
                                               parse_vector3f(shape_json, "p2"),
                                               parse_vector3f(shape_json, "n0"),
                                               parse_vector3f(shape_json, "n1"),
                                               parse_vector3f(shape_json, "n2"));
            }
            else{
                return Shape::Create<Triangle>(parse_vector3f(shape_json, "p0"),
                                               parse_vector3f(shape_json, "p1"),
                                               parse_vector3f(shape_json, "p2"));
            }
        }

        CRM_ERROR("Unsupported shape type : " + type);
        return nullptr;
    }

    AreaLight* SceneParser::parse_arealight(const SceneParser::Json &shape_json) const {
        const Json child = get_unique_first_elem(shape_json, "arealight");
        return AreaLight::Create(parse_vector3f(child, "radiance"));
    }

    BSDF* SceneParser::parse_bsdf(const SceneParser::Json &bsdf_json) const {
        const Json child = get_unique_first_elem(bsdf_json, "bsdf");
        const std::string type = parse_string(child, "type");
        if(type == "diffuse"){
            return child.contains("albedo") ?
                                            BSDF::Create<Diffuse>(parse_vector3f(child, "albedo")) :
                                            BSDF::Create<Diffuse>();
        }
        else if(type=="mirror"){
            return BSDF::Create<Mirror>();
        }
        else if(type=="dielectric"){
            return BSDF::Create<Dielectric>(parse_positive_float(child, "in_ior"),
                                            parse_positive_float(child, "ex_ior"));
        }
        else if(type=="microfacet"){
            return BSDF::Create<Microfacet>(parse_positive_float(child, "alpha"),
                                            parse_positive_float(child, "in_ior"),
                                            parse_positive_float(child, "ex_ior"),
                                            parse_vector3f(child, "kd"));
        }
        else if(type=="orennayar"){
            return BSDF::Create<OrenNayar>(parse_vector3f(child, "albedo"),
                                           parse_positive_float(child, "sigma"));
        }
        else{
            CRM_ERROR(type + "bsdf is not supported : "+ to_string(child));
        }

    }

    SceneParser::Json SceneParser::get_unique_first_elem(const SceneParser::Json &parent, const std::string &key) const {
        if(!parent.contains(key)){
            CRM_ERROR("Can not found " + key + " in json : "+ to_string(parent));
        }
        if(parent.count(key) > 1){
            CRM_WARNING("Duplicated key " + key + "is found in json : " + to_string(parent));
        }
        return parent[key];
    }

    Vector3f SceneParser::parse_vector3f(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);

        if(child.is_array() && child.size()==3 &&
            std::all_of(child.begin(), child.end(), [](auto e){return e.is_number();})){
            return Vector3f{static_cast<Float>(child[0]),
                            static_cast<Float>(child[1]),
                            static_cast<Float>(child[2])};
        }
        CRM_ERROR("Can not parse vector3f : " + to_string(child));
    }

    std::string SceneParser::parse_string(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);

        if(child.is_string()){
            return child;
        }
        // ???
        CRM_ERROR("Can not parse string : " + to_string(child));
    }

    Float SceneParser::parse_positive_float(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(child.is_number() && child > Float0){
            return static_cast<Float>(child);
        }
        CRM_ERROR("Can not parse float : " + to_string(child));
    }

    Index SceneParser::parse_positive_int(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(child.is_number_integer() && child > 0){
            return static_cast<Index>(child);
        }
        CRM_ERROR("Can not parse non-negative int : " + to_string(child));
    }

    Matrix44f SceneParser::parse_matrix44f(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(!child.is_array()){
            CRM_ERROR("Can not parse matrix : " + to_string(child));
        }

        if(child.size() == 16 &&
            std::all_of(child.begin(), child.end(), [](auto e){return e.is_number();})){
            return Matrix44f{static_cast<Float>(child[0]), static_cast<Float>(child[1]), static_cast<Float>(child[2]), static_cast<Float>(child[3]),
                             static_cast<Float>(child[4]), static_cast<Float>(child[5]), static_cast<Float>(child[6]), static_cast<Float>(child[7]),
                             static_cast<Float>(child[8]), static_cast<Float>(child[9]), static_cast<Float>(child[10]), static_cast<Float>(child[11]),
                             static_cast<Float>(child[12]), static_cast<Float>(child[13]), static_cast<Float>(child[14]), static_cast<Float>(child[15])};
        }
        else if(std::all_of(child.begin(), child.end(), [](auto e){return e.is_object();})){
            Matrix44f mat = Matrix44f::identity();
            for(const auto &e : child){
                const std::string key = parse_string(e, "type");
                if(key == "translate"){
                    const Vector3f val = parse_vector3f(e, "value");
                    mat = translate(val[0], val[1], val[2]) * mat;
                }
                else if(key == "scale"){
                    const Vector3f val = parse_vector3f(e, "value");
                    mat = scale(val[0], val[1], val[2]) * mat;
                }
                else if(key=="rotate_x"){
                    mat = rotate_x(parse_positive_float(e, "degree")) * mat;
                }
                else if(key=="rotate_y"){
                    mat = rotate_y(parse_positive_float(e, "degree")) * mat;
                }
                else if(key=="rotate_z"){
                    mat = rotate_z(parse_positive_float(e, "degree")) * mat;
                }
                else{
                    CRM_ERROR("Can not parse transform : " + to_string(e));
                }
            }
            return mat;
        }
        CRM_ERROR("Can not parse matrix : " + to_string(child));
        return Matrix44f();
    }

}

