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

#include <fstream>
#include <cstdint>
#include <array>

#include <scene_parser.h>

#include <scene.h>
#include <integrators.h>
#include <camera.h>
#include <bsdf.h>
#include <shape.h>
#include <logger.h>
#include <light.h>
#include <textures.h>
#include <transform.h>

namespace Caramel{
    SceneParser::SceneParser(const std::filesystem::path &path) {
        CRM_LOG("Parsing " + path.string());
        if(!std::filesystem::exists(path)){
            CRM_ERROR("Scene file does not exists : " + path.string());
        }
        
        std::ifstream stream(path);
        m_scene_json = Json::parse(stream);
    }

    void SceneParser::parse_bsdfs_map() {
        if(!m_scene_json.contains("bsdfs")){
            return;
        }
        const Json bsdfs = m_scene_json["bsdfs"];
        if(bsdfs.is_array()){
            for(const auto &b : bsdfs){
                if(!b.contains("id")){
                    CRM_ERROR("BSDF in bsdfs list must have an id : " + to_string(b));
                }
                const std::string id = parse_string(b, "id");
                if(m_bsdf_map.find(id) != m_bsdf_map.end()){
                    CRM_ERROR("Duplicated BSDF id : " + id);
                }
                
                // Wrap to match parse_bsdf expectation
                Json wrapper;
                wrapper["bsdf"] = b;
                m_bsdf_map[id] = parse_bsdf(wrapper);
            }
        }
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
        // else if(type=="direct"){
        //     return Integrator::Create<DirectIntegrator>(parse_positive_int(child, "spp"));
        // }
        else if(type=="path"){
            return Integrator::Create<PathIntegrator>(parse_nonnegative_int(child, "depth_rr"),
                                                      parse_nonnegative_int(child, "depth_max"),
                                                      parse_positive_int(child, "spp"));
        }
        else{
            CRM_ERROR(type + "integrator is not supported : "+ to_string(child));
        }
    }

    Camera* SceneParser::parse_camera() const {
        const Json child = get_unique_first_elem(m_scene_json, "camera");
        const std::string type = parse_string(child, "type");
        if(type == "pinhole"){
            if(child.contains("pos")){
                return Camera::Create<Pinhole>(parse_vector3f(child, "pos"),
                                               parse_vector3f(child, "dir"),
                                               parse_vector3f(child, "up"),
                                               parse_positive_int(child, "width"),
                                               parse_positive_int(child, "height"),
                                               parse_positive_float(child, "fov"));
            }
            else{
                return Camera::Create<Pinhole>(parse_matrix44f(child, "matrix"),
                                               parse_positive_int(child, "width"),
                                               parse_positive_int(child, "height"),
                                               parse_positive_float(child, "fov"));
            }
        }
        else if(type == "thinlens"){
            if(child.contains("pos")){
                return Camera::Create<ThinLens>(parse_vector3f(child, "pos"),
                                               parse_vector3f(child, "dir"),
                                               parse_vector3f(child, "up"),
                                               parse_positive_int(child, "width"),
                                               parse_positive_int(child, "height"),
                                               parse_positive_float(child, "fov"),
                                               parse_positive_float(child, "lens_radius"),
                                               parse_positive_float(child, "focal_dist"));
            }
            else{
                return Camera::Create<ThinLens>(parse_matrix44f(child, "matrix"),
                                               parse_positive_int(child, "width"),
                                               parse_positive_int(child, "height"),
                                               parse_positive_float(child, "fov"),
                                               parse_positive_float(child, "lens_radius"),
                                               parse_positive_float(child, "focal_dist"));
            }
        }
        else{
            CRM_ERROR(type + " camera is not supported : "+ to_string(child));
        }
    }

    std::vector<Shape*> SceneParser::parse_shapes() const {
        std::vector<Shape*> shapes;

        const Json child = get_unique_first_elem(m_scene_json, "shape");
        if(child.is_array()){
            for(const auto &ch : child){
                const std::string type = parse_string(ch, "type");
                if(type == "instance"){
                    parse_instanced_shapes(ch, shapes);
                }
                else if(type == "curve"){
                    parse_curve_shapes(ch, shapes);
                }
                else if(type == "curvefile"){
                    parse_curve_file_shapes(ch, shapes);
                }
                else{
                    shapes.emplace_back(parse_shape(ch));
                }
            }
        }
        return shapes;
    }

    void SceneParser::parse_instanced_shapes(const SceneParser::Json &shape_json, std::vector<Shape*> &out) const {
        // Build the template once (group-local space): parallel geometry + bsdf arrays.
        //   shapes-form : many sub-shapes, each carrying its own geometry + bsdf.
        std::vector<const Shape*> geometries;
        std::vector<BSDF*> bsdfs;
        std::vector<Vector3f> radiances;

        const Json sub_shapes = get_unique_first_elem(shape_json, "shapes");
        if(!sub_shapes.is_array() || sub_shapes.empty()){
            CRM_ERROR("instance 'shapes' must be a non-empty array");
        }

        // One Instance per (template sub-shape x placement); geometry shared across placements.
        const Json instance_list = get_unique_first_elem(shape_json, "instances");
        if(!instance_list.is_array() || instance_list.empty()){
            CRM_ERROR("instance 'instances' must be a non-empty array");
        }

        for(const auto &s : sub_shapes){
            Json geom = s;
            geom.erase("arealight");
            Shape *geometry = parse_shape(geom);
            geometries.emplace_back(geometry);
            bsdfs.emplace_back(geometry->get_bsdf());

            if(s.contains("arealight")){
                const Json al_child = get_unique_first_elem(s, "arealight");
                radiances.emplace_back(parse_vector3f(al_child, "radiance"));
            } else {
                radiances.emplace_back(Vector3f{-1.0f, -1.0f, -1.0f});
            }
        }

        for(const auto &inst : instance_list){
            const Matrix44f to_world = parse_matrix44f(inst, "to_world");
            for(std::size_t i = 0; i < geometries.size(); ++i){
                AreaLight *al = nullptr;
                if(radiances[i][0] >= 0.0f){
                    al = AreaLight::Create(radiances[i]);
                }
                out.emplace_back(Shape::Create<Instance>(geometries[i], to_world, bsdfs[i], al));
            }
        }
    }

    void SceneParser::parse_curve_shapes(const SceneParser::Json &shape_json, std::vector<Shape*> &out) const {
        const Json Pj = get_unique_first_elem(shape_json, "P");
        if(!Pj.is_array() || Pj.size() % 3 != 0 || Pj.size() < 12){
            CRM_ERROR("curve 'P' must be a flat array of 3*N floats with N >= 4");
        }
        const Matrix44f to_world = shape_json.contains("to_world")
            ? parse_matrix44f(shape_json, "to_world") : Matrix44f::identity();

        std::vector<Vector3f> P;
        for(std::size_t i = 0; i + 2 < Pj.size(); i += 3){
            P.push_back(transform_point(Vector3f{static_cast<Float>(Pj[i]), static_cast<Float>(Pj[i+1]), static_cast<Float>(Pj[i+2])}, to_world));
        }

        const int degree = shape_json.contains("degree") ? static_cast<int>(parse_positive_int(shape_json, "degree")) : 3;
        if(degree != 2 && degree != 3){ CRM_ERROR("curve 'degree' must be 2 or 3"); }
        const bool bspline = shape_json.contains("basis") && parse_string(shape_json, "basis") == "bspline";

        CurveType type = CurveType::Flat;
        if(shape_json.contains("curve_type")){
            const std::string s = parse_string(shape_json, "curve_type");
            if(s == "cylinder")      type = CurveType::Cylinder;
            else if(s == "ribbon")   type = CurveType::Ribbon;
            else if(s != "flat")     CRM_ERROR("curve 'curve_type' must be flat, cylinder or ribbon");
        }

        Float w0, w1;
        if(shape_json.contains("width0") || shape_json.contains("width1")){
            w0 = parse_float(shape_json, "width0");
            w1 = parse_float(shape_json, "width1");
        }
        else if(shape_json.contains("width")){ w0 = w1 = parse_float(shape_json, "width"); }
        else { w0 = w1 = Float1; }

        std::vector<Vector3f> normals;
        if(shape_json.contains("N")){
            const Json Nj = get_unique_first_elem(shape_json, "N");
            if(!Nj.is_array() || Nj.size() % 3 != 0){ CRM_ERROR("curve 'N' must be a flat array of 3*M floats"); }
            for(std::size_t i = 0; i + 2 < Nj.size(); i += 3){
                normals.push_back(transform_normal(Vector3f{static_cast<Float>(Nj[i]), static_cast<Float>(Nj[i+1]), static_cast<Float>(Nj[i+2])}, to_world));
            }
        }

        const int split_depth = shape_json.contains("splitdepth") ? static_cast<int>(parse_nonnegative_int(shape_json, "splitdepth")) : 2;

        BSDF *bsdf = parse_bsdf(shape_json);
        create_curve(std::move(P), degree, bspline, type, w0, w1, normals, split_depth, bsdf, out);
    }

    // Bulk curves from a binary file: int32 count, then count * 12 float32 (4 control points each).
    // All curves share the entry's width/type/bsdf. Used for huge sets (e.g. bunny-fur's 1.5M curves)
    // where one JSON object per curve would be impractically large and slow to parse.
    void SceneParser::parse_curve_file_shapes(const SceneParser::Json &shape_json, std::vector<Shape*> &out) const {
        const std::string path = parse_string(shape_json, "path");
        std::ifstream f(path, std::ios::binary);
        if(!f){ CRM_ERROR("curvefile does not exist : " + path); }
        std::int32_t n = 0;
        f.read(reinterpret_cast<char*>(&n), sizeof(n));
        if(n <= 0){ CRM_ERROR("curvefile has no curves : " + path); }

        CurveType type = CurveType::Flat;
        if(shape_json.contains("curve_type")){
            const std::string s = parse_string(shape_json, "curve_type");
            if(s == "cylinder")    type = CurveType::Cylinder;
            else if(s == "ribbon") type = CurveType::Ribbon;
            else if(s != "flat")   CRM_ERROR("curvefile 'curve_type' must be flat, cylinder or ribbon");
        }
        Float w0, w1;
        if(shape_json.contains("width0") || shape_json.contains("width1")){ w0 = parse_float(shape_json, "width0"); w1 = parse_float(shape_json, "width1"); }
        else if(shape_json.contains("width")){ w0 = w1 = parse_float(shape_json, "width"); }
        else { w0 = w1 = Float1; }
        const int split_depth = shape_json.contains("splitdepth") ? static_cast<int>(parse_nonnegative_int(shape_json, "splitdepth")) : 0;
        const Matrix44f to_world = shape_json.contains("to_world") ? parse_matrix44f(shape_json, "to_world") : Matrix44f::identity();
        BSDF *bsdf = parse_bsdf(shape_json);

        if(type == CurveType::Ribbon){ CRM_ERROR("curvefile does not support ribbon (no per-curve normals); use flat or cylinder"); }

        // All curves -> ONE CurveMesh (single TLAS leaf + its own inner BVH), like TriangleMesh.
        std::vector<CurveCommon> commons;
        commons.reserve(static_cast<std::size_t>(n));
        float buf[12];
        for(std::int32_t c = 0; c < n; ++c){
            f.read(reinterpret_cast<char*>(buf), sizeof(buf));
            const std::array<Vector3f, 4> cp = {
                transform_point(Vector3f{buf[0], buf[1], buf[2]}, to_world),
                transform_point(Vector3f{buf[3], buf[4], buf[5]}, to_world),
                transform_point(Vector3f{buf[6], buf[7], buf[8]}, to_world),
                transform_point(Vector3f{buf[9], buf[10], buf[11]}, to_world)};
            commons.emplace_back(cp, w0, w1, type, nullptr);
        }
        out.push_back(new CurveMesh(std::move(commons), split_depth, bsdf));
    }

    std::vector<Light*> SceneParser::parse_lights() const{
        std::vector<Light*> lights;

        const Json child = get_unique_first_elem(m_scene_json, "light", true/*optional*/);
        if(child.is_array()){
            bool is_envmap_parsed = false;

            for(const auto &ch : child){
                auto light = parse_light(ch);
                if (light->is_envlight() && is_envmap_parsed) {
                    CRM_ERROR("Environment map can't be more than 1");
                }
                is_envmap_parsed |= light->is_envlight();
                lights.emplace_back(light);
            }
        }
        return lights;
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
        else if(type=="ply"){
            return Shape::Create<PLYMesh>(parse_string(shape_json, "path"),
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
                                               parse_vector3f(shape_json, "n2"),
                                               parse_bsdf(shape_json));
            }
            else{
                return Shape::Create<Triangle>(parse_vector3f(shape_json, "p0"),
                                               parse_vector3f(shape_json, "p1"),
                                               parse_vector3f(shape_json, "p2"),
                                               parse_bsdf(shape_json));
            }
        }
        else if(type=="trianglemesh"){
            const Json Pj = get_unique_first_elem(shape_json, "P");
            const Json Ij = get_unique_first_elem(shape_json, "indices");
            if(!Pj.is_array() || Pj.size() % 3 != 0){
                CRM_ERROR("trianglemesh 'P' must be a flat array of 3*N numbers");
            }
            if(!Ij.is_array() || Ij.size() % 3 != 0){
                CRM_ERROR("trianglemesh 'indices' must be a flat array of 3*M integers");
            }
            std::vector<Vector3f> positions;
            for(std::size_t i = 0; i + 2 < Pj.size(); i += 3){
                positions.push_back(Vector3f{static_cast<Float>(Pj[i]), static_cast<Float>(Pj[i+1]), static_cast<Float>(Pj[i+2])});
            }
            std::vector<Vector3i> indices;
            for(std::size_t i = 0; i + 2 < Ij.size(); i += 3){
                indices.push_back(Vector3i{static_cast<Int>(Ij[i]), static_cast<Int>(Ij[i+1]), static_cast<Int>(Ij[i+2])});
            }
            for(const auto &t : indices){
                for(int k = 0; k < 3; ++k){
                    if(t[k] < 0 || static_cast<std::size_t>(t[k]) >= positions.size()){
                        CRM_ERROR("trianglemesh index out of range");
                    }
                }
            }
            std::vector<Vector3f> normals;
            if(shape_json.contains("N")){
                const Json Nj = get_unique_first_elem(shape_json, "N");
                if(!Nj.is_array() || Nj.size() != Pj.size()){
                    CRM_ERROR("trianglemesh 'N' must be a flat per-vertex array matching 'P' length");
                }
                for(std::size_t i = 0; i + 2 < Nj.size(); i += 3){
                    normals.push_back(Vector3f{static_cast<Float>(Nj[i]), static_cast<Float>(Nj[i+1]), static_cast<Float>(Nj[i+2])});
                }
            }
            return Shape::Create<InlineTriangleMesh>(positions, indices, normals,
                                                     parse_bsdf(shape_json),
                                                     shape_json.contains("arealight") ? parse_arealight(shape_json) : nullptr,
                                                     shape_json.contains("to_world") ? parse_matrix44f(shape_json, "to_world") : Matrix44f::identity());
        }

        CRM_ERROR("Unsupported shape type : " + type);
        return nullptr;
    }

    Light* SceneParser::parse_light(const SceneParser::Json &light_json) const {
        const std::string type = parse_string(light_json, "type");
        if(type=="point"){
            return Light::Create<PointLight>(parse_vector3f(light_json, "pos"),
                                             parse_vector3f(light_json, "radiance"));
        }
        else if(type=="constant_env"){
            return Light::Create<ConstantEnvLight>(parse_vector3f(light_json, "radiance")/*will be replaced soon*/,
                                              parse_positive_float(light_json, "scale"));
        }
        else if(type=="image_env"){
            return Light::Create<ImageEnvLight>(parse_string(light_json, "path"),
                                                parse_positive_float(light_json, "scale"),
                                                light_json.contains("to_world") ? parse_matrix44f(light_json, "to_world") :
                                                                                  Matrix44f::identity());
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

        if(child.is_string()){
            const std::string id = child;
            if(m_bsdf_map.find(id) == m_bsdf_map.end()){
                CRM_ERROR("BSDF id not found : " + id);
            }
            return m_bsdf_map.at(id);
        }

        const std::string type = parse_string(child, "type");
        if(type == "diffuse"){
            return child.contains("albedo") ? BSDF::Create<Diffuse>(parse_vector3f(child, "albedo")) :
                   child.contains("texture") ? BSDF::Create<Diffuse>(parse_texture(child)) :
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
        else if(type=="conductor"){
            const std::string conductor = parse_string(child, "material");
            const Conductors c = conductor == "Au" ? Conductors::Au :
                                 conductor == "Ag" ? Conductors::Ag :
                                 conductor == "Al" ? Conductors::Al :
                             /*child == "Cu" ?*/ Conductors::Cu;
            return BSDF::Create<Conductor>(c, parse_positive_float(child, "ex_ior"));
        }
        else if(type=="twosided"){
            BSDF *front = parse_bsdf(child);
            Json back_wrapper;
            if(child.contains("back_bsdf")){
                back_wrapper["bsdf"] = child.at("back_bsdf");
                return BSDF::Create<TwoSided>(front, parse_bsdf(back_wrapper));
            }
            return BSDF::Create<TwoSided>(front, front);
        }
        else{
            CRM_ERROR(type + "bsdf is not supported : "+ to_string(child));
        }

    }

    Texture* SceneParser::parse_texture(const Json &texture_json) const{
        const Json child = get_unique_first_elem(texture_json, "texture");
        const std::string type = parse_string(child, "type");
        if(type=="image"){
            return Texture::Create<ImageTexture>(parse_string(child, "path"));
        }
        CRM_ERROR("Can not parse texture : " + to_string(child));
    }

    SceneParser::Json SceneParser::get_unique_first_elem(const SceneParser::Json &parent, const std::string &key, bool optional) const {
        if(!parent.contains(key)){
            if(optional){
                return {};
            }
            else{
                CRM_ERROR("Can not found " + key + " in json : "+ to_string(parent));
            }
        }
        if(parent.count(key) > 1){
            CRM_WARNING("Duplicated key " + key + "is found in json : " + to_string(parent));
        }
        return parent[key];
    }

    Vector3f SceneParser::parse_vector3f(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);

        if(child.is_array() && child.size()==3 &&
            std::ranges::all_of(child, [](auto e){return e.is_number();})){
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

    Float SceneParser::parse_float(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(child.is_number()){
            return static_cast<Float>(child);
        }
        CRM_ERROR("Can not parse float : " + to_string(child));
    }

    Float SceneParser::parse_positive_float(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(child.is_number() && child > Float0){
            return static_cast<Float>(child);
        }
        CRM_ERROR("Can not parse positive float : " + to_string(child));
    }

    Index SceneParser::parse_positive_int(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(child.is_number_integer() && child > 0){
            return static_cast<Index>(child);
        }
        CRM_ERROR("Can not parse non-negative int : " + to_string(child));
    }

    Index SceneParser::parse_nonnegative_int(const SceneParser::Json &parent, const std::string &key) const {
        const Json child = get_unique_first_elem(parent, key);
        if(child.is_number_integer() && child >= 0){
            return static_cast<Index>(child);
        }
        CRM_ERROR("Can not parse negative int : " + to_string(child));
    }

    Matrix44f SceneParser::parse_matrix44f(const SceneParser::Json &parent, const std::string &_key) const {
        const Json child = get_unique_first_elem(parent, _key);
        if(!child.is_array()){
            CRM_ERROR("Can not parse matrix : " + to_string(child));
        }

        if(child.size() == 16 &&
            std::ranges::all_of(child, [](auto e){return e.is_number();})){
                return Matrix44f{static_cast<Float>(child[0]), static_cast<Float>(child[1]), static_cast<Float>(child[2]), static_cast<Float>(child[3]),
                                 static_cast<Float>(child[4]), static_cast<Float>(child[5]), static_cast<Float>(child[6]), static_cast<Float>(child[7]),
                                 static_cast<Float>(child[8]), static_cast<Float>(child[9]), static_cast<Float>(child[10]), static_cast<Float>(child[11]),
                                 static_cast<Float>(child[12]), static_cast<Float>(child[13]), static_cast<Float>(child[14]), static_cast<Float>(child[15])};
        }
        else if(std::ranges::all_of(child, [](auto e){return e.is_object();})){
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
                    mat = rotate_x(parse_float(e, "degree")) * mat;
                }
                else if(key=="rotate_y"){
                    mat = rotate_y(parse_float(e, "degree")) * mat;
                }
                else if(key=="rotate_z"){
                    mat = rotate_z(parse_float(e, "degree")) * mat;
                }
                else{
                    CRM_ERROR("Can not parse transform : " + to_string(e));
                }
            }
            return mat;
        }
        CRM_ERROR("Can not parse matrix : " + to_string(child));
        return Matrix44f::identity();
    }

}

