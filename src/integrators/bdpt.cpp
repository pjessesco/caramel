#include <integrators.h>
#include <scene.h>
#include <image.h>
#include <sampler.h>
#include <parallel_for.h>
#include <light.h>
#include <camera.h>
#include <bsdf.h>
#include <rayintersectinfo.h>
#include <shape.h>

#include <mutex>
#include <vector>

namespace Caramel{

    struct Vertex {
        RayIntersectInfo info;
        Vector3f beta; // Throughput
        Vector3f wo; // Incoming direction (pointing IN to surface)
        bool is_surface = false;
        bool is_delta = false;
    };

    BDPTIntegrator::BDPTIntegrator(Index max_depth, Index spp) 
        : m_max_depth(max_depth), m_spp(spp) {}

    void BDPTIntegrator::pre_process(const Scene &) {}

    int generate_camera_path(const Scene &scene, Sampler &sampler, int max_depth, 
                             const Vector2f &film_pos, std::vector<Vertex> &path) {
        Ray ray = scene.m_cam->sample_ray(film_pos[0], film_pos[1]);
        Vector3f beta = vec3f_one; 
        
        for(int depth=0; depth < max_depth; ++depth){
            auto [is_hit, info] = scene.ray_intersect(ray);
            if(!is_hit) break;

            Vertex v;
            v.info = info;
            v.beta = beta;
            v.wo = ray.m_d;
            v.is_surface = true;
            v.is_delta = info.shape->get_bsdf()->is_discrete();
            path.push_back(v);

            auto [local_next_dir, bsdf_val, bsdf_pdf] = info.shape->get_bsdf()->sample_recursive_dir(
                info.sh_coord.to_local(ray.m_d), info.tex_uv, sampler);

            if(is_zero(bsdf_val)) break;

            beta = beta % bsdf_val;
            ray = Ray(info.p, info.sh_coord.to_world(local_next_dir));
            
            if (depth > 3) {
                Float q = std::max(static_cast<Float>(0.05f), Float1 - beta.max());
                if (sampler.sample_1d() < q) break;
                beta = beta / (Float1 - q);
            }
        }
        return path.size();
    }

    int generate_light_path(const Scene &scene, Sampler &sampler, int max_depth, std::vector<Vertex> &path) {
        auto [light, light_pdf] = scene.sample_light(sampler);
        auto [ray, normal, Le, pdf_pos, pdf_dir] = light->sample_le(sampler);
        
        Float cos_light = Float1;
        if(!light->is_delta()) cos_light = std::abs(normal.dot(ray.m_d));
        
        Vector3f beta = Le * (cos_light / (light_pdf * pdf_pos * pdf_dir));
        
        for(int depth=0; depth < max_depth; ++depth){
            auto [is_hit, info] = scene.ray_intersect(ray);
            if(!is_hit) break;
            
            Vertex v;
            v.info = info;
            v.beta = beta;
            v.wo = ray.m_d;
            v.is_surface = true;
            v.is_delta = info.shape->get_bsdf()->is_discrete();
            path.push_back(v);
            
            auto [local_next_dir, bsdf_val, bsdf_pdf] = info.shape->get_bsdf()->sample_recursive_dir(
                info.sh_coord.to_local(ray.m_d), info.tex_uv, sampler);
                
            if(is_zero(bsdf_val)) break;
            
            beta = beta % bsdf_val;
            ray = Ray(info.p, info.sh_coord.to_world(local_next_dir));
            
            if (depth > 3) {
                Float q = std::max(static_cast<Float>(0.05f), Float1 - beta.max());
                if (sampler.sample_1d() < q) break;
                beta = beta / (Float1 - q);
            }
        }
        return path.size();
    }

    Image BDPTIntegrator::render(const Scene &scene) {
        const auto size = scene.m_cam->get_size();
        Image img(size.first, size.second);
        std::vector<std::mutex> scanline_mutexes(size.second);

        parallel_for(0, size.first * size.second, [&](Index idx){
            int x = idx % size.first;
            int y = idx / size.first;
            
            UniformStdSampler sampler(idx);

            for(int s=0; s<m_spp; ++s){
                std::vector<Vertex> camera_path;
                generate_camera_path(scene, sampler, m_max_depth, Vector2f(static_cast<Float>(x), static_cast<Float>(y)), camera_path);
                
                std::vector<Vertex> light_path;
                generate_light_path(scene, sampler, m_max_depth, light_path);
                
                Vector3f pixel_val = vec3f_zero;
                
                // 1. Camera Path only (s=0, t>=1) - Light Hit / Env Hit
                for(size_t i=0; i<camera_path.size(); ++i){
                    const auto& cv = camera_path[i];
                    if(cv.info.shape->is_light()){
                        Vector3f Le = cv.info.shape->get_arealight()->radiance(cv.info.p, cv.info.p, cv.info.sh_coord.m_world_n);
                        int k = i + 1; // Path length
                        Float weight = 1.0f / (k + 2); // Equal weighting
                        pixel_val = pixel_val + (Le % cv.beta) * weight;
                    }
                }
                
                // 1.5 Direct Lighting (s=1, t>=1) - NEE
                for(size_t i=0; i<camera_path.size(); ++i){
                    const auto& cv = camera_path[i];
                    auto [light, light_pick_pdf] = scene.sample_light(sampler);
                    auto [emitted_rad, light_pos, light_n_world, light_pos_pdf, light_info] = light->sample_direct_contribution(scene, cv.info, sampler);
                    
                    if(!is_zero(emitted_rad)){
                        Vector3f hitpos_to_light_local = cv.info.sh_coord.to_local(light_pos - cv.info.p).normalize();
                        Vector3f f = cv.info.shape->get_bsdf()->get_reflection(
                            cv.info.sh_coord.to_local(cv.wo),
                            hitpos_to_light_local,
                            cv.info.tex_uv);
                            
                        if(!is_zero(f)){
                            int k = (i + 1) + 1;
                            Float weight = 1.0f / (k + 2);
                            
                            if(light->is_delta()){
                                pixel_val = pixel_val + (cv.beta % f % emitted_rad) * weight; // PointLight: G included
                            } else {
                                Vector3f dir = light_pos - cv.info.p;
                                Float dist2 = dir.dot(dir);
                                Float cos_light = std::abs(light_n_world.dot(-dir.normalize()));
                                Float G = cos_light / dist2;
                                Float cos_surf = std::abs(hitpos_to_light_local[2]);
                                
                                pixel_val = pixel_val + (cv.beta % f % emitted_rad) * (weight * G * cos_surf / (light_pick_pdf * light_pos_pdf));
                            }
                        }
                    }
                }

                // 2. Vertex Connection (s>=2, t>=1)
                for(size_t j=0; j<light_path.size(); ++j){
                    const auto& lv = light_path[j];
                    for(size_t i=0; i<camera_path.size(); ++i){
                        const auto& cv = camera_path[i];
                        
                        if(lv.is_delta || cv.is_delta) continue;
                        
                        auto [visible, _] = scene.is_visible(lv.info.p, cv.info.p);
                        if(visible){
                            Vector3f dir_l_to_c = cv.info.p - lv.info.p;
                            Float dist2 = dir_l_to_c.dot(dir_l_to_c);
                            Float dist = std::sqrt(dist2);
                            dir_l_to_c = dir_l_to_c / dist;
                            
                            Vector3f f_l = lv.info.shape->get_bsdf()->get_reflection(
                                lv.info.sh_coord.to_local(lv.wo), 
                                lv.info.sh_coord.to_local(dir_l_to_c), 
                                lv.info.tex_uv);
                                
                            Vector3f f_c = cv.info.shape->get_bsdf()->get_reflection(
                                cv.info.sh_coord.to_local(cv.wo), 
                                cv.info.sh_coord.to_local(-dir_l_to_c), 
                                cv.info.tex_uv);
                                
                            if(!is_zero(f_l) && !is_zero(f_c)){
                                Float G = std::abs(dir_l_to_c.dot(lv.info.sh_coord.m_world_n)) * 
                                          std::abs(dir_l_to_c.dot(cv.info.sh_coord.m_world_n)) / dist2;
                                          
                                int k = (j + 1) + (i + 1) + 1;
                                Float weight = 1.0f / (k + 2);
                                          
                                pixel_val = pixel_val + ((lv.beta % f_l % f_c % cv.beta) * (G * weight));
                            }
                        }
                    }
                }
                
                std::lock_guard<std::mutex> lock(scanline_mutexes[y]);
                img.add_pixel_value(x, y, pixel_val);
            }
        });
        
        img.scale(Float1 / m_spp);
        return img;
    }
}
