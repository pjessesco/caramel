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

    ParticleIntegrator::ParticleIntegrator(Index max_depth, Index spp) 
        : m_max_depth(max_depth), m_spp(spp) {}

    void ParticleIntegrator::pre_process(const Scene &) {}

    Image ParticleIntegrator::render(const Scene &scene) {
        const auto size = scene.m_cam->get_size();
        Image img(size.first, size.second);

        const Index total_samples = m_spp * size.first * size.second;
        const Index num_lights = scene.m_lights.size();
        
        std::vector<std::mutex> scanline_mutexes(size.second);

        parallel_for(0, total_samples, [&](Index i){
            UniformStdSampler sampler(i);

            // 1. Select Light
            if (num_lights == 0) return;
            
            Float light_pdf = Float1 / num_lights;
            Index light_idx = static_cast<Index>(sampler.sample_1d() * num_lights);
            if(light_idx == num_lights) light_idx--;
            const Light* light = scene.m_lights[light_idx];

            // 2. Sample Ray from Light
            auto [ray, normal, Le, pdf_pos, pdf_dir] = light->sample_le(sampler);
            
            Float cos_light = Float1;
            if(!light->is_delta()){ // Area Light
                 cos_light = std::abs(normal.dot(ray.m_d));
            }

            Vector3f throughput = Le * (cos_light / (light_pdf * pdf_pos * pdf_dir));

            // 3. Trace Light Path
            for(Index depth=0; depth < m_max_depth; depth++){
                auto [is_hit, info] = scene.ray_intersect(ray);
                if(!is_hit) break;

                // Connection to Camera (Next Event Estimation for Camera)
                auto [We, dir_to_cam, dist_to_cam, raster_pos, pdf_we] = scene.m_cam->sample_wi(info.p, Vector2f{Float0, Float0});
                
                // Visibility check & Contribution
                if(pdf_we > 0){
                    Vector3f wo = ray.m_d; // incoming from light (pointing IN)
                    Vector3f wi = dir_to_cam; // outgoing to camera (pointing OUT)
                    
                    Vector3f wo_local = info.sh_coord.to_local(wo);
                    Vector3f wi_local = info.sh_coord.to_local(wi);
                    
                    Vector3f f = info.shape->get_bsdf()->get_reflection(wo_local, wi_local, info.tex_uv);
                    
                    if(!is_zero(f)){
                        // Check visibility to camera
                         auto [visible, _] = scene.is_visible(info.p, scene.m_cam->get_pos()); 
                         
                         if(visible){
                             Float cos_surface = std::abs(wi_local[2]);
                             Float G = cos_surface / (dist_to_cam * dist_to_cam);
                             
                             Vector3f contrib = (throughput % f % We) * G;
                             
                             // Splat to image
                             int x = static_cast<int>(raster_pos[0]);
                             int y = static_cast<int>(raster_pos[1]);
                             
                             if(x >=0 && x < size.first && y >=0 && y < size.second){
                                  std::lock_guard<std::mutex> lock(scanline_mutexes[y]);
                                  img.add_pixel_value(x, y, contrib);
                             }
                         }
                    }
                }
                
                // Sample Next Ray
                auto [local_next_dir, bsdf_val, bsdf_pdf] = info.shape->get_bsdf()->sample_recursive_dir(info.sh_coord.to_local(ray.m_d), info.tex_uv, sampler);
                
                if(is_zero(bsdf_val)) break;
                
                // Update throughput
                throughput = throughput % bsdf_val;
                
                // Russian Roulette
                if (depth > 3) {
                    Float q = std::max(static_cast<Float>(0.05f), Float1 - throughput.max());
                    if (sampler.sample_1d() < q) break;
                    throughput = throughput / (Float1 - q);
                }

                // Update ray
                Vector3f world_next_dir = info.sh_coord.to_world(local_next_dir);
                ray = Ray(info.p, world_next_dir);
            }
        });
        
        img.scale(Float1 / m_spp); 
        
        return img;
    }
}
