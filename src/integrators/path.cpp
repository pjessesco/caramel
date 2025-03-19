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

#include <integrators.h>

#include <light.h>
#include <scene.h>
#include <ray.h>
#include <camera.h>
#include <shape.h>
#include <bsdf.h>
#include <sampler.h>
#include <rayintersectinfo.h>

namespace Caramel{
    PathIntegrator::PathIntegrator(Index rr_depth, Index max_depth, Index spp)
        : MCIntegrator(spp), m_rr_depth{rr_depth}, m_max_depth{max_depth} {}

    // Different with albedo precisely...
    Vector3f PathIntegrator::get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) {
        // See previous commits for brdf sampling / light sampling only
        return mis_sampling_path(scene, i, j, sampler);
    }

    Vector3f PathIntegrator::mis_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler){
        Ray ray = scene.m_cam->sample_ray(i, j);
        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;
        bool from_specular = true;
        Float prev_brdf_pdf = Float1;

        for(Index depth=1;depth<=m_max_depth;depth++){
            auto [is_hit, info] = scene.ray_intersect(ray);

            if(!is_hit){
                if (auto envmap_light = scene.m_envmap_light; envmap_light) {
                    // We might use brdf sampling & MIS here, but light sampling itself is efficient enough
                    const Vector3f contrib = envmap_light->radiance(ray.m_o, ray.m_o + (ray.m_d * scene.m_sceneRadius * 2), -ray.m_d) % current_brdf;
                    ret = ret + contrib;
                }
                break;
            }

            const Shape *shape = scene.m_meshes[info.idx];
            const BSDF *shape_bsdf = shape->get_bsdf();
            const Vector3f local_ray_dir = info.sh_coord.to_local(ray.m_d);

            if(shape->is_light()){
                const auto light = shape->get_arealight();
                const Float pdf_solidangle = light->pdf_solidangle(ray.m_o, info.p, info.sh_coord.m_world_n);
                const Float pdf_pick_light = Float1 / static_cast<Float>(scene.m_lights.size());
                const Float weight = balance_heuristic(prev_brdf_pdf, pdf_pick_light * pdf_solidangle);
                const Vector3f contrib = light->radiance(ray.m_o, info.p, info.sh_coord.m_world_n) % current_brdf;

                ret = ret + ((depth==0 || from_specular) ? contrib : contrib * weight);
                break;
            }

            if(depth == m_max_depth){
                break;
            }

            // emiiter sampling
            const bool is_current_specular = shape_bsdf->is_discrete();
            if(!is_current_specular){
                auto [light, light_pick_pdf] = scene.sample_light(sampler);
                auto [emitted_rad, light_pos, light_n_world, light_pos_pdf, light_info] = light->sample_direct_contribution(scene, info.p, sampler);

                // Continue if light sampling succeed
                if(!is_zero(emitted_rad)){
                    const Vector3f hitpos_to_light_local_normal = info.sh_coord.to_local(light_pos - info.p).normalize();
                    const Vector3f fr = shape_bsdf->get_reflection(local_ray_dir, hitpos_to_light_local_normal, info.tex_uv);

                    if(light->is_delta() || light->is_envlight()){
                        // We don't perform MIS for delta light
                        ret = ret + (fr % emitted_rad % current_brdf) * std::abs(hitpos_to_light_local_normal[2]) / light_pick_pdf;
                    }
                    else{
                        // MIS for light sampling
                        const Float pdf_solidangle = light->pdf_solidangle(info.p, light_pos, light_info.sh_coord.m_world_n);
                        const Float bsdf_pdf = shape->get_bsdf()->pdf(local_ray_dir, hitpos_to_light_local_normal);
                        const Float light_pdf = light_pick_pdf * pdf_solidangle;
                        ret = ret + (fr % emitted_rad % current_brdf) * std::abs(hitpos_to_light_local_normal[2]) * balance_heuristic(light_pdf, bsdf_pdf) / light_pdf;
                    }
                }
            }

            /* Russian roulette */{
                if(!from_specular && depth >= m_rr_depth){
                    if(current_brdf.max() > sampler.sample_1d()){
                        current_brdf = current_brdf / current_brdf.max();
                    }
                    else{
                        break;
                    }
                }
            }

            /* brdf sampling */{
                auto [local_recursive_dir, sampled_brdf, bsdf_pdf] = shape_bsdf->sample_recursive_dir(local_ray_dir, info.tex_uv, sampler);
                current_brdf = current_brdf % sampled_brdf;
                from_specular = is_current_specular;
                ray = info.recursive_ray_to(local_recursive_dir);
                prev_brdf_pdf = bsdf_pdf;
            }
        }
        return ret;
    }
}
