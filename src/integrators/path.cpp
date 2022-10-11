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

#include <integrators.h>
#include <light.h>
#include <scene.h>

namespace Caramel{
    PathIntegrator::PathIntegrator(const Scene &scene, Index max_depth, SamplingType sampling_type)
        : Integrator(scene), m_max_depth{max_depth}, m_sampling_type(sampling_type) {}

    // Different with albedo precisely...
    Vector3f PathIntegrator::get_pixel_value(Float i, Float j, Sampler &sampler) {
        switch (m_sampling_type) {
            case SamplingType::BSDF:
                return brdf_sampling_path(i, j, sampler);
            case SamplingType::LIGHT:
                return emitter_sampling_path(i, j, sampler);
            case SamplingType::MIS:
                return mis_sampling_path(i, j, sampler);
        }
    }

    Vector3f PathIntegrator::brdf_sampling_path(Float i, Float j, Sampler &sampler){
        Ray ray = m_scene.m_cam.sample_ray(i, j);
        RayIntersectInfo info;

        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;

        // Note the loop range difference
        for(Index depth=0;depth<=m_max_depth;depth++){
            bool is_hit;
            std::tie(is_hit, info) = m_scene.ray_intersect(ray);

            if(!is_hit){
                return vec3f_zero;
            }

            if(m_scene.m_meshes[info.idx]->is_light()){
                return mult_ewise(current_brdf, m_scene.m_meshes[info.idx]->m_arealight->radiance());
            }

            // brdf sample
            auto [recursive_dir, sampled_brdf, brdf_pdf] = m_scene.m_meshes[info.idx]->m_bsdf->sample_recursive_dir(ray.m_d, sampler, info.sh_coord);
            current_brdf = mult_ewise(current_brdf, sampled_brdf);

            ray = Ray(info.p, recursive_dir);
        }

        return ret;
    }


    Vector3f PathIntegrator::emitter_sampling_path(Float i, Float j, Sampler &sampler){
        Ray ray = m_scene.m_cam.sample_ray(i, j);
        RayIntersectInfo info;

        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;
        bool from_specular = true;

        for(Index depth=0;depth<m_max_depth;depth++){
            bool is_hit;
            std::tie(is_hit, info) = m_scene.ray_intersect(ray);

            if(!is_hit){
                break;
            }

            if(m_scene.m_meshes[info.idx]->is_light()){
                if(depth == 0 || from_specular){
                    ret = ret + mult_ewise(m_scene.m_meshes[info.idx]->m_arealight->radiance(), current_brdf);
                }
                break;
            }

            // emiiter sampling
            const bool is_current_specular = m_scene.m_meshes[info.idx]->m_bsdf->is_discrete();
            if(!is_current_specular){
                auto [light, light_pdf] = m_scene.sample_light(sampler);
                auto [emitted_rad, light_pos, light_n, light_pos_pdf] = light->sample_contribution(info.p, sampler);

                const Vector3f hitpos_to_light_world = light_pos - info.p;
                const Vector3f hitpos_to_light_world_normal = hitpos_to_light_world.normalize();
                const Float dist_square = hitpos_to_light_world.dot(hitpos_to_light_world);

                Vector3f fr = m_scene.m_meshes[info.idx]->m_bsdf->get_reflection(ray.m_d, hitpos_to_light_world.normalize(), info.sh_coord);

                Float geo = light_n.dot(-1 * hitpos_to_light_world_normal) * info.sh_coord.m_world_n.dot(hitpos_to_light_world_normal) / dist_square;
                Float pdf = light_pdf * light_pos_pdf;

                ret = ret + mult_ewise(mult_ewise(fr, emitted_rad), current_brdf) * geo / pdf;
            }

            // brdf sampling
            auto [recursive_dir, sampled_brdf, brdf_pdf] = m_scene.m_meshes[info.idx]->m_bsdf->sample_recursive_dir(ray.m_d, sampler, info.sh_coord);
            current_brdf = mult_ewise(current_brdf, sampled_brdf);
            from_specular = m_scene.m_meshes[info.idx]->m_bsdf->is_discrete();

            ray = Ray(info.p, recursive_dir);
        }
        return ret;
    }


    Vector3f PathIntegrator::mis_sampling_path(Float i, Float j, Sampler &sampler){
        Ray ray = m_scene.m_cam.sample_ray(i, j);
        RayIntersectInfo info;

        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;
        bool from_specular = true;

        for(Index depth=0;depth<m_max_depth;depth++){
            bool is_hit;
            std::tie(is_hit, info) = m_scene.ray_intersect(ray);

            if(!is_hit){
                break;
            }

            if(m_scene.m_meshes[info.idx]->is_light()){
                ret = ret + mult_ewise(m_scene.m_meshes[info.idx]->m_arealight->radiance(), current_brdf);
            }

            // emiiter sampling
            auto [light, light_choose_pdf] = m_scene.sample_light(sampler);
            auto [emitted_rad, light_pos, light_n, light_pos_pdf] = light->sample_contribution(info.p, sampler);

            const Vector3f hitpos_to_light_world = light_pos - info.p;
            const Vector3f hitpos_to_light_world_normal = hitpos_to_light_world.normalize();
            const Float dist_square = hitpos_to_light_world.dot(hitpos_to_light_world);

            Vector3f fr = m_scene.m_meshes[info.idx]->m_bsdf->get_reflection(ray.m_d, hitpos_to_light_world.normalize(), info.sh_coord);

            Float geo = light_n.dot(-1 * hitpos_to_light_world_normal) * info.sh_coord.m_world_n.dot(hitpos_to_light_world_normal) / dist_square;
            Float light_pdf = light_choose_pdf * light_pos_pdf;

            // Store light contribution, this will be calculated after getting brdf_pdf.
            Vector3f light_contrib = mult_ewise(mult_ewise(fr, emitted_rad), current_brdf) * geo / light_pdf;


            // brdf sampling
            auto [recursive_dir, sampled_brdf, brdf_pdf] = m_scene.m_meshes[info.idx]->m_bsdf->sample_recursive_dir(ray.m_d, sampler, info.sh_coord);

            ret = ret + light_contrib * (light_pdf) / (light_pdf + brdf_pdf);

            current_brdf = mult_ewise(current_brdf, sampled_brdf) * (brdf_pdf) / (light_pdf + brdf_pdf);
            from_specular = m_scene.m_meshes[info.idx]->m_bsdf->is_discrete();

            ray = Ray(info.p, recursive_dir);
        }
        return ret;
    }
}
