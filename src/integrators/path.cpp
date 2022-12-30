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
    PathIntegrator::PathIntegrator(Index rr_depth, Index max_depth, Index spp, SamplingType sampling_type)
        : Integrator(spp), m_rr_depth{rr_depth}, m_max_depth{max_depth}, m_sampling_type(sampling_type) {}

    // Different with albedo precisely...
    Vector3f PathIntegrator::get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) {
        switch (m_sampling_type) {
            case SamplingType::BSDF:
                return brdf_sampling_path(scene, i, j, sampler);
            case SamplingType::LIGHT:
                return emitter_sampling_path(scene, i, j, sampler);
            case SamplingType::MIS:
                return mis_sampling_path(scene, i, j, sampler);
            default:
                return vec3f_zero;
        }
    }

    Vector3f PathIntegrator::brdf_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler){
        Ray ray = scene.m_cam->sample_ray(i, j);

        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;

        // Note the loop range difference
        for(Index depth=0;depth<=m_max_depth;depth++){
            auto [is_hit, info] = scene.ray_intersect(ray);

            if(!is_hit){
                return vec3f_zero;
            }

            if(scene.m_meshes[info.idx]->is_light()){
                return Peanut::EMult(current_brdf, scene.m_meshes[info.idx]->get_arealight()->radiance());
            }

            // brdf sample
            auto [recursive_dir, sampled_brdf, brdf_pdf] = scene.m_meshes[info.idx]->get_bsdf()->sample_recursive_dir(ray.m_d, sampler, info.sh_coord);
            current_brdf = Peanut::EMult(current_brdf, sampled_brdf);

            ray = Ray(info.p, recursive_dir);
        }

        return ret;
    }

    Vector3f PathIntegrator::emitter_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler){
        Ray ray = scene.m_cam->sample_ray(i, j);

        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;
        bool from_specular = true;

        for(Index depth=0;depth<m_max_depth;depth++){
            auto [is_hit, info] = scene.ray_intersect(ray);

            if(!is_hit){
                break;
            }

            if(scene.m_meshes[info.idx]->is_light()){
                if(depth == 0 || from_specular){
                    ret = ret + Peanut::EMult(scene.m_meshes[info.idx]->get_arealight()->radiance(), current_brdf);
                }
                break;
            }

            // emiiter sampling
            const bool is_current_specular = scene.m_meshes[info.idx]->get_bsdf()->is_discrete();
            if(!is_current_specular){
                auto [light, light_pdf] = scene.sample_light(sampler);
                auto [emitted_rad, light_pos, light_n, light_pos_pdf] = light->sample_contribution(scene, info.p, sampler);

                const Vector3f hitpos_to_light_world = light_pos - info.p;
                const Vector3f hitpos_to_light_world_normal = hitpos_to_light_world.normalize();
                const Float dist_square = hitpos_to_light_world.dot(hitpos_to_light_world);

                const Vector3f fr = scene.m_meshes[info.idx]->get_bsdf()->get_reflection(ray.m_d, hitpos_to_light_world.normalize(), info.sh_coord);

                const Float geo = light_n.dot(-1 * hitpos_to_light_world_normal) * info.sh_coord.m_world_n.dot(hitpos_to_light_world_normal) / dist_square;
                const Float pdf = light_pdf * light_pos_pdf;

                ret = ret + Peanut::EMult(Peanut::EMult(fr, emitted_rad), current_brdf) * geo / pdf;
            }

            // brdf sampling
            auto [recursive_dir, sampled_brdf, _] = scene.m_meshes[info.idx]->get_bsdf()->sample_recursive_dir(ray.m_d, sampler, info.sh_coord);
            current_brdf = Peanut::EMult(current_brdf, sampled_brdf);
            from_specular = scene.m_meshes[info.idx]->get_bsdf()->is_discrete();

            if(depth >= m_rr_depth){
                const Float rr = current_brdf.max();
                if(sampler.sample_1d() <= rr){
                    current_brdf = current_brdf / rr;
                }
                else{
                    break;
                }
            }

            ray = Ray(info.p, recursive_dir);
        }
        return ret;
    }


    Vector3f PathIntegrator::mis_sampling_path(const Scene &scene, Float i, Float j, Sampler &sampler){
        // WIP
        return Vector3f();
    }
}
