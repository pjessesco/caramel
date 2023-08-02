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
#include <camera.h>
#include <ray.h>
#include <shape.h>
#include <bsdf.h>
#include <sampler.h>
#include <rayintersectinfo.h>

namespace Caramel{
    DirectIntegrator::DirectIntegrator(Index spp, SamplingType sampling_type)
        : Integrator(spp), m_sampling_type(sampling_type) {}

    Vector3f DirectIntegrator::get_pixel_value(const Scene &scene, Float i, Float j, Sampler &sampler) {
        switch (m_sampling_type) {
            case SamplingType::BSDF:
                return brdf_sampling_direct(scene, i, j, sampler);
            case SamplingType::LIGHT:
                return emitter_sampling_direct(scene, i, j, sampler);
            case SamplingType::MIS:
                return mis_sampling_direct(scene, i, j, sampler);
            default:
                return vec3f_zero;
        }
    }

    Vector3f DirectIntegrator::brdf_sampling_direct(const Scene &scene, Float i, Float j, Sampler &sampler) {
        const Ray ray = scene.m_cam->sample_ray(i, j);
        const auto [is_hit, info] = scene.ray_intersect(ray);

        if(!is_hit){
            return vec3f_zero;
        }

        const auto mesh = scene.m_meshes[info.idx];
        if(mesh->is_light()){
            return mesh->get_arealight()->radiance();
        }

        const Vector3f local_ray_dir = info.sh_coord.to_local(ray.m_d);
        auto [local_outgoing, contrib, _] = mesh->get_bsdf()->sample_recursive_dir(local_ray_dir, sampler);

        const Ray recursive_ray = info.recursive_ray_to(local_outgoing);
        auto [recursive_is_hit, recursive_info] = scene.ray_intersect(recursive_ray);

        const auto recursive_mesh = scene.m_meshes[recursive_info.idx];

        if(!recursive_is_hit || !recursive_mesh->is_light()){
            return vec3f_zero;
        }

        const Vector3f rad = recursive_mesh->get_arealight()->radiance();

        return contrib % rad;
    }

    Vector3f DirectIntegrator::emitter_sampling_direct(const Scene &scene, Float i, Float j, Sampler &sampler) {
        const Ray ray = scene.m_cam->sample_ray(i, j);
        const auto [is_hit, info] = scene.ray_intersect(ray);

        if(!is_hit) {
            return vec3f_zero;
        }

        const auto mesh = scene.m_meshes[info.idx];

        if(mesh->is_light()){
            return mesh->get_arealight()->radiance();
        }

        // Direct light sampling
        auto [light, light_pdf] = scene.sample_light(sampler);
        auto [emitted_rad, light_pos, light_n_world, light_pos_pdf, light_info] = light->sample_direct_contribution(scene, info.p, sampler);

        const Vector3f hitpos_to_light_local_normal = info.sh_coord.to_local(light_pos - info.p).normalize();

        const Vector3f local_ray_dir = info.sh_coord.to_local(ray.m_d);
        const Vector3f fr = mesh->get_bsdf()->get_reflection(local_ray_dir, hitpos_to_light_local_normal);
        const Float pdf_solidangle = light->pdf_solidangle(info.p, light_info.p, light_info.sh_coord.m_world_n);

        return (fr % emitted_rad) * std::abs(hitpos_to_light_local_normal[2]) / (light_pdf * pdf_solidangle);
    }

    Vector3f DirectIntegrator::mis_sampling_direct(const Scene &scene, Float i, Float j, Sampler &sampler) {
        const Ray ray = scene.m_cam->sample_ray(i, j);
        const auto [is_hit, info] = scene.ray_intersect(ray);

        if(!is_hit){
            return vec3f_zero;
        }

        const auto mesh = scene.m_meshes[info.idx];
        const Vector3f local_ray_dir = info.sh_coord.to_local(ray.m_d);

        if(mesh->is_light()){
            return mesh->get_arealight()->radiance();
        }

        auto [light, light_pdf] = scene.sample_light(sampler);

        Vector3f L1 = vec3f_zero;
        Vector3f L2 = vec3f_zero;

        /* Light sampling */{
            auto [emitted_rad, light_pos, light_n_world, light_pos_pdf, light_info] = light->sample_direct_contribution(scene, info.p, sampler);

            // Continue if light sampling succeed
            if(emitted_rad.max() > Float0){
                const Vector3f hitpos_to_light_local_normal = info.sh_coord.to_local(light_pos - info.p).normalize();
                const Vector3f fr = mesh->get_bsdf()->get_reflection(local_ray_dir, hitpos_to_light_local_normal);
                const Float pdf_solidangle = light->pdf_solidangle(info.p, light_info.p, light_info.sh_coord.m_world_n);

                // MIS for light sampling
                const Float bsdf_pdf = mesh->get_bsdf()->pdf(local_ray_dir, hitpos_to_light_local_normal);
                L1 = (fr % emitted_rad) * std::abs(hitpos_to_light_local_normal[2]) * balance_heuristic(light_pdf * pdf_solidangle, bsdf_pdf) / (light_pdf * pdf_solidangle);
            }
        }

        // BRDF sampling is not possible if brdf is discrete
        if(mesh->get_bsdf()->is_discrete()){
            return L1;
        }

        /* BRDF sampling */{
            // Spawn ray using BRDF sampling
            auto [local_outgoing, contrib, bsdf_pdf] = mesh->get_bsdf()->sample_recursive_dir(local_ray_dir, sampler);
            const Ray recursive_ray = info.recursive_ray_to(local_outgoing);

            // BRDF sampling is not possible if spawned ray doesn't hit
            const auto [recursive_hit, recursive_info] = scene.ray_intersect(recursive_ray);
            if(!recursive_hit){
                return L1;
            }

            // BRDF sampling is meaningless if spawned ray doesn't hit light
            // TODO : consider environment map
            const auto recursive_mesh = scene.m_meshes[recursive_info.idx];
            if(!recursive_mesh->is_light()){
                return L1;
            }

            const Float light_pdf_solidangle = recursive_mesh->get_arealight()->pdf_solidangle(info.p, recursive_info.p, recursive_info.sh_coord.m_world_n);
            L2 = contrib % recursive_mesh->get_arealight()->radiance() * balance_heuristic(bsdf_pdf, light_pdf_solidangle * light_pdf);
        }

        return L1 + L2;
    }
}
