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
#include <progress.h>
#include <scene.h>

namespace Caramel{
    DirectIntegrator::DirectIntegrator(const Scene &scene) : Integrator(scene) {}

    // Different with albedo precisely...
    Vector3f DirectIntegrator::get_pixel_value(Float i, Float j, Sampler &sampler) {
        const Ray ray = m_scene.m_cam.sample_ray(i, j);
        auto [is_hit, info] = m_scene.ray_intersect(ray);

        if(!is_hit){
            return Vector3f{Float0, Float0, Float0};
        }

        if(m_scene.m_meshes[info.idx]->is_light()){
            return m_scene.m_meshes[info.idx]->m_arealight->radiance();
        }

        // Direct light sampling
        if constexpr(true){
            auto [light, light_pdf] = m_scene.sample_light(sampler);

            auto [emitted_rad, light_pos, light_n, light_pos_pdf] = light->sample_contribution(info.p, sampler);

            const Vector3f local_incoming_dir = info.sh_coord.to_local(ray.m_d);
            const Vector3f hitpos_to_light_world = light_pos - info.p;
            const Vector3f hitpos_to_light_world_normal = hitpos_to_light_world.normalize();
            const Float dist_square = hitpos_to_light_world.dot(hitpos_to_light_world);
            const Vector3f local_outgoing_dir = info.sh_coord.to_local(hitpos_to_light_world.normalize());

            Vector3f fr = m_scene.m_meshes[info.idx]->m_bsdf->get_reflection(local_incoming_dir, local_outgoing_dir);

            Float geo = light_n.dot(-1 * hitpos_to_light_world_normal) * info.sh_coord.m_world_n.dot(hitpos_to_light_world_normal) / dist_square;
            Float pdf = light_pdf * light_pos_pdf;

            return mult_ewise(fr, emitted_rad) * geo / pdf;
        }

        // BRDF sampling
        else{
            auto [local_outgoing, contrib] = m_scene.m_meshes[info.idx]->m_bsdf->sample_recursive_dir(info.sh_coord.to_local(ray.m_d), sampler);

            const Ray recursive_ray(info.p, info.sh_coord.to_world(local_outgoing));

            auto [recursive_is_hit, recursive_info] = m_scene.ray_intersect(recursive_ray);

            if(!recursive_is_hit || !m_scene.m_meshes[recursive_info.idx]->is_light()){
                return vec3f_zero;
            }

            Vector3f rad = m_scene.m_meshes[recursive_info.idx]->m_arealight->radiance();

            return mult_ewise(contrib, rad);
        }
    }
}
