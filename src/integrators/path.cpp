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
    PathIntegrator::PathIntegrator(const Scene &scene, Index max_depth) : Integrator(scene), m_max_depth{max_depth} {}

    // Different with albedo precisely...
    Vector3f PathIntegrator::get_pixel_value(Float i, Float j, Sampler &sampler) {
        return brdf_sampling_path(i, j, sampler);
    }

    Vector3f PathIntegrator::brdf_sampling_path(Float i, Float j, Sampler &sampler){
        Ray ray = m_scene.m_cam.sample_ray(i, j);
        RayIntersectInfo info;

        Vector3f current_brdf = vec3f_one;
        Vector3f ret = vec3f_zero;

        for(Index depth=0;depth<m_max_depth;depth++){
            bool is_hit;
            std::tie(is_hit, info) = m_scene.ray_intersect(ray);

            if(!is_hit){
                return vec3f_zero;
            }

            if(m_scene.m_meshes[info.idx]->is_light()){
                return mult_ewise(current_brdf, m_scene.m_meshes[info.idx]->m_arealight->radiance());
            }

            // brdf sample
            auto [recursive_dir, sampled_brdf] = m_scene.m_meshes[info.idx]->m_bsdf->sample_recursive_dir(ray.m_d, sampler, info.sh_coord);
            current_brdf = mult_ewise(current_brdf, sampled_brdf);

            ray = Ray(info.p, recursive_dir);
        }

        return ret;
    }


    Vector3f PathIntegrator::emitter_sampling_path(Float i, Float j, Sampler &sampler){
        return vec3f_zero;
    }

}
