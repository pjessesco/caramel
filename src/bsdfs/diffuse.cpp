//
// Created by Jino on 2022/09/25.
//

#include <common.h>
#include <bsdf.h>
#include <warp_sample.h>

namespace Caramel{
    Diffuse::Diffuse(const Vector3f &albedo)
        : m_albedo{albedo} {}

    std::tuple<Vector3f, Vector3f> Diffuse::sample_recursive_dir(const Vector3f &, Sampler &sampler) {
        auto [local_outgoing, dir_pdf] = sample_unit_hemisphere_cosine(sampler);
        return {local_outgoing, m_albedo};
    }

    Vector3f Diffuse::get_reflection(const Vector3f &, const Vector3f &) {
        return m_albedo * PI_INV;
    }

}