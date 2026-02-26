//
// Created by Jino Park on 2023/01/20.
//

#include <bsdf.h>

#include <common.h>
#include <warp_sample.h>

namespace Caramel{
    OrenNayar::OrenNayar(const Vector3f &reflection, Float sigma) : m_reflection{reflection} {
        const Float s = deg_to_rad(sigma);
        const Float s_2 = s * s;
        m_A = Float1 - (s_2 / (Float2 * (s_2 + static_cast<Float>(0.33))));
        m_B = static_cast<Float>(0.45) * s_2 / (s_2 + static_cast<Float>(0.09));
    }

    std::tuple<Vector3f, Vector3f, Float> OrenNayar::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {
        auto [local_outgoing_dir, pdf] = sample_unit_hemisphere_cosine(sampler);
        return {local_outgoing_dir,
                get_reflection(local_incoming_dir, local_outgoing_dir, Vector2f(/*dummy*/)) * local_outgoing_dir[2] / pdf,
                pdf};
    }

    Float OrenNayar::pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const {
        return sample_unit_hemisphere_cosine_pdf(local_outgoing_dir);
    }

    Vector3f OrenNayar::get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const {
        // from hitpoint to incoming point
        const Vector3f local_incoming_flipped = -local_incoming_dir.normalize();

        if(local_incoming_flipped[2] <= Float0 || local_outgoing_dir[2] <= Float0){
            // Not allow ray from backside
            return vec3f_zero;
        }

        Float cos_d = Float0;
        const Float sin_wo = vec_sin(local_outgoing_dir);
        const Float sin_wi = vec_sin(local_incoming_flipped);
        Float cos_max = Float0;

        if(sin_wo > Float0 && sin_wi > Float0){
            const Float sin_phi_wi = vec_sin_phi(local_incoming_flipped);
            const Float cos_phi_wi = vec_cos_phi(local_incoming_flipped);
            const Float sin_phi_wo = vec_sin_phi(local_outgoing_dir);
            const Float cos_phi_wo = vec_cos_phi(local_outgoing_dir);
            cos_d = std::max(Float0, (cos_phi_wi * cos_phi_wo) + (sin_phi_wi * sin_phi_wo));
        }

        cos_max = std::max(Float0, cos_d);

        using std::abs;
        Float sin_alpha, tan_beta;
        if(abs(local_incoming_flipped[2]) > abs(local_outgoing_dir[2])){
            sin_alpha = sin_wo;
            tan_beta = sin_wi / abs(local_incoming_flipped[2]);
        }
        else{
            sin_alpha = sin_wi;
            tan_beta = sin_wo / abs(local_outgoing_dir[2]);
        }

        const Vector3f ret = m_reflection * PI_INV * (m_A + (m_B * cos_max * sin_alpha * tan_beta));

        return ret;

    }

    bool OrenNayar::is_discrete(bool /*frontside*/) const {
        return false;
    }
}


