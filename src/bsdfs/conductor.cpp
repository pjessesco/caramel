#include <bsdf.h>

#include <common.h>
#include <logger.h>
#include <warp_sample.h>

namespace Caramel{

    Conductor::Conductor(const Conductors &mat, Float ex_ior)
    : m_ex_ior{ex_ior} {
        if(!IOR::eta_map.contains(mat) || !IOR::k_map.contains(mat)){
            CRM_ERROR("Given material is not found");
        }
        m_in_ior = IOR::eta_map.find(mat)->second;
        m_in_ior_img = IOR::k_map.find(mat)->second;
    }

    std::tuple<Vector3f, Vector3f, Float> Conductor::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const {
        const Vector3f local_outgoing{local_incoming_dir[0], local_incoming_dir[1], -local_incoming_dir[2]};
        const Vector3f n{Float0, Float0, Float1};
        const Float local_incoming_cos = n.dot(-local_incoming_dir);

        if(local_incoming_cos <= Float0){
            return {local_outgoing, vec3f_one, Float0};
        }

        return {local_outgoing, fresnel_conductor(local_incoming_cos, {m_ex_ior, m_ex_ior, m_ex_ior}, m_in_ior, m_in_ior_img), Float0};
    }

    Float Conductor::pdf(const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    Vector3f Conductor::get_reflection(const Vector3f &, const Vector3f &, const Vector2f &) const {
        return vec3f_zero;
    }

    bool Conductor::is_discrete() const {
        return true;
    }


}