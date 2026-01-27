#include <bsdf.h>

#include <common.h>
#include <warp_sample.h>
#include <textures.h>

namespace Caramel{
    Diffuse::Diffuse(const Vector3f &albedo)
        : m_albedo{albedo}, m_texture{nullptr} {}

    Diffuse::Diffuse(Texture *texture)
        : m_albedo{vec3f_zero}, m_texture{texture} {}

    Diffuse::~Diffuse(){
        delete m_texture;
    }

    std::tuple<Vector3f, Vector3f, Float> Diffuse::sample_recursive_dir(const Vector3f &, const Vector2f &uv, Sampler &sampler) const {
        auto [local_outgoing, dir_pdf] = sample_unit_hemisphere_cosine(sampler);
        return {local_outgoing, get_reflection(vec3f_zero, vec3f_zero, uv) * PI_INV, dir_pdf};
    }

    Float Diffuse::pdf(const Vector3f &, const Vector3f &local_outgoing_dir) const{
        return sample_unit_hemisphere_cosine_pdf(local_outgoing_dir);
    }

    Vector3f Diffuse::get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &uv) const {
        // from hitpoint to incoming point
        const Vector3f local_incoming_flipped = -local_incoming_dir.normalize();

        if(local_incoming_flipped[2] <= Float0 || local_outgoing_dir[2] <= Float0){
            // Not allow ray from backside
            return vec3f_zero;
        }

        return (m_texture == nullptr ? m_albedo : m_texture->get_val(uv)) * PI_INV;
    }

    bool Diffuse::is_discrete() const{
        return false;
    }

}