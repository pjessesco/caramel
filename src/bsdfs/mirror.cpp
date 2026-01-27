#include <bsdf.h>

#include <common.h>
#include <warp_sample.h>

namespace Caramel{
    Mirror::Mirror() = default;

    std::tuple<Vector3f, Vector3f, Float> Mirror::sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &) const {
        const Vector3f local_outgoing{local_incoming_dir[0], local_incoming_dir[1], -local_incoming_dir[2]};
        return {local_outgoing, vec3f_one, Float0};
    }

    Float Mirror::pdf(const Vector3f &, const Vector3f &) const{
        return Float0;
    }

    Vector3f Mirror::get_reflection(const Vector3f &, const Vector3f &, const Vector2f &) const {
        return vec3f_zero;
    }

    bool Mirror::is_discrete() const{
        return true;
    }

}