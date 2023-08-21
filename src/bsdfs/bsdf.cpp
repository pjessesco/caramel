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

#include <bsdf.h>

#include <common.h>
#include <logger.h>

namespace Caramel{
    // Debug from mitsuba3 rgb mode
    const std::unordered_map<std::string, Vector3f> IOR::eta_map{
        {"Au"/*Gold*/,   {static_cast<Float>(0.143035978), static_cast<Float>(0.375307083), static_cast<Float>(1.44204533)}},
        {"Ag"/*Silver*/, {static_cast<Float>(0.155276194), static_cast<Float>(0.116727956), static_cast<Float>(0.138387635)}},
        {"Al"/*Aluminium*/, {static_cast<Float>(1.65750086), static_cast<Float>(0.880404711), static_cast<Float>(0.521244466)}},
        {"Cu"/*Copper*/, {static_cast<Float>(0.201005474), static_cast<Float>(0.923749506), static_cast<Float>(1.10221541)}},
    };
    const std::unordered_map<std::string, Vector3f> IOR::k_map{
        {"Au"/*Gold*/, {static_cast<Float>(3.98299694), static_cast<Float>(2.38555646), static_cast<Float>(1.60335922)}},
        {"Ag"/*Silver*/, {static_cast<Float>(4.82835436), static_cast<Float>(3.12222242), static_cast<Float>(2.14690113)}},
        {"Al"/*Aluminium*/, {static_cast<Float>(9.22381114), static_cast<Float>(6.26950216), static_cast<Float>(4.83700418)}},
        {"Cu"/*Copper*/, {static_cast<Float>(3.91326213), static_cast<Float>(2.45304513), static_cast<Float>(2.14208984)}},
    };

    // Snell's Law : eta_i * sin_i = eta_t * sin_t
    // Calculate `sin_t` using above equation.
    Float snell_get_sin_t(Float sin_i, Float eta_i, Float eta_t) {
        assert(sin_i >= Float0);
        return eta_i * sin_i / eta_t;
    }

    Vector3f reflect(const Vector3f &local_incoming_dir, const Vector3f &normal){
        return local_incoming_dir + static_cast<Float>(2) * -local_incoming_dir.dot(normal) * normal;
    }

    Vector3f refract(const Vector3f &local_incoming_dir, const Vector3f &n, Float in_ior, Float ex_ior){
        const Float eta_ratio = ex_ior / in_ior;
        const Float sin_i = std::sqrt(1 - (local_incoming_dir[2] * local_incoming_dir[2]));
        const Float sin_t = snell_get_sin_t(sin_i, ex_ior, in_ior);
        const Float cos_t = std::sqrt(1 - (sin_t * sin_t));
        const Vector3f local_outgoing_dir = eta_ratio * local_incoming_dir + (eta_ratio * n.dot(-local_incoming_dir) - cos_t) * n;
        return local_outgoing_dir;
    }

    // Calculate fresnel reflectance for dielectric <-> dielectric.
    // This is special case of `fresnel_conductor()` with k=0.
    Float fresnel_dielectric(Float _cos_i, Float eta_i/* ex */, Float eta_t/* in */) {
        Float cos_i = _cos_i;
        if(_cos_i < Float0){
            cos_i = Float0;
            CRM_WARNING("cos_i(" + std::to_string(_cos_i) + ") is negative which is not allowed, clamped to 0");
        }
        if(_cos_i > Float1){
            cos_i = Float1;
            CRM_WARNING("cos_i(" + std::to_string(_cos_i) + ") exceeds 1 which is not allowed, clamped to 1");
        }

        const Float sin_i = std::sqrt(Float1 - (cos_i * cos_i));
        const Float sin_t = snell_get_sin_t(sin_i, eta_i, eta_t);

        // Total reflection
        if(sin_t >= Float1) return Float1;

        const Float cos_t = std::sqrt(Float1 - (sin_t * sin_t));

        const Float eta_t_cos_t = eta_t * cos_t;
        const Float eta_t_cos_i = eta_t * cos_i;
        const Float eta_i_cos_t = eta_i * cos_t;
        const Float eta_i_cos_i = eta_i * cos_i;

        const Float r_parallel = (eta_t_cos_i - eta_i_cos_t) / (eta_t_cos_i + eta_i_cos_t);
        const Float r_perpendicular = (eta_i_cos_i - eta_t_cos_t) / (eta_i_cos_i + eta_t_cos_t);

        return (r_parallel * r_parallel + r_perpendicular * r_perpendicular) * Float0_5;
    }

    // Calculate fresnel reflectance for dielectric <-> conductor
    Vector3f fresnel_conductor(Float _cos_i, const Vector3f &eta_i/* ex */, const Vector3f &eta_t/* in */, const Vector3f eta_t_k){
        Float cos_i = _cos_i;
        if(_cos_i < Float0){
            cos_i = Float0;
            CRM_WARNING("cos_i(" + std::to_string(_cos_i) + ") is negative which is not allowed, clamped to 0");
        }
        if(_cos_i > Float1){
            cos_i = Float1;
            CRM_WARNING("cos_i(" + std::to_string(_cos_i) + ") exceeds 1 which is not allowed, clamped to 1");
        }

        // eta + ik = nt / ni
        const Vector3f eta = div(eta_t, eta_i);
        const Vector3f etak = div(eta_t_k, eta_i);

        const Float cos_i_sq = cos_i * cos_i;
        const Float _sin_i_sq = Float1 - cos_i_sq;
        const Vector3f sin_i_sq{_sin_i_sq, _sin_i_sq, _sin_i_sq};
        const Vector3f sin_i_4 = sin_i_sq % sin_i_sq;
        const Vector3f eta_sq = eta % eta;
        const Vector3f etak_sq = etak % etak;

        // Calculate a^2+b^2
        const Vector3f tmp1 = eta_sq - etak_sq - sin_i_sq;
        const Vector3f tmp2 = static_cast<Float>(4) * eta_sq % etak_sq;
        const Vector3f a2b2 = sqrt(tmp1%tmp1 + tmp2);
        // See 'Physically Based Lighting Calculations for Computer Graphics' from Peter Shirley
        const Vector3f a = sqrt(Float0_5 * (a2b2 + tmp1));

        Vector3f r_perpendicular;
        {
            const Vector3f _cos_i_sq{cos_i_sq, cos_i_sq, cos_i_sq};
            r_perpendicular = div(a2b2 + _cos_i_sq - (Float2 * a * cos_i),
                                  a2b2 + _cos_i_sq + (Float2 * a * cos_i));
        }

        const Vector3f r_parallel = r_perpendicular % div((cos_i_sq * a2b2) + sin_i_4 - (Float2 * cos_i * a % sin_i_sq),
                                                          (cos_i_sq * a2b2) + sin_i_4 + (Float2 * cos_i * a % sin_i_sq));

        return ((r_parallel % r_parallel) + (r_perpendicular % r_perpendicular)) * Float0_5;
    }




}