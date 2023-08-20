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

#pragma once

#include <tuple>
#include <unordered_map>

#include <common.h>

namespace Caramel{

    class Sampler;
    class Texture;

    // Commonly used functions for bsdfs

    // Snell's Law : eta_i * sin_i = eta_t * sin_t
    // Calculate `sin_t` using above equation.
    Float snell_get_sin_t(Float sin_i, Float eta_i, Float eta_t);

    // Calculate fresnel reflectance for dielectric <-> dielectric.
    // This is special case of `fresnel_conductor()` with k=0.
    Float fresnel_dielectric(Float cos_i, Float eta_i/* ex */, Float eta_t/* in */);

    // Calculate fresnel reflectance for dielectric <-> conductor
    Vector3f fresnel_conductor(Float cos_i, const Vector3f &eta_i/* ex */, const Vector3f &eta_t/* in */, const Vector3f eta_t_k);

    struct IOR{
        static constexpr Float VACUUM       = static_cast<Float>(1.0);
        static constexpr Float ICE          = static_cast<Float>(1.31);
        static constexpr Float FUSED_QUARTZ = static_cast<Float>(1.46);
        static constexpr Float GLASS        = static_cast<Float>(1.55);
        static constexpr Float SAPPHIRE     = static_cast<Float>(1.77);
        static constexpr Float DIAMOND      = static_cast<Float>(2.42);

        static const std::unordered_map<std::string, Vector3f> eta_map;
        static const std::unordered_map<std::string, Vector3f> k_map;
    };

    // Class definitions
    class BSDF{
    public:
        virtual ~BSDF() = default;
        // Given incoming dir, returns sampled recursive ray direction, reflectance * cos / pdf, and its pdf if non-discrete,
        virtual std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &uv, Sampler &sampler) const = 0;

        // calculate pdf()
        virtual Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const = 0;

        // Given incoming & outgoing dir, returns reflectance
        virtual Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &uv) const = 0;

        // Returns whether bsdf is discrete (mirror, etc) or continuous (diffuse, etc).
        virtual bool is_discrete() const = 0;

        template <typename Type, typename ...Param>
        static BSDF* Create(Param ...args){
            return dynamic_cast<BSDF*>(new Type(args...));
        }
    };

    class Diffuse final : public BSDF{
    public:
        explicit Diffuse(const Vector3f &albedo = Vector3f{Float0_5, Float0_5, Float0_5});
        explicit Diffuse(Texture *texture);
        virtual ~Diffuse();
        std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &, const Vector2f &uv, Sampler &sampler) const override;
        Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const override;
        Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &uv) const override;
        bool is_discrete() const override;
    private:
        const Vector3f m_albedo;
        const Texture *m_texture;
    };

    class Mirror final : public BSDF{
    public:
        explicit Mirror();
        std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &) const override;
        Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const override;
        Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const override;
        bool is_discrete() const override;
    };

    class Dielectric final : public BSDF{
    public:
        Dielectric(Float in_ior = IOR::GLASS, Float ex_ior = IOR::VACUUM);
        std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &) const override;
        Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const override;
        Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const override;
        bool is_discrete() const override;

    private:
        Float m_in_index_of_refraction;
        Float m_ex_index_of_refraction;
    };

    class Conductor final : public BSDF{
    public:
        Conductor(const std::string &mat, Float ex_ior);
        std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &) const override;
        Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const override;
        Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const override;
        bool is_discrete() const override;

    private:
        Vector3f m_in_ior;
        Vector3f m_in_ior_img; // Conductor has complex number IOR
        Float m_ex_ior;
    };

    class Microfacet final : public BSDF{
    public:
        Microfacet(Float alpha, Float in_ior, Float ex_ior, const Vector3f &kd);
        std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &) const override;
        Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const override;
        Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const override;
        bool is_discrete() const override;

    private:
        Float G1(const Vector3f &wv, const Vector3f &wh) const;

        Float m_alpha;
        Float m_in_index_of_refraction;
        Float m_ex_index_of_refraction;
        Float m_ks;
        Vector3f m_kd;
    };

    // Reference : https://www.pbr-book.org/3ed-2018/Reflection_Models/Microfacet_Models#OrenndashNayarDiffuseReflection
    class OrenNayar final : public BSDF{
    public:
        OrenNayar(const Vector3f &reflection, Float sigma);
        std::tuple<Vector3f, Vector3f, Float> sample_recursive_dir(const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const override;
        Float pdf(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir) const override;
        Vector3f get_reflection(const Vector3f &local_incoming_dir, const Vector3f &local_outgoing_dir, const Vector2f &) const override;
        bool is_discrete() const override;

    private:
        Vector3f m_reflection;
        Float m_A;
        Float m_B;
    };

}
