# Microfacet Rough BSDF: 이론, 코드 비교, Caramel 구현 가이드

> **참고 자료:**
> - [pbr-book 4ed: Roughness Using Microfacet Theory](https://www.pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory)
> - [pbr-book 4ed: Rough Dielectric BSDF](https://www.pbr-book.org/4ed/Reflection_Models/Rough_Dielectric_BSDF)
> - [pbr-book 4ed: Conductor BRDF](https://www.pbr-book.org/4ed/Reflection_Models/Conductor_BRDF)
> - pbrt-v4 소스코드 (`src/pbrt/util/scattering.h`, `src/pbrt/bxdfs.h`, `src/pbrt/bxdfs.cpp`)
> - mitsuba3 소스코드 (`include/mitsuba/render/microfacet.h`, `src/bsdfs/roughconductor.cpp`, `src/bsdfs/roughdielectric.cpp`)

---

## 목차

1. [Microfacet 이론 기초](#1-microfacet-이론-기초)
2. [분포 함수 D(omega_m)](#2-분포-함수-domega_m)
3. [마스킹-섀도잉 함수 G](#3-마스킹-섀도잉-함수-g)
4. [가시 노말 분포와 샘플링](#4-가시-노말-분포와-샘플링)
5. [Torrance-Sparrow BRDF (반사)](#5-torrance-sparrow-brdf-반사)
6. [Conductor BRDF (금속 반사)](#6-conductor-brdf-금속-반사)
7. [Rough Dielectric BSDF (유전체 반사+투과)](#7-rough-dielectric-bsdf-유전체-반사투과)
8. [코드 비교: pbrt-v4 vs mitsuba3 vs caramel](#8-코드-비교-pbrt-v4-vs-mitsuba3-vs-caramel)
9. [Caramel 구현 가이드](#9-caramel-구현-가이드)

---

## 1. Microfacet 이론 기초

### 1.1 풀고자 하는 문제

현실의 대부분의 표면은 **완벽한 거울도, 완벽한 난반사도 아닌** 중간 상태이다.
금속 냄비의 뿌옇게 비치는 반사, 젖은 아스팔트의 번들거림, 유리의 서리 낀 모습 —
이 모든 것은 표면의 **미세한 거칠기(roughness)**에 의해 발생한다.

이 거칠기를 어떻게 렌더링에 반영할 것인가? 이것이 microfacet 이론이 해결하는 문제이다.

### 1.2 핵심 아이디어

거친(rough) 표면을 **수많은 작은 평면(microfacet)의 집합**으로 모델링한다.
각 마이크로면은 개별적으로 완벽한 거울(또는 완벽한 유전체 경계)로 동작하지만,
노말 방향의 **통계적 분포**에 의해 전체적으로 흐릿한(glossy) 반사/투과가 나타난다.

```
거친 표면 (확대)
  /\  /\    /\
 /  \/  \  /  \    ← 각각의 작은 면(microfacet)
/        \/    \      법선 방향 ω_m이 각기 다름
──────────────────  ← 매크로 표면 (법선 n)
```

직관적으로: "표면이 완벽히 매끄러우면 모든 마이크로면의 법선이 n과 같아서 완벽 거울이 되고,
거칠수록 법선이 다양한 방향으로 퍼지면서 반사가 흐릿해진다."

### 1.3 왜 Microfacet인가? — 다른 접근법의 한계

**접근 1: 실제 기하를 메시로 표현**

마이크로미터 단위의 요철을 삼각형으로 모델링하면 물리적으로 정확하겠지만,
저장과 트레이싱 비용이 천문학적이다. 실용적이지 않다.

**접근 2: 완벽 거울/유전체 (Dirac delta BSDF)**

caramel의 현재 `Mirror`, `Conductor`, `Dielectric`이 이 방식이다.
BSDF가 Dirac delta 함수이므로 특정 방향으로만 빛이 나간다.
이것은 수학적으로 **`f()` 함수를 평가할 수 없다**는 것을 의미한다
— BSDF 값이 특정 방향에서 무한대이고 나머지에서 0이기 때문이다.

이 제약이 왜 문제인가? path tracing에서 **light sampling(NEE, Next Event Estimation)**을
사용하려면 임의의 (ω_o, ω_i) 쌍에 대해 `f(ω_o, ω_i)`를 평가할 수 있어야 한다.
delta BSDF에서는 이것이 불가능하므로, light sampling을 사용할 수 없고
오직 BSDF sampling에만 의존해야 한다. 이는 작은 광원 아래에서 극심한 노이즈를 만든다.

**접근 3: Microfacet 통계 모델 (이 장에서 다루는 방법)**

미세 기하를 직접 표현하지 않고, 마이크로면 법선의 **통계적 분포**만 모델링한다.
이 분포가 부드러운 함수이므로:

- `f()` 평가가 가능하다 → light sampling 전략 사용 가능 → MIS로 노이즈 감소
- 효율적인 중요도 샘플링이 가능하다 → 적은 샘플로 깨끗한 이미지
- 파라미터 하나(α)로 smooth ↔ rough를 연속적으로 제어할 수 있다

| 접근 | BSDF | f() 평가 | Light sampling | 실용성 |
|------|------|----------|----------------|--------|
| 실제 메시 | 정확 | 가능 | 가능 | 비용 불가 |
| Delta BSDF | 정확 | **불가** | **불가** | 제한적 |
| **Microfacet** | 근사 | **가능** | **가능** | 실용적 |

### 1.4 세 가지 기하학적 효과 (pbr-book Figure 9.21)

```
(a) Masking              (b) Shadowing           (c) Interreflection
    관찰자                    광원                     광원
     ↓                       ↓                       ↓
  /\ ■ /\               /\ ■ /\                /\   /\
 /  \|/  \             /  \|/  \              / ↗\ ↙/ \
/    |    \           /    |    \            /  ↗ \↙   \
```

- **Masking**: 다른 마이크로면에 의해 관찰자에게서 가려짐
- **Shadowing**: 다른 마이크로면에 의해 광원으로부터 가려짐
- **Interreflection**: 마이크로면 사이에서 빛이 여러 번 반사 (대부분의 모델에서 무시됨)

현재 널리 쓰이는 모델(pbrt-v4, mitsuba3 포함)은 interreflection을 무시한다. 이로 인해 roughness가 높을수록 실제보다 **약간 어둡게** 렌더링되는 한계가 있다.

왜 무시하는가? interreflection을 정확히 계산하려면 마이크로면 사이의 다중 산란(multiple scattering)을 시뮬레이션해야 하는데, 이는 마이크로면의 실제 기하를 알아야 가능하다. microfacet 모델은 애초에 기하를 통계로 대체한 것이기 때문에, 이 정보가 없다. 최근 연구(Heitz 2016, "Multiple-Scattering Microfacet BSDFs with the Smith Model")가 이를 근사적으로 보상하는 방법을 제시하고 있지만, 아직 표준 구현에는 포함되지 않은 경우가 많다.

---

## 2. 분포 함수 D(omega_m)

### 2.1 D가 필요한 이유

microfacet 모델의 핵심 질문은 "마이크로면 법선이 어떤 방향 분포를 따르는가?"이다.
이 분포가 BRDF의 모양을 결정한다:

- 법선이 매크로 법선 n 근처에 집중되면 → 날카로운 specular highlight (매끈한 표면)
- 법선이 넓게 퍼져 있으면 → 넓고 흐린 highlight (거친 표면)

`D(ω_m)`은 이 분포를 수학적으로 표현한다.

### 2.2 정의

`D(ω_m)`은 마이크로면 법선이 `ω_m` 방향인 마이크로면의 **상대적 차분 면적(relative differential area)**을 나타낸다.

완벽 거울이면: `D(ω_m) = δ(ω_m − n)` — 모든 법선이 n 방향, Dirac delta.

### 2.3 정규화 조건 (Eq. 9.15)

D는 아무 함수나 될 수 없다. 물리적으로 타당하려면 **마이크로면들이 매크로 표면을 정확히 덮어야 한다**.

마이크로면을 매크로 표면 법선 방향으로 투영하면, 총 면적이 매크로 면적 `dA`와 같아야 한다:

```
∫_{H²(n)} D(ω_m) · cos θ_m · dω_m = 1
```

이 조건이 없으면 어떻게 되는가? 적분이 1보다 크면 에너지가 창조되고(표면이 받은 것보다 더 많이 반사), 1보다 작으면 에너지가 사라진다. 정규화 조건은 **에너지 보존의 필요 조건**이다.

직관적으로: "위에서 내려다볼 때, 마이크로면들이 매크로 표면을 정확히 덮는다 — 빈틈도, 겹침도 없이."

### 2.4 Beckmann 분포

Gaussian 랜덤 표면에서 유도된 분포:

```
D_Beckmann(ω_m) = exp(-tan²θ_m / α²) / (π · α² · cos⁴θ_m)
```

- **물리적 유래**: 표면 높이가 Gaussian 분포를 따른다고 가정하면 자연스럽게 유도됨
- **꼬리(tail)**: 짧음 — 큰 각도에서 빠르게 감소 (exponential decay)
- **한계**: 실제 측정된 표면은 Beckmann보다 grazing angle 근처에서 더 많은 마이크로면이 관찰됨
- **Caramel 현재 구현**: `warp_sample.h:111`에서 이 분포만 사용

### 2.5 Trowbridge-Reitz (GGX) 분포 (Eq. 9.16)

#### 왜 GGX가 필요한가?

Beckmann은 수학적으로 깔끔하지만, **실제 표면의 반사 패턴과 맞지 않는** 경우가 많다.
특히 specular highlight의 가장자리가 너무 급격하게 사라진다.
실제 표면(금속, 플라스틱 등)을 측정하면 highlight 주변에 부드럽게 퍼지는 "halo"가 관찰되는데,
이것은 grazing angle 근처에서 마이크로면 밀도가 Beckmann 예측보다 높기 때문이다.

GGX(Trowbridge-Reitz)는 이 **긴 꼬리(long tail)**를 자연스럽게 포함하는 분포이다.

타원체(ellipsoid) 표면의 법선 분포에서 유도:

```
                              1
D_TR(ω_m) = ─────────────────────────────────────────────
             π · α_x · α_y · cos⁴θ_m · (1 + e)²

             where  e = tan²θ_m · (cos²φ_m/α_x² + sin²φ_m/α_y²)
```

**등방성(isotropic) 케이스** (`α_x = α_y = α`):

```
                              1
D_TR(ω_m) = ─────────────────────────────────
             π · α² · cos⁴θ_m · (1 + tan²θ_m/α²)²
```

#### 파라미터

| 파라미터 | 의미 |
|----------|------|
| `α_x, α_y` | 접선 방향별 거칠기 (anisotropy 제어) |
| `α → 0` | 완벽 거울에 수렴 |
| `α_x = α_y` | 등방성 |
| `α ≈ 0.3` | 상당히 거친 표면 |

#### Roughness → Alpha 변환 (pbrt-v4)

```cpp
static Float RoughnessToAlpha(Float roughness) {
    return std::sqrt(roughness);
}
```

#### Beckmann vs GGX 비교

```
   D(ω_m)
   ↑
   │    *
   │   * *        ── Beckmann (α=0.5)
   │  *   *       ── GGX (α=0.5)
   │ *     *
   │*       **
   │*        ****  ← GGX가 이 영역(tail)에서 더 높음
   │*            ********___
   └────────────────────────→ θ_m
```

GGX는 **긴 꼬리(long tail)**를 가져서 grazing angle 근처에서도 마이크로면 밀도가 상당하다. 이것이 실제 표면 측정과 더 잘 맞기 때문에 현대 렌더러에서 GGX를 선호한다.

### 2.6 Beckmann과 GGX의 관계 — 추상화

Beckmann과 GGX는 서로 다른 수식이지만 **완전히 동일한 역할**을 한다:

```
MicrofacetDistribution (추상적 역할)
  ├── D(ω_m)     "법선이 ω_m인 마이크로면이 얼마나 있는가"
  ├── Λ(ω)       "방향 ω에서의 masking 보조 함수"
  └── Sample(ω)  "가시 노말을 하나 뽑아라"
```

이 세 함수의 **시그니처와 의미**가 동일하고, **내부 수식만 다르다**:

| 메서드 | Beckmann | GGX |
|--------|----------|-----|
| `D(ω_m)` | `exp(-tan²θ/α²) / (π·α²·cos⁴θ)` | `1 / (π·α²·cos⁴θ·(1+tan²θ/α²)²)` |
| `Λ(ω)` | erf 기반 → rational approx | `(√(1+α²tan²θ)-1)/2` (해석적) |
| `Sample_wm` | erfinv 수치역산 | disk warp (Heitz 2018) |

나머지 — `G1`, `G`, `PDF` — 는 위 세 함수의 **조합**으로 정의되므로 분포에 무관하다:

```
G1(ω)       = 1 / (1 + Λ(ω))                           ← Λ만 사용
G(ω_o, ω_i) = 1 / (1 + Λ(ω_o) + Λ(ω_i))               ← Λ만 사용
PDF(ω, ω_m) = G1(ω) / |cosθ| · D(ω_m) · |ω · ω_m|     ← D, G1 조합
```

이것은 **전략 패턴(Strategy Pattern)**에 해당한다. mitsuba3이 정확히 이 구조를 사용한다:

```cpp
// mitsuba3: 하나의 클래스, enum으로 분기
class MicrofacetDistribution {
    MicrofacetType m_type;  // Beckmann or GGX

    Float eval(const Vector3f &m) const {
        if (m_type == MicrofacetType::Beckmann) { /* Beckmann D */ }
        else                                     { /* GGX D */     }
    }
    Float smith_g1(const Vector3f &v, const Vector3f &m) const {
        if (m_type == MicrofacetType::Beckmann) { /* rational approx */ }
        else                                     { /* closed-form */    }
    }
    // G, pdf, sample은 eval/smith_g1의 조합 → 분기 불필요
};
```

이렇게 추상화하면 `RoughConductor`와 `RoughDielectric`은 **분포 타입을 전혀 알 필요가 없다**:

```cpp
class RoughConductor final : public BSDF {
    MicrofacetDistribution m_distrib;  // Beckmann이든 GGX이든 상관없음
    // ... D·F·G / (4·cosθ_i·cosθ_o) 공식은 동일
};
```

---

## 3. 마스킹-섀도잉 함수 G

### 3.1 G가 필요한 이유

D만으로 BRDF를 만들면 어떻게 될까? "법선이 ω_m 방향인 마이크로면이 얼마나 있는지"만 알면 반사를 계산할 수 있을 것 같지만, **실제로는 그렇지 않다**.

마이크로면은 서로를 가린다. 거친 표면을 옆에서(grazing angle) 보면, 앞쪽 마이크로면이 뒤쪽을 가려서 많은 면이 보이지 않는다. 이 효과를 무시하면:

- **Grazing angle에서 에너지가 폭발한다**: D/cosθ 형태의 항이 cosθ→0에서 발산
- **결과적으로 물리적으로 불가능한 밝기**가 나타남 (에너지 보존 위반)

G 함수는 "마이크로면 중 실제로 **가려지지 않는 비율**"을 나타내어, 이 에너지 발산을 억제한다. 특히 grazing angle에서 G→0이 되면서 D의 발산을 상쇄해준다.

**요약: D가 "마이크로면이 얼마나 있는가"라면, G는 "그 중 실제로 기여하는 것이 얼마인가"이다.**

### 3.2 G1 — 단방향 마스킹 함수

`G1(ω)`은 방향 `ω`에서 볼 때 **가려지지 않는** 마이크로면의 비율 (`0 ≤ G1 ≤ 1`).

**정규화 조건 (Eq. 9.17):**

```
∫_{H²(n)} D(ω_m) · G1(ω, ω_m) · max(0, ω · ω_m) · dω_m = cos θ
```

좌변: 가시 마이크로면의 투영 면적 합. 우변: 매크로 표면의 투영 면적.

이 조건의 의미: 어떤 방향에서 보든, **보이는 마이크로면의 투영 면적 합은 매크로 표면의 투영 면적과 같아야 한다**. 보이지 않는 면만큼 다른 면이 더 보여야 하므로, 총합은 보존된다.

### 3.3 Smith 근사 — 왜 근사가 필요한가

정확한 G1(ω, ω_m)을 구하려면 마이크로면의 실제 기하학적 배치를 알아야 한다 — 어떤 면이 어떤 면을 가리는지는 배치에 따라 다르기 때문이다. 하지만 microfacet 모델은 기하를 통계로 대체했으므로 이 정보가 없다.

**Smith의 핵심 가정**: 표면 높이와 법선이 통계적으로 독립이다.

이 가정은 "높은 곳에 있는 마이크로면이라고 해서 특별한 법선 방향을 가지지 않는다"는 뜻이다. 이 가정 하에서 G1은 `ω_m`에 의존하지 않게 되어 (backfacing 제외) 적분에서 빼낼 수 있다:

```
G1(ω) = cos θ / ∫_{H²(n)} D(ω_m) · max(0, ω · ω_m) · dω_m    (Eq. 9.18)
```

이로써 G1은 **D만의 함수**가 되어, D를 정하면 G1이 자동으로 결정된다. 매우 과감한 단순화이지만, 시뮬레이션 및 실측과 잘 맞는 것으로 검증되었다.

### 3.4 Lambda 함수 (Eq. 9.19, 9.20)

G1을 직접 계산하면 적분이 필요하다. 이를 피하기 위해 보조 함수 `Λ(ω)`를 도입한다. Λ는 Eq. 9.18의 적분을 해석적으로 풀어서 얻는 것으로, D의 구체적인 형태(Beckmann, GGX 등)에 따라 달라진다.

Smith masking을 보조 함수 `Λ(ω)`로 표현:

```
G1(ω) = 1 / (1 + Λ(ω))    (Eq. 9.19)
```

**Trowbridge-Reitz (GGX)의 Λ** (Eq. 9.20, isotropic):

```
Λ(ω) = (√(1 + α² · tan²θ) − 1) / 2
```

**비등방성(anisotropic) 일반화** (Eq. 9.21):

방향 `ω`의 방위각 `φ`에 대해 보간된 roughness를 사용:

```
α_interp = √(α_x² · cos²φ + α_y² · sin²φ)
```

이 `α_interp`를 isotropic Λ 공식에 대입한다.

**Beckmann의 G1** (caramel 현재 구현, rational approximation):

Beckmann 분포의 Λ를 해석적으로 구하면 erf(error function)가 나와서 계산 비용이 높다. 그래서 rational function(유리 함수)으로 근사한다:

```cpp
// caramel microfacet.cpp:107
Float G1(const Vector3f &wv, const Vector3f &wh) const {
    if (wv.dot(wh) / wv[2] <= 0) return 0;  // backfacing check
    Float b = wv[2] / (sqrt(wv[0]*wv[0] + wv[1]*wv[1]) * m_alpha);  // 1/(α·tanθ)
    if (b >= 1.6) return 1;  // 거의 수직 → masking 없음
    Float b2 = b * b;
    return (3.535*b + 2.181*b2) / (1 + 2.276*b + 2.577*b2);
}
```

vs **GGX의 G1** (해석적, closed-form — rational approx 불필요):

GGX의 장점 중 하나: Λ가 깔끔한 해석적 형태를 가진다 (erf 불필요):

```cpp
Float G1(const Vector3f &w) const {
    return 1 / (1 + Lambda(w));
}
// Λ(ω) = (√(1 + α²·tan²θ) − 1) / 2
```

### 3.5 양방향 마스킹-섀도잉 G(ω_o, ω_i) — 왜 G1 두 번이 아닌가

BRDF에서는 한 방향(관찰자)이 아니라 **두 방향(관찰자 + 광원)** 모두에서 가려지지 않아야 한다. 즉, 마이크로면이 빛을 받을 수 있고(shadowing이 없고) 동시에 관찰자에게 보여야(masking이 없어야) 한다.

가장 단순한 접근은 두 방향의 masking을 독립으로 가정하여 곱하는 것이다:

```
G_uncorrelated(ω_o, ω_i) = G1(ω_o) · G1(ω_i)
```

하지만 이것은 **이중 계수(double counting)** 문제가 있다. 높은 곳에 있는 마이크로면은 두 방향 모두에서 보일 가능성이 높은데, 독립 가정은 이 상관관계를 무시하여 가려지는 비율을 과대 추정한다. 결과적으로 표면이 **실제보다 어둡게** 렌더링된다.

**Height-correlated 모델** (Eq. 9.22)은 이 상관관계를 반영한다:

```
G(ω_o, ω_i) = 1 / (1 + Λ(ω_o) + Λ(ω_i))
```

| 모델 | 수식 | 문제 |
|------|------|------|
| Uncorrelated | `G1(ω_o) · G1(ω_i)` | 이중 계수 → 너무 어두움 |
| **Height-correlated** (Eq. 9.22) | `1 / (1 + Λ(ω_o) + Λ(ω_i))` | 상관 반영 → 더 정확 |

Caramel 현재 `microfacet.cpp`: `G1(wi, wh) * G1(wo, wh)` → uncorrelated.
pbrt-v4 / mitsuba3: height-correlated.

---

## 4. 가시 노말 분포와 샘플링

### 4.1 왜 D만으로 샘플링하면 안 되는가

Monte Carlo 렌더링에서 microfacet BRDF를 효율적으로 계산하려면, 마이크로면 법선 ω_m을 **중요도 샘플링(importance sampling)**해야 한다. 가장 단순한 방법은 D(ω_m)에 비례하여 법선을 뽑는 것이다.

문제는: D(ω_m)은 **모든 방향의 마이크로면**을 포함한다. 관찰자 방향 ω_o에서 **뒤를 향하는(backfacing)** 마이크로면도 샘플될 수 있다. 이런 법선은 반사 방향이 표면 아래로 가므로 무효 샘플이 되어 버려진다.

특히 grazing angle(거의 옆에서 보는 경우)에서는 **대부분의 마이크로면이 뒤를 향하고 있어** D 기반 샘플링의 대다수가 낭비된다. 이것이 높은 분산(노이즈)의 원인이다.

### 4.2 가시 노말 분포 D_ω (Eq. 9.23)

해결책: D 대신, 방향 `ω`에서 **실제로 보이는 마이크로면만**의 법선 분포를 정의한다:

```
D_ω(ω_m) = G1(ω) / cos θ · D(ω_m) · max(0, ω · ω_m)
```

각 항의 역할:
- `D(ω_m)`: 법선이 ω_m인 마이크로면이 얼마나 있는가
- `max(0, ω · ω_m)`: ω 방향에서 바라본 투영 면적 (뒤를 향하면 0)
- `G1(ω)`: masking에 의해 가려지지 않는 비율
- `1/cos θ`: 정규화 상수 (∫ D_ω dω_m = 1이 되도록)

이것은 다음을 만족하는 정규화된 확률 밀도이다:

```
∫ D_ω(ω_m) dω_m = 1
```

### 4.3 비가시 vs 가시 노말 샘플링 비교

```
(a) D(ω_m) 기반 샘플링              (b) D_ω(ω_m) 기반 샘플링
    (비가시, caramel 현재)               (가시, pbrt-v4/mitsuba3)

   모든 방향의 법선 샘플              보이는 법선만 샘플
   → 뒷면 법선도 포함 → 버려짐        → 낭비 없음
   → grazing angle에서 대부분 무효     → grazing에서도 효율적
   → 분산 높음, 노이즈 많음           → 분산 낮음, 깨끗
```

Caramel의 현재 `sample_beckmann_distrib()`은 비가시 노말 샘플링(non-visible)이다.
pbrt-v4와 mitsuba3(기본값)은 가시 노말 샘플링(visible normal sampling)을 사용한다.

### 4.4 가시 노말 샘플링 알고리즘 (pbrt-v4 Sample_wm)

D_ω(ω_m)에서 직접 샘플링하는 것은 해석적 역함수가 없어서 어렵다. 대신 기하학적 트릭을 사용한다.

**핵심 아이디어**: microfacet 표면을 타원체의 집합으로 생각하면, 가시 노말 분포는 "ω 방향에서 타원체를 향해 평행 광선을 쏴서 교차점의 법선을 모은 것"과 같다. 비등방성 타원체 문제를 α로 스케일링하여 **등방성 반구 문제로 변환**하면, 반구 위에서 균일 디스크 샘플링으로 풀 수 있다.

```
Step 1: Stretch (늘리기)
  wh = normalize(α_x·w.x, α_y·w.y, w.z)
  if (wh.z < 0) wh = -wh

  → 타원체를 반구로 변환하는 스케일링

Step 2: 직교 기저 구성
  T1 = (wh.z < 0.99999) ? normalize(cross((0,0,1), wh))
                         : (1,0,0)
  T2 = cross(wh, T1)

Step 3: 균일 디스크 샘플링
  p = SampleUniformDiskPolar(u)

Step 4: 가시성 보정 (warping)
  h = √(1 − p.x²)
  p.y = lerp((1 + wh.z)/2, h, p.y)

  → 비수직 투영에 의한 y축 스케일링을 보상
  → cos θ 에 비례하는 변환

Step 5: 반구에 재투영 & 역변환
  pz = √(max(0, 1 − p.x² − p.y²))
  nh = p.x·T1 + p.y·T2 + pz·wh

  return normalize(α_x·nh.x, α_y·nh.y, max(1e-6, nh.z))
```

**Step 5의 미묘한 포인트**: 역변환에서 `α`를 **곱한다** (나누지 않음).
이는 법선 변환이 역전치(inverse-transpose) 성질을 따르기 때문이다.

### 4.5 가시 노말 PDF

가시 노말 분포 자체가 이미 정규화된 확률 밀도이므로, PDF는 그대로 D_ω이다:

```
PDF(ω, ω_m) = D_ω(ω_m)
            = G1(ω) / |cos θ| · D(ω_m) · |ω · ω_m|
```

pbrt-v4에서:

```cpp
Float PDF(Vector3f w, Vector3f wm) const {
    return D(w, wm);  // D(w, wm)이 곧 D_ω(ω_m)
}
Float D(Vector3f w, Vector3f wm) const {
    return G1(w) / AbsCosTheta(w) * D(wm) * AbsDot(w, wm);
}
```

---

## 5. Torrance-Sparrow BRDF (반사)

### 5.1 D, F, G를 어떻게 조합하여 BRDF를 만드는가

지금까지 세 가지 도구를 준비했다:
- **D**: 마이크로면 법선 분포 (어떤 법선이 얼마나 있는가)
- **F** (Fresnel): 개별 마이크로면에서 빛이 얼마나 반사되는가
- **G**: 마이크로면 중 실제로 가려지지 않는 비율

이제 이 세 요소를 **올바르게 조합하여** BRDF를 유도해야 한다. 단순히 D·F·G가 아니다 — 정확한 BRDF를 얻으려면 변수 변환(Jacobian)과 투영(cos θ)을 올바르게 처리해야 한다. 이것이 Torrance-Sparrow 모델이다.

### 5.2 Half-vector — 왜 필요한가 (Eq. 9.25)

각 마이크로면은 완벽한 거울이므로, 출사 방향 ω_o에서 온 빛이 입사 방향 ω_i로 반사되려면, 마이크로면 법선이 **ω_o와 ω_i의 정확히 중간 방향**을 가리켜야 한다:

```
ω_m = (ω_i + ω_o) / ‖ω_i + ω_o‖    (Eq. 9.25)
```

이것이 **half-vector(절반 벡터)**이다. microfacet BRDF에서 가장 핵심적인 관계 — 주어진 (ω_o, ω_i) 쌍에 대해, 빛의 전달에 기여하는 마이크로면은 **오직 법선이 ω_m인 것들 뿐**이다.

### 5.3 변수 변환 Jacobian — 왜 필요한가 (Eq. 9.27)

우리는 D(ω_m) — 마이크로면 법선의 분포 — 를 알고 있다. 하지만 BRDF와 PDF는 **ω_i 공간**에서 정의된다. 따라서 ω_m 공간의 확률 밀도를 ω_i 공간의 확률 밀도로 **변환**해야 한다.

이 변환의 비율이 Jacobian이다. ω_m의 입체각과 ω_i의 입체각 사이 관계:

```
dω_m / dω_i = 1 / (4 · (ω_o · ω_m))
```

**유도 과정:**
1. 반사 법칙: `θ_i = 2θ_m` (ω_o 기준 좌표계에서)
2. `φ_i = φ_m` (반사면 내에서 방위각 동일)
3. `dω = sin θ · dθ · dφ` 이므로:
   - `dω_i = sin(2θ_m) · 2dθ_m · dφ_m = 4·cos θ_m · sin θ_m · dθ_m · dφ_m`
   - `dω_m = sin θ_m · dθ_m · dφ_m`
   - 비율: `dω_m/dω_i = 1 / (4·cos θ_m) = 1 / (4·(ω_o · ω_m))`

### 5.4 PDF (Eq. 9.28)

```
p(ω_i) = D_ωo(ω_m) / (4 · (ω_o · ω_m))
```

의미: "가시 노말 분포에서 ω_m을 샘플하고, 그 ω_m 기준으로 반사하여 ω_i를 얻었을 때의 확률 밀도."

D_ωo(ω_m)은 ω_m 공간의 밀도이고, `1/(4·(ω_o · ω_m))`은 ω_m → ω_i 변환의 Jacobian이다.

### 5.5 BRDF 유도 (Eq. 9.29 → 9.33) — 왜 D·F·G/(4cosθ_i·cosθ_o)인가

최종 BRDF 공식이 왜 이런 형태인지를 이해하는 것이 핵심이다.
Monte Carlo 추정기에서 출발한다. 샘플링 과정을 따라가보자:

```
f_r · L_i · |cos θ_i| / p(ω_i) = F(ω_o · ω_m) · G1(ω_i) · L_i
```

좌변의 `p(ω_i)`에 Eq. 9.28을 대입하고, `D_ωo`에 Eq. 9.23을 대입하면:

```
      D_ωo(ω_m) · F · G1(ω_i)
f_r = ────────────────────────────
       4 · (ω_o · ω_m) · |cos θ_i|

              ↓  D_ωo 전개

      G1(ω_o) · D(ω_m) · (ω_o · ω_m) · F · G1(ω_i)
f_r = ──────────────────────────────────────────────────
       cos θ_o · 4 · (ω_o · ω_m) · |cos θ_i|

              ↓  (ω_o · ω_m) 소거, G1·G1 → G

      D(ω_m) · F(ω_o · ω_m) · G(ω_i, ω_o)
f_r = ────────────────────────────────────────    ← 최종 (Eq. 9.33)
             4 · cos θ_i · cos θ_o
```

### 5.6 최종 Torrance-Sparrow BRDF — 각 항의 물리적 의미

```
                 D(ω_m) · F(ω_o · ω_m) · G(ω_o, ω_i)
f_r(ω_o, ω_i) = ─────────────────────────────────────────
                         4 · |cos θ_i| · |cos θ_o|
```

**각 항이 왜 있는지:**

| 항 | 역할 | 없으면? |
|---|---|---|
| `D(ω_m)` | 법선이 ω_m인 마이크로면의 밀도 | 어떤 방향의 마이크로면이 기여하는지 알 수 없음 |
| `F(ω_o · ω_m)` | 그 마이크로면에서의 Fresnel 반사율 | 금속/유전체 구분 불가, grazing 반사 효과 없음 |
| `G(ω_o, ω_i)` | 가려지지 않는 비율 | grazing angle에서 에너지 발산 |
| `4 · cosθ_i · cosθ_o` | 투영 면적 보정 (미분 입체각 → 면적) | 단위가 맞지 않음, 에너지 보존 위반 |

**핵심 포인트:**
- `ω_m = normalize(ω_i + ω_o)` — half-vector
- `D` — 아무 마이크로면 분포든 사용 가능 (Beckmann, GGX 등)
- `F` — 아무 Fresnel이든 사용 가능 (유전체, 도체)
- `G` — Smith masking-shadowing
- Fresnel은 **마이크로면 법선 기준 각도** `(ω_o · ω_m)`으로 계산 (매크로 법선이 아님!)

이것이 microfacet 모델의 아름다운 점이다: **D, F, G를 독립적으로 교체할 수 있다.** D를 Beckmann에서 GGX로 바꿔도 나머지 구조는 동일하다. F를 유전체 Fresnel에서 도체 Fresnel로 바꾸면 금속이 된다.

---

## 6. Conductor BRDF (금속 반사)

### 6.1 금속은 유전체와 무엇이 다른가

유전체(유리, 물)는 빛이 내부로 들어가서 통과하거나 산란할 수 있다.
금속은 다르다 — 빛이 내부에 침투하면 **자유 전자에 의해 거의 즉시 흡수**되어 열로 변환된다.

이것이 물리적으로 의미하는 바:
1. **투과(transmission)가 없다** — 금속 BSDF에는 반사만 존재
2. **Fresnel 계산에 복소수가 필요하다** — 흡수를 표현하려면 허수부가 필요

### 6.2 도체의 Fresnel — 왜 복소 굴절률인가

도체(금속)는 **복소 굴절률** `η + ik`를 가진다:
- `η` (real part): 일반적인 굴절률 — 빛의 위상 속도에 영향
- `k` (imaginary part, extinction coefficient): 흡수 계수 — 빛이 금속 내부에서 얼마나 빨리 감쇠하는지

유전체의 Fresnel은 실수만으로 계산할 수 있지만, 금속은 k가 0이 아니므로 복소수 연산이 필요하다. k 값이 클수록 금속적 느낌이 강하고 (구리, 금의 특유한 색감), k=0이면 유전체 Fresnel과 동일하다.

또한 η과 k가 **파장에 따라 다르다**는 것이 금속의 색을 결정한다:
- 금(Au): 적색/녹색 파장을 많이 반사하고 청색을 적게 반사 → 금색
- 구리(Cu): 적색을 많이 반사 → 구리색
- 알루미늄(Al): 모든 파장을 비슷하게 반사 → 은회색

```cpp
// pbrt-v4
Float FrComplex(Float cosTheta_i, pstd::complex<Float> eta) {
    cosTheta_i = Clamp(cosTheta_i, 0, 1);
    Float sin2Theta_i = 1 - Sqr(cosTheta_i);
    Complex sin2Theta_t = sin2Theta_i / Sqr(eta);
    Complex cosTheta_t = sqrt(1 - sin2Theta_t);

    Complex r_parl = (eta * cosTheta_i - cosTheta_t)
                   / (eta * cosTheta_i + cosTheta_t);
    Complex r_perp = (cosTheta_i - eta * cosTheta_t)
                   / (cosTheta_i + eta * cosTheta_t);
    return (norm(r_parl) + norm(r_perp)) / 2;
}
```

### 6.3 Caramel의 현재 도체 Fresnel

`bsdf.cpp:86`에서 Peter Shirley의 방법을 사용하여 per-channel Vector3f로 계산:

```cpp
Vector3f fresnel_conductor(Float cos_i,
    const Vector3f &eta_i,    // 외부 IOR (실수)
    const Vector3f &eta_t,    // 내부 IOR (실수 부분, RGB)
    const Vector3f &eta_t_k)  // 내부 IOR (허수 부분, RGB)
```

### 6.4 Rough Conductor BRDF — 공식

5장의 Torrance-Sparrow 공식에서 F를 도체 Fresnel로 교체하기만 하면 된다.
이것이 microfacet 프레임워크의 모듈성이다:

```
                 D(ω_m) · F_conductor(ω_o · ω_m, η, k) · G(ω_o, ω_i)
f_r(ω_o, ω_i) = ──────────────────────────────────────────────────────────
                                4 · |cos θ_i| · |cos θ_o|
```

반사만 있고 투과는 없다 (빛이 금속에 들어가면 즉시 흡수). 이것이 rough conductor가 rough dielectric보다 훨씬 구현이 단순한 이유이다 — 투과 lobe가 없으므로 일반화된 half-vector도, 투과 Jacobian도, η² 보정도 필요 없다.

#### 샘플링

```
1. ω_m = Sample_wm(ω_o, u)           // 가시 노말 샘플
2. ω_i = reflect(ω_o, ω_m)           // 반사
3. if (!SameHemisphere(ω_o, ω_i)) → 무효
4. pdf = PDF(ω_o, ω_m) / (4·|ω_o · ω_m|)
5. F = F_conductor(|ω_o · ω_m|, η, k)
6. f = D(ω_m) · F · G(ω_o, ω_i) / (4·cosθ_i·cosθ_o)
7. return {ω_i, f, pdf}
```

#### 평가

```
1. ω_m = normalize(ω_i + ω_o)
2. F = F_conductor(|ω_o · ω_m|, η, k)
3. return D(ω_m) · F · G(ω_o, ω_i) / (4·|cosθ_i|·|cosθ_o|)
```

#### PDF

```
1. ω_m = FaceForward(normalize(ω_i + ω_o), n)
2. return PDF(ω_o, ω_m) / (4·|ω_o · ω_m|)
```

### 6.5 alpha → 0일 때 (EffectivelySmooth) — 왜 분기가 필요한가

α가 매우 작으면 GGX 분포가 거의 delta 함수에 가까워진다. 수치적으로:
- `D(ω_m)`이 특정 방향에서 극단적으로 큰 값을 가짐 → 오버플로우 위험
- `1/cos⁴θ` 항이 분모에서 문제를 일으킴

따라서 `α < 1e-3`이면 microfacet 계산을 아예 건너뛰고 완벽 거울로 취급한다. 이것은 물리적으로도 맞다 — α→0이면 정확히 완벽 거울이므로:

```cpp
// pbrt-v4 ConductorBxDF::Sample_f (smooth case)
Vector3f wi(-wo.x, -wo.y, wo.z);  // 정반사
SampledSpectrum f = FrComplex(AbsCosTheta(wi), eta, k) / AbsCosTheta(wi);
return BSDFSample(f, wi, 1, BxDFFlags::SpecularReflection);
```

이 경우 `f()` 평가는 항상 0을 반환한다 (Dirac delta이므로).

---

## 7. Rough Dielectric BSDF (유전체 반사+투과)

### 7.1 왜 Conductor와 별도로 필요한가

Rough conductor(6장)는 반사만 다루면 됐다. 하지만 유전체(유리, 물, 플라스틱 표면)는 빛이 내부로 투과할 수 있다. 이것은 두 가지 근본적인 복잡성을 추가한다:

1. **두 개의 lobe**: 하나의 마이크로면에서 빛이 반사될 수도, 투과될 수도 있다. 어느 쪽인지는 Fresnel 확률로 결정된다.
2. **Snell의 법칙**: 투과 시 빛의 방향이 굴절률 비에 의해 꺾인다. 이것은 반사(입사각=반사각)보다 수학이 복잡하다.

Rough dielectric은 microfacet 모델에서 **가장 복잡한** BSDF이다.

```
        ω_o (출사)
         ↑
    ┌────┤────┐
    │    │    │    ← 매크로 표면
    │ /\ │ /\ │
    │/ωm\│/  \│    ← 마이크로면 (ω_m 방향)
    │    │    │
    ↙         ↘
  ω_i(반사)  ω_i(투과)

  Fresnel R 확률로 반사, (1-R) 확률로 투과
```

### 7.2 일반화된 Half-vector — 왜 반사와 다른가 (Eq. 9.34)

반사에서는 `ω_m = normalize(ω_i + ω_o)`였다 — ω_i와 ω_o가 ω_m 기준으로 대칭이기 때문이다.

투과에서는 대칭이 깨진다. Snell의 법칙 `η_i sin θ_i = η_o sin θ_o`에 의해 입사각과 투과각이 다르다. 마이크로면 법선 ω_m에 대해 입사/투과 방향의 **접선 성분이 상쇄되려면** η로 스케일링이 필요하다:

```
ω_m = normalize(η · ω_i + ω_o)    (Eq. 9.34)
```

여기서 `η = η_i / η_o` (상대 굴절률).

직관적으로: 반사에서는 ω_i와 ω_o를 "같은 무게로" 더하면 중간 방향이 나온다. 투과에서는 굴절 때문에 한쪽이 더 많이 꺾이므로 η로 보정해야 정확한 중간 방향(= 마이크로면 법선)이 나온다.

반사의 경우 `η = 1`이므로 `ω_m = normalize(ω_i + ω_o)`로 환원된다.

**구현 시 주의:** `FaceForward`를 적용하여 `ω_m`이 항상 매크로 법선과 같은 쪽을 향하도록 한다.

### 7.3 투과의 Jacobian — 왜 반사와 다른가 (Eq. 9.36)

반사 Jacobian은 깔끔한 `1 / (4·(ω_o · ω_m))`이었다. 반사가 대칭적이기 때문이다.

투과에서는 **굴절에 의해 입체각 자체가 변한다**. 밀도가 높은 매질에서 낮은 매질로 나갈 때 빛 줄기가 넓어지고(입체각 확장), 반대 방향이면 좁아진다(입체각 압축). 이 비대칭적 변환을 Jacobian이 포착한다:

```
dω_m     |ω_i · ω_m|
──── = ──────────────────────────
dω_i   (ω_i · ω_m + ω_o · ω_m / η)²
```

또는 동등하게:

```
dω_m     η² · |ω_i · ω_m|
──── = ──────────────────────────────────
dω_i   (η · (ω_i · ω_m) + (ω_o · ω_m))²
```

반사와 비교하면:
- 반사: `dω_m/dω_i = 1/(4·(ω_o·ω_m))` — 깔끔한 상수 비율
- 투과: 분모가 `(...)²` 형태 — η에 의존하는 복잡한 비율

이 Jacobian을 빠뜨리면 PDF가 틀어지고, 결과 이미지에 밝기 편향(bias)이 생긴다.

### 7.4 BTDF (투과 BSDF) (Eq. 9.40)

```
                D(ω_m) · (1 − F) · G(ω_i, ω_o)     |ω_i · ω_m| · |ω_o · ω_m|
f_t(ω_o,ω_i) = ──────────────────────────────── · ─────────────────────────────
                (ω_i·ω_m + ω_o·ω_m/η)²              |cos θ_i| · |cos θ_o|
```

유도 과정 (Eq. 9.38 → 9.40):

```
Monte Carlo 추정기:  f_t · L_i · |cos θ_i| / p(ω_i) = (1−F) · G1(ω_i) · L_i

p(ω_i) = D_ωo(ω_m) · |ω_i·ω_m| / (ω_i·ω_m + ω_o·ω_m/η)²   (Eq. 9.37)

                  D_ωo(ω_m) · |ω_i·ω_m| · (1−F) · G1(ω_i)
→ f_t = ─────────────────────────────────────────────────────────
         (ω_i·ω_m + ω_o·ω_m/η)² · |cos θ_i|

→ D_ωo 전개 (Eq. 9.23), G1·G1 → G:

                  D(ω_m) · (1−F) · G(ω_i,ω_o) · |ω_i·ω_m| · |ω_o·ω_m|
→ f_t = ────────────────────────────────────────────────────────────────────   (Eq. 9.40)
         (ω_i·ω_m + ω_o·ω_m/η)² · |cos θ_i| · |cos θ_o|
```

### 7.5 비대칭 산란 보정 (Non-Symmetric Scattering) — 왜 η² 나눗셈이 필요한가

BSDF는 본래 `f(ω_o, ω_i) = f(ω_i, ω_o)` (상호성, reciprocity)를 만족해야 한다. 하지만 **투과는 상호적이지 않다**.

이유: radiance(방사휘도)는 입체각당 에너지를 나타내는데, 빛이 굴절률이 다른 매질을 통과하면 **입체각 자체가 η²만큼 변한다**. 같은 에너지가 더 넓은(또는 좁은) 입체각에 분배되므로 radiance가 변한다.

구체적으로:
- 공기(η=1) → 유리(η=1.5)로 들어갈 때: 입체각이 η²=2.25배 좁아짐 → radiance 증가
- 유리 → 공기로 나갈 때: 입체각이 2.25배 넓어짐 → radiance 감소

이 효과를 보정하지 않으면 양방향 path tracing에서 에너지가 맞지 않는다:

```
f_t_corrected = f_t / η²
```

```cpp
// pbrt-v4
if (mode == TransportMode::Radiance)
    ft /= Sqr(etap);
```

참고: caramel의 기존 `Dielectric` 구현(`dielectric.cpp:60`)에도 이 보정이 TODO로 남아있다.

### 7.6 반사 + 투과 통합 샘플링 — 왜 Fresnel로 lobe를 선택하는가

마이크로면 하나를 샘플했을 때, 그 면에서 빛이 반사될지 투과될지는 Fresnel 확률 R에 의해 결정된다. 따라서 sampling에서도 같은 확률로 lobe를 선택하는 것이 최적이다 — 이것이 **Fresnel-weighted lobe selection**이다.

만약 50:50으로 선택하면? 대부분의 빛이 반사되는 상황에서도 투과를 절반 확률로 샘플하게 되어, 투과 샘플의 기여가 작아 분산이 높아진다.

```
1. ω_m = Sample_wm(ω_o, sampler)            // 가시 노말 샘플
2. R = FrDielectric(ω_o · ω_m, η)           // Fresnel 반사율
3. T = 1 − R

4. if (rand < R/(R+T)):  ── 반사 ──
     ω_i = reflect(ω_o, ω_m)
     if (!SameHemisphere(ω_o, ω_i)) → 무효

     pdf = D_ωo(ω_m) / (4·|ω_o·ω_m|) · R/(R+T)
     f   = D·R·G / (4·|cosθ_i|·|cosθ_o|)

5. else:                 ── 투과 ──
     ω_i = refract(ω_o, ω_m, η)
     if (SameHemisphere(ω_o, ω_i) || TIR) → 무효

     denom = (ω_i·ω_m + ω_o·ω_m/η)²
     pdf   = D_ωo(ω_m) · |ω_i·ω_m| / denom · T/(R+T)
     f     = D·T·G · |ω_i·ω_m|·|ω_o·ω_m| / (|cosθ_i|·|cosθ_o|·denom)
     if (radiance mode): f /= η²
```

### 7.7 PDF 계산 (평가 시)

주어진 (ω_o, ω_i) 쌍에 대해 PDF를 계산할 때:

```
1. 반사인지 투과인지 판별:
   reflect = (cosθ_i · cosθ_o > 0)  // 같은 반구면 반사

2. Half-vector 계산:
   if reflect: ω_m = normalize(ω_i + ω_o)
   else:       ω_m = normalize(η·ω_i + ω_o),   η = cosθ_o > 0 ? η_rel : 1/η_rel

3. FaceForward(ω_m, n)

4. Fresnel 가중치:
   R = FrDielectric(ω_o · ω_m, η)
   T = 1 − R

5. PDF:
   if reflect: pdf = D_ωo(ω_m) / (4·|ω_o·ω_m|) · R/(R+T)
   else:       pdf = D_ωo(ω_m) · |ω_i·ω_m| / denom · T/(R+T)
```

### 7.8 에지 케이스 — 왜 이런 검사가 필요한가

| 케이스 | 처리 | 왜 필요한가 |
|--------|------|-------------|
| `cosθ_i = 0` 또는 `cosθ_o = 0` | 0 반환 | BRDF 분모 `4·cosθ_i·cosθ_o`가 0이 되어 발산 |
| `‖ω_m‖ = 0` | 0 반환 | ω_i와 ω_o가 정반대이면 half-vector가 정의 불가 |
| 전반사 (TIR) | 투과 불가능 → 반사만 | sin θ_t > 1이면 물리적으로 투과 불가 (임계각 초과) |
| `ω_i.z = 0` | 무효 | 접선 방향은 표면에 평행 — 물리적으로 무의미 |
| backfacing microfacet | `(ω_m·ω_i)·cosθ_i < 0`이면 제외 | 마이크로면 법선과 매크로 법선이 다른 쪽을 향하면 물리적으로 불가능한 배치 |

---

## 8. 코드 비교: pbrt-v4 vs mitsuba3 vs caramel

### 8.1 아키텍처 매핑

```
pbrt-v4                       mitsuba3                      caramel (현재)
───────────────────────────────────────────────────────────────────────────
TrowbridgeReitzDistribution   MicrofacetDistribution        (없음 — inline 함수만)
  alpha_x, alpha_y              m_alpha_u, m_alpha_v         m_alpha (float)
  D(wm)                         eval(wm)                     sample_beckmann_distrib_pdf()
  G(wo, wi)                     G(wi, wo, wm)                G1() * G1()
  Lambda(w)                     smith_g1(v, m)               (rational approx, G1 내부)
  Sample_wm(w, u)               sample(wi, u)                sample_beckmann_distrib()
  PDF(w, wm)                    pdf(wi, wm)                  (inline)
  EffectivelySmooth()           (없음 — 외부에서 처리)         (없음)

ConductorBxDF                 RoughConductor                ❌ 없음
  eta (SampledSpectrum)          m_eta (Spectrum)             (Conductor: delta only)
  k (SampledSpectrum)            m_k (Spectrum)

DielectricBxDF                RoughDielectric               ❌ 없음
  eta (Float)                    m_eta (Float)                (Dielectric: delta only)
  mfDistrib                      m_distr
```

### 8.2 분포 비교

| | Beckmann | GGX (Trowbridge-Reitz) |
|---|---|---|
| **caramel** | `warp_sample.h` | **미구현** |
| **pbrt-v4** | 미구현 (GGX만) | `scattering.h` |
| **mitsuba3** | `microfacet.h` | `microfacet.h` |

### 8.3 샘플링 전략 비교

| | caramel | pbrt-v4 | mitsuba3 |
|---|---|---|---|
| **노말 샘플링** | 비가시 (Non-visible) | **가시 (Visible)** | 둘 다 (기본: 가시) |
| **D 분포** | Beckmann | GGX | Beckmann/GGX |
| **G 모델** | Uncorrelated `G1·G1` | Height-correlated | Height-correlated |

### 8.4 BSDF 인터페이스 비교

```
pbrt-v4 BxDF                  mitsuba3 BSDF                caramel BSDF
─────────────────────────────────────────────────────────────────────────
f(wo, wi, mode)               eval(ctx, si, wo)            get_reflection(in, out, uv)
  → SampledSpectrum             → Spectrum                   → Vector3f

Sample_f(wo, uc, u, mode)     sample(ctx, si, sample1)     sample_recursive_dir(in, uv, sampler)
  → BSDFSample{f,wi,pdf}       → (wo, weight, pdf)          → tuple{dir, weight, pdf}

PDF(wo, wi, mode)             pdf(ctx, si, wo)             pdf(in, out)
  → Float                      → Float                      → Float

Flags()                       (flags in ctor)              is_discrete(frontside)
```

**핵심 차이:**

1. **caramel의 `sample_recursive_dir()`은 `f·cosθ/pdf`를 반환**한다 (weight).
   pbrt는 `f`와 `pdf`를 별도로 반환하고 호출자가 `f·cosθ/pdf`를 계산한다.

2. **caramel은 `local_incoming_dir`가 표면을 향한다** (z < 0이 앞면에서 들어옴).
   pbrt/mitsuba는 `wo`가 표면에서 나가는 방향 (z > 0이 앞면에서 나감).
   → caramel에서는 `local_incoming_flipped = -local_incoming_dir`로 뒤집어 사용.

### 8.5 방향 Convention 정리

```
caramel:                          pbrt-v4 / mitsuba3:

     ω_o (out)                         ω_o (out)
      ↑                                ↑
      │                                │
 ─────┼──── surface                ────┼──── surface
      │                                │
      ↓                                ↓
 incoming_dir                      ω_i (in, evaluated)
 (점 → 카메라 반대 방향)

 local_incoming_flipped
   = -incoming_dir                  ω_o in pbrt =
   = ω_o (pbrt의 wo와 동일)         local_incoming_flipped in caramel
```

---

## 9. Caramel 구현 가이드

### 9.1 구현 단계

```
Phase 1: MicrofacetDistribution 클래스 (Beckmann + GGX 통합)
   └─ D, Lambda, G1, G, Sample_wm, PDF
   └─ chi2 테스트로 샘플링 검증 (Beckmann / GGX 각각)

Phase 2: RoughConductor (반사만 — 비교적 단순)
   └─ alpha≈0 → 기존 Conductor와 동일 결과 검증
   └─ alpha=0.1~0.5 → glossy metal
   └─ distribution 파라미터로 Beckmann/GGX 선택

Phase 3: RoughDielectric (반사+투과 — 가장 복잡)
   └─ alpha≈0 → 기존 Dielectric와 동일 결과 검증
   └─ alpha=0.1~0.3 → frosted glass
```

### 9.2 Phase 1: MicrofacetDistribution (Beckmann + GGX 추상화)

#### 설계 원칙

2.6절에서 정리했듯이, Beckmann과 GGX는 **동일한 역할의 서로 다른 구현체**이다.
분기가 필요한 곳(D, Λ, Sample_wm)과 공통인 곳(G1, G, PDF)을 하나의 클래스로 통합한다.

```
MicrofacetDistribution
  ├── D(wm)          분기: Beckmann / GGX
  ├── Lambda(w)      분기: rational approx / closed-form
  ├── Sample_wm(w)   분기: erfinv 수치역산 / disk warp
  ├── G1(w)          공통: 1 / (1 + Λ)
  ├── G(wo, wi)      공통: 1 / (1 + Λ_o + Λ_i)
  └── PDF(w, wm)     공통: G1/cosθ · D · |w·wm|
```

mitsuba3과 동일한 패턴이다 (`microfacet.h`의 `MicrofacetDistribution`).
pbrt-v4는 GGX만 구현(`TrowbridgeReitzDistribution`)하고 있다.

#### 파일 위치

`include/microfacet_distrib.h` — 새 파일로 생성한다. 기존 `warp_sample.h`의 `sample_beckmann_distrib`은
하위 호환을 위해 유지하되, 새 코드에서는 `MicrofacetDistribution`을 사용한다.

#### 코드

```cpp
#pragma once

#include <cmath>
#include <common.h>
#include <sampler.h>

namespace Caramel {

enum class MicrofacetType { Beckmann, GGX };

class MicrofacetDistribution {
public:
    MicrofacetDistribution(MicrofacetType type, Float alpha)
        : m_type{type}, m_alpha{std::max(alpha, static_cast<Float>(1e-4))} {}

    // ──────────────────────────────────────────────────
    //  분기 필요: D, Lambda, Sample_wm
    // ──────────────────────────────────────────────────

    /// 노말 분포 함수 D(ω_m)
    /// Beckmann: exp(-tan²θ/α²) / (π·α²·cos⁴θ)
    /// GGX:      1 / (π·α²·cos⁴θ·(1+tan²θ/α²)²)
    Float D(const Vector3f &wm) const {
        Float cos_theta = wm[2];
        if (cos_theta <= Float0) return Float0;
        Float cos2 = cos_theta * cos_theta;
        Float cos4 = cos2 * cos2;
        if (cos4 < static_cast<Float>(1e-16)) return Float0;
        Float tan2 = (wm[0]*wm[0] + wm[1]*wm[1]) / cos2;
        if (std::isinf(tan2)) return Float0;
        Float a2 = m_alpha * m_alpha;

        if (m_type == MicrofacetType::Beckmann) {
            return std::exp(-tan2 / a2) / (PI * a2 * cos4);
        } else { // GGX
            Float tmp = Float1 + tan2 / a2;
            return Float1 / (PI * a2 * cos4 * tmp * tmp);
        }
    }

    /// Smith 보조 함수 Λ(ω)
    /// Beckmann: rational approximation (erf 대용, <0.35% 오차)
    /// GGX:      (√(1 + α²·tan²θ) − 1) / 2 (해석적)
    Float Lambda(const Vector3f &w) const {
        Float cos2 = w[2] * w[2];
        Float sin2 = w[0]*w[0] + w[1]*w[1];
        Float tan2 = sin2 / cos2;
        if (std::isinf(tan2)) return Float0;

        if (m_type == MicrofacetType::Beckmann) {
            Float a = Float1 / (m_alpha * std::sqrt(tan2)); // 1/(α·tanθ)
            if (a >= static_cast<Float>(1.6)) return Float0;
            Float a2 = a * a;
            return (static_cast<Float>(3.535) * a + static_cast<Float>(2.181) * a2)
                 / (Float1 + static_cast<Float>(2.276) * a + static_cast<Float>(2.577) * a2)
                 ;
            // 주의: 이 rational approx는 G1 = approx(a)의 형태이다.
            // Λ = 1/G1 - 1로 변환하거나, G1에서 직접 사용하는 방식을 택할 수 있다.
            // 아래 G1()에서 이를 처리한다.
        } else { // GGX
            Float a2 = m_alpha * m_alpha;
            return (std::sqrt(Float1 + a2 * tan2) - Float1) * Float0_5;
        }
    }

    /// 가시 노말 샘플링
    /// Beckmann: 비가시 노말 샘플링 (기존 caramel 방식, 추후 가시로 업그레이드 가능)
    /// GGX:      가시 노말 샘플링 (pbrt-v4 / Heitz 2018 알고리즘)
    Vector3f Sample_wm(const Vector3f &w, Sampler &sampler) const {
        if (m_type == MicrofacetType::Beckmann) {
            return sample_beckmann(sampler);
        } else { // GGX
            return sample_ggx_visible(w, sampler);
        }
    }

    // ──────────────────────────────────────────────────
    //  공통: G1, G, PDF — 분포에 무관
    // ──────────────────────────────────────────────────

    /// Smith G1(ω) = 1 / (1 + Λ(ω))
    Float G1(const Vector3f &w) const {
        if (m_type == MicrofacetType::Beckmann) {
            // Beckmann은 Λ 대신 rational approx로 G1을 직접 계산하는 것이 효율적
            return G1_beckmann(w);
        }
        return Float1 / (Float1 + Lambda(w));
    }

    /// Height-correlated masking-shadowing G(ω_o, ω_i) = 1 / (1 + Λ(ω_o) + Λ(ω_i))
    Float G(const Vector3f &wo, const Vector3f &wi) const {
        if (m_type == MicrofacetType::Beckmann) {
            // Beckmann: rational approx로 G1을 직접 구한 뒤 height-correlated로 조합
            // G = 1/(1 + Λ_o + Λ_i), Λ = 1/G1 - 1 이므로:
            // G = 1/(1 + (1/G1_o - 1) + (1/G1_i - 1)) = 1/(1/G1_o + 1/G1_i - 1)
            Float g1_o = G1_beckmann(wo);
            Float g1_i = G1_beckmann(wi);
            Float denom = Float1 / g1_o + Float1 / g1_i - Float1;
            return (denom > Float0) ? Float1 / denom : Float0;
        }
        return Float1 / (Float1 + Lambda(wo) + Lambda(wi));
    }

    /// 가시 노말 분포 D_ω(ω_m) — 이것이 곧 PDF
    Float PDF(const Vector3f &w, const Vector3f &wm) const {
        if (m_type == MicrofacetType::Beckmann) {
            // 비가시 노말 샘플링의 PDF: D(wm) · cosθ_m
            return D(wm) * std::max(Float0, wm[2]);
        }
        // GGX 가시 노말 샘플링의 PDF: D_ω(ω_m)
        return G1(w) / std::abs(w[2]) * D(wm) * std::abs(w.dot(wm));
    }

    bool EffectivelySmooth() const {
        return m_alpha < static_cast<Float>(1e-3);
    }

    MicrofacetType type() const { return m_type; }
    Float alpha() const { return m_alpha; }

private:
    MicrofacetType m_type;
    Float m_alpha;

    // ── Beckmann 전용 ──

    /// Beckmann G1 — rational approximation (mitsuba3/caramel 동일)
    Float G1_beckmann(const Vector3f &w) const {
        Float cos2 = w[2] * w[2];
        Float sin2 = w[0]*w[0] + w[1]*w[1];
        if (sin2 == Float0) return Float1; // 수직 입사 → masking 없음
        Float a = w[2] / (std::sqrt(sin2) * m_alpha); // 1/(α·tanθ) 와 동치
        if (a >= static_cast<Float>(1.6)) return Float1;
        Float a2 = a * a;
        return (static_cast<Float>(3.535) * a + static_cast<Float>(2.181) * a2)
             / (Float1 + static_cast<Float>(2.276) * a + static_cast<Float>(2.577) * a2);
    }

    /// Beckmann 비가시 노말 샘플링 (기존 caramel sample_beckmann_distrib과 동일)
    Vector3f sample_beckmann(Sampler &sampler) const {
        using std::atan; using std::sqrt; using std::log;
        using std::cos; using std::sin;
        Float s1 = sampler.sample_1d();
        Float s2 = sampler.sample_1d();
        Float phi = PI_2 * s1;
        Float theta = atan(sqrt(-m_alpha*m_alpha * log(Float1 - s2)));
        return Vector3f{sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)};
    }

    // ── GGX 전용 ──

    /// GGX 가시 노말 샘플링 (pbrt-v4 / Heitz 2018)
    Vector3f sample_ggx_visible(const Vector3f &w, Sampler &sampler) const {
        using std::sqrt; using std::cos; using std::sin;

        // Step 1: Stretch — 타원체를 반구로 변환
        Vector3f wh = Vector3f{m_alpha * w[0], m_alpha * w[1], w[2]}.normalize();
        if (wh[2] < Float0) wh = -wh;

        // Step 2: wh 기준 직교 기저 구성
        Vector3f T1 = (wh[2] < static_cast<Float>(0.99999))
            ? Vector3f{Float0, Float0, Float1}.cross(wh).normalize()
            : Vector3f{Float1, Float0, Float0};
        Vector3f T2 = wh.cross(T1);

        // Step 3: 균일 디스크 샘플링 (극좌표)
        Float r = sqrt(sampler.sample_1d());
        Float phi = PI_2 * sampler.sample_1d();
        Float t1 = r * cos(phi);
        Float t2_raw = r * sin(phi);

        // Step 4: 가시성 보정 (warping)
        Float h = sqrt(std::max(Float0, Float1 - t1 * t1));
        Float s = (Float1 + wh[2]) * Float0_5;
        Float t2 = (Float1 - s) * h + s * t2_raw;

        // Step 5: 반구 재투영 & 역변환 (unstretching)
        Float pz = sqrt(std::max(Float0, Float1 - t1*t1 - t2*t2));
        Vector3f nh = t1 * T1 + t2 * T2 + pz * wh;
        return Vector3f{
            m_alpha * nh[0],
            m_alpha * nh[1],
            std::max(static_cast<Float>(1e-6), nh[2])
        }.normalize();
    }
};

} // namespace Caramel
```

#### 주요 설계 포인트

**1. Beckmann은 비가시, GGX는 가시 노말 샘플링**

Beckmann의 가시 노말 샘플링은 erfinv 수치역산이 필요하여 구현이 복잡하다(mitsuba3 `microfacet.h:368` 참조). 1단계에서는 기존 caramel의 Beckmann 비가시 샘플링을 그대로 사용하고, GGX만 가시 노말 샘플링으로 구현한다.

이로 인해 `PDF()`가 분포에 따라 다르다:
- Beckmann (비가시): `PDF = D(wm) · cosθ_m`
- GGX (가시): `PDF = G1(w)/|cosθ| · D(wm) · |w · wm|`

추후 Beckmann도 가시 노말 샘플링으로 업그레이드하면 PDF를 통일할 수 있다.

**2. Beckmann의 G1 — rational approximation 직접 사용**

Beckmann의 Λ는 erf 기반이라 해석적 형태가 복잡하다.
`G1 = 1/(1+Λ)`를 직접 구하는 대신, mitsuba3/caramel이 모두 사용하는
rational approximation으로 G1을 바로 계산한다.

Height-correlated G는 `Λ = 1/G1 - 1`을 대입하여 구한다:

```
G = 1/(1 + Λ_o + Λ_i) = 1/(1/G1_o + 1/G1_i - 1)
```

**3. 기존 코드와의 호환**

기존 `Microfacet` 클래스(diffuse+specular 혼합 모델)와 `warp_sample.h`의 함수들은
그대로 유지한다. 새 `RoughConductor`/`RoughDielectric`만 `MicrofacetDistribution`을 사용한다.

### 9.3 Phase 2: RoughConductor

**`include/bsdf.h`에 클래스 선언 추가:**

```cpp
class RoughConductor final : public BSDF {
public:
    RoughConductor(MicrofacetType distrib_type, Float alpha,
                   const Conductors &mat, Float ex_ior);
    std::tuple<Vector3f, Vector3f, Float>
        sample_recursive_dir(const Vector3f &local_incoming_dir,
                             const Vector2f &uv, Sampler &sampler) const override;
    Float pdf(const Vector3f &local_incoming_dir,
              const Vector3f &local_outgoing_dir) const override;
    Vector3f get_reflection(const Vector3f &local_incoming_dir,
                            const Vector3f &local_outgoing_dir,
                            const Vector2f &uv) const override;
    bool is_discrete(bool) const override;

private:
    MicrofacetDistribution m_distrib;  // Beckmann 또는 GGX — 이 클래스는 신경 쓰지 않음
    Vector3f m_eta;   // 복소 IOR 실수부 (RGB)
    Vector3f m_k;     // 복소 IOR 허수부 (RGB)
    Float m_ex_ior;
};
```

**`src/bsdfs/roughconductor.cpp` 핵심 로직:**

```cpp
// ── get_reflection (= eval, f 계산) ──
Vector3f RoughConductor::get_reflection(
    const Vector3f &local_incoming_dir,
    const Vector3f &local_outgoing_dir, const Vector2f &) const
{
    if (m_distrib.EffectivelySmooth()) return vec3f_zero;  // delta

    const Vector3f wo = (-local_incoming_dir).normalize();  // caramel convention
    const Vector3f wi = local_outgoing_dir.normalize();

    if (wo[2] <= Float0 || wi[2] <= Float0) return vec3f_zero;

    Vector3f wm = (wo + wi).normalize();
    if (wm.length_sq() == Float0) return vec3f_zero;

    Float cos_o = wo[2], cos_i = wi[2];

    // f = D · F · G / (4 · cosθ_i · cosθ_o)
    Float D = m_distrib.D(wm);
    Float G = m_distrib.G(wo, wi);
    Vector3f F = fresnel_conductor(std::abs(wo.dot(wm)),
                    {m_ex_ior, m_ex_ior, m_ex_ior}, m_eta, m_k);

    Float denom = static_cast<Float>(4) * cos_i * cos_o;
    return F * (D * G / denom);
}

// ── pdf ──
Float RoughConductor::pdf(
    const Vector3f &local_incoming_dir,
    const Vector3f &local_outgoing_dir) const
{
    if (m_distrib.EffectivelySmooth()) return Float0;

    const Vector3f wo = (-local_incoming_dir).normalize();
    const Vector3f wi = local_outgoing_dir.normalize();

    if (wo[2] <= Float0 || wi[2] <= Float0) return Float0;

    Vector3f wm = (wo + wi).normalize();

    // pdf = D_vis(wo, wm) / (4 · |wo · wm|)
    return m_distrib.PDF(wo, wm) / (static_cast<Float>(4) * std::abs(wo.dot(wm)));
}

// ── sample_recursive_dir ──
std::tuple<Vector3f, Vector3f, Float> RoughConductor::sample_recursive_dir(
    const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const
{
    const Vector3f wo = (-local_incoming_dir).normalize();
    if (wo[2] <= Float0) return {vec3f_zero, vec3f_zero, Float0};

    // Smooth → perfect mirror
    if (m_distrib.EffectivelySmooth()) {
        Vector3f wi = {-wo[0], -wo[1], wo[2]};  // 정반사 (z 뒤집기가 아님, reflect)
        // 실제로는 reflect(wo, (0,0,1)) = (-wo.x, -wo.y, wo.z)
        Vector3f F = fresnel_conductor(wo[2],
                        {m_ex_ior, m_ex_ior, m_ex_ior}, m_eta, m_k);
        return {wi, F, Float0};  // pdf=0 → delta
    }

    // 가시 노말 샘플
    Vector3f wm = m_distrib.Sample_wm(wo, sampler);
    Vector3f wi = reflect(-wo, wm);  // reflect 사용

    if (wi[2] <= Float0) return {vec3f_zero, vec3f_zero, Float0};

    Float pdf_ = m_distrib.PDF(wo, wm) /
                 (static_cast<Float>(4) * std::abs(wo.dot(wm)));

    Vector3f F = fresnel_conductor(std::abs(wo.dot(wm)),
                    {m_ex_ior, m_ex_ior, m_ex_ior}, m_eta, m_k);
    Float D = m_distrib.D(wm);
    Float G = m_distrib.G(wo, wi);
    Float denom = static_cast<Float>(4) * wi[2] * wo[2];

    Vector3f f = F * (D * G / denom);
    return {wi, f * wi[2] / pdf_, pdf_};
}

bool RoughConductor::is_discrete(bool) const {
    return m_distrib.EffectivelySmooth();
}
```

### 9.4 Phase 3: RoughDielectric

**`include/bsdf.h`에 클래스 선언 추가:**

```cpp
class RoughDielectric final : public BSDF {
public:
    RoughDielectric(MicrofacetType distrib_type, Float alpha,
                    Float in_ior, Float ex_ior);
    std::tuple<Vector3f, Vector3f, Float>
        sample_recursive_dir(const Vector3f &local_incoming_dir,
                             const Vector2f &uv, Sampler &sampler) const override;
    Float pdf(const Vector3f &local_incoming_dir,
              const Vector3f &local_outgoing_dir) const override;
    Vector3f get_reflection(const Vector3f &local_incoming_dir,
                            const Vector3f &local_outgoing_dir,
                            const Vector2f &uv) const override;
    bool is_discrete(bool frontside) const override;

private:
    MicrofacetDistribution m_distrib;  // Beckmann 또는 GGX
    Float m_in_ior;
    Float m_ex_ior;
    Float m_eta;  // m_in_ior / m_ex_ior
};
```

**`src/bsdfs/roughdielectric.cpp` 핵심 로직:**

```cpp
// ── get_reflection (= eval) ──
Vector3f RoughDielectric::get_reflection(
    const Vector3f &local_incoming_dir,
    const Vector3f &local_outgoing_dir, const Vector2f &) const
{
    if (m_distrib.EffectivelySmooth()) return vec3f_zero;

    const Vector3f wo = (-local_incoming_dir).normalize();
    const Vector3f wi = local_outgoing_dir.normalize();
    Float cos_o = wo[2], cos_i = wi[2];

    bool is_reflect = (cos_i * cos_o > Float0);
    Float etap = Float1;
    if (!is_reflect)
        etap = cos_o > Float0 ? m_eta : (Float1 / m_eta);

    // Half-vector
    Vector3f wm = (is_reflect) ? (wi + wo) : (wi * etap + wo);
    if (wm.length_sq() == Float0) return vec3f_zero;
    wm = wm.normalize();
    if (wm[2] < Float0) wm = -wm;  // FaceForward

    // Backfacing microfacet check
    if (wm.dot(wi) * cos_i < Float0 || wm.dot(wo) * cos_o < Float0)
        return vec3f_zero;

    Float F = fresnel_dielectric(std::abs(wo.dot(wm)), m_ex_ior, m_in_ior);
    Float D = m_distrib.D(wm);
    Float G = m_distrib.G(wo, wi);

    if (is_reflect) {
        // BRDF: D·F·G / (4·|cosθ_i|·|cosθ_o|)
        Float val = D * F * G /
            (static_cast<Float>(4) * std::abs(cos_i) * std::abs(cos_o));
        return {val, val, val};
    } else {
        // BTDF: D·(1-F)·G·|ω_i·ω_m|·|ω_o·ω_m| / (|cosθ_i|·|cosθ_o|·denom²)
        Float denom = wi.dot(wm) + wo.dot(wm) / etap;
        Float ft = D * (Float1 - F) * G *
                   std::abs(wi.dot(wm) * wo.dot(wm)) /
                   (std::abs(cos_i) * std::abs(cos_o) * denom * denom);
        // Non-symmetric correction (radiance mode)
        ft /= (etap * etap);
        return {ft, ft, ft};
    }
}

// ── pdf ──
Float RoughDielectric::pdf(
    const Vector3f &local_incoming_dir,
    const Vector3f &local_outgoing_dir) const
{
    if (m_distrib.EffectivelySmooth()) return Float0;

    const Vector3f wo = (-local_incoming_dir).normalize();
    const Vector3f wi = local_outgoing_dir.normalize();
    Float cos_o = wo[2], cos_i = wi[2];

    bool is_reflect = (cos_i * cos_o > Float0);
    Float etap = Float1;
    if (!is_reflect)
        etap = cos_o > Float0 ? m_eta : (Float1 / m_eta);

    Vector3f wm = is_reflect ? (wi + wo) : (wi * etap + wo);
    if (wm.length_sq() == Float0) return Float0;
    wm = wm.normalize();
    if (wm[2] < Float0) wm = -wm;

    if (wm.dot(wi) * cos_i < Float0 || wm.dot(wo) * cos_o < Float0)
        return Float0;

    Float R = fresnel_dielectric(std::abs(wo.dot(wm)), m_ex_ior, m_in_ior);
    Float T = Float1 - R;

    Float pdf;
    if (is_reflect) {
        pdf = m_distrib.PDF(wo, wm) /
              (static_cast<Float>(4) * std::abs(wo.dot(wm)))
              * R / (R + T);
    } else {
        Float denom = wi.dot(wm) + wo.dot(wm) / etap;
        Float dwm_dwi = std::abs(wi.dot(wm)) / (denom * denom);
        pdf = m_distrib.PDF(wo, wm) * dwm_dwi * T / (R + T);
    }
    return pdf;
}

// ── sample_recursive_dir ──
std::tuple<Vector3f, Vector3f, Float> RoughDielectric::sample_recursive_dir(
    const Vector3f &local_incoming_dir, const Vector2f &, Sampler &sampler) const
{
    const Vector3f wo = (-local_incoming_dir).normalize();

    // Smooth → 기존 Dielectric 로직과 동일
    if (m_distrib.EffectivelySmooth()) {
        /* smooth 케이스: 기존 Dielectric::sample_recursive_dir() 로직 재사용 */
        // ... (생략 — 기존 Dielectric 코드와 동일)
    }

    Vector3f wm = m_distrib.Sample_wm(wo, sampler);
    Float R = fresnel_dielectric(std::abs(wo.dot(wm)), m_ex_ior, m_in_ior);
    Float T = Float1 - R;

    if (sampler.sample_1d() < R / (R + T)) {
        // ── 반사 ──
        Vector3f wi = reflect(-wo, wm);
        if (wi[2] * wo[2] <= Float0) return {vec3f_zero, vec3f_zero, Float0};

        Float pdf_ = m_distrib.PDF(wo, wm) /
                     (static_cast<Float>(4) * std::abs(wo.dot(wm)))
                     * R / (R + T);
        Float D = m_distrib.D(wm);
        Float G = m_distrib.G(wo, wi);
        Float f = D * R * G /
                  (static_cast<Float>(4) * std::abs(wi[2]) * std::abs(wo[2]));

        Vector3f weight = Vector3f{f, f, f} * wi[2] / pdf_;
        return {wi, weight, pdf_};
    } else {
        // ── 투과 ──
        // refract 시 마이크로면 법선 ω_m 기준으로 굴절
        Float eta_ratio = (wo.dot(wm) > Float0)
            ? (m_ex_ior / m_in_ior)
            : (m_in_ior / m_ex_ior);
        Float etap = (wo.dot(wm) > Float0) ? m_eta : (Float1 / m_eta);

        // Snell's law: refract wo around wm
        Float cos_i = wo.dot(wm);
        Float sin2_t = eta_ratio * eta_ratio * (Float1 - cos_i * cos_i);
        if (sin2_t >= Float1) return {vec3f_zero, vec3f_zero, Float0};  // TIR
        Float cos_t = std::sqrt(Float1 - sin2_t);
        Vector3f wi = eta_ratio * (-wo) + (eta_ratio * cos_i - cos_t) * wm;

        if (wi[2] * wo[2] >= Float0 || wi[2] == Float0)
            return {vec3f_zero, vec3f_zero, Float0};

        Float denom = wi.dot(wm) + wo.dot(wm) / etap;
        Float dwm_dwi = std::abs(wi.dot(wm)) / (denom * denom);
        Float pdf_ = m_distrib.PDF(wo, wm) * dwm_dwi * T / (R + T);

        Float D = m_distrib.D(wm);
        Float G = m_distrib.G(wo, wi);
        Float ft = D * T * G *
                   std::abs(wi.dot(wm) * wo.dot(wm)) /
                   (std::abs(wi[2]) * std::abs(wo[2]) * denom * denom);
        ft /= (etap * etap);  // radiance mode correction

        Vector3f weight = Vector3f{ft, ft, ft} * std::abs(wi[2]) / pdf_;
        return {wi, weight, pdf_};
    }
}

bool RoughDielectric::is_discrete(bool) const {
    return m_distrib.EffectivelySmooth();
}
```

### 9.5 Scene Parser 등록

```cpp
// distribution 문자열을 MicrofacetType으로 변환하는 헬퍼
MicrofacetType parse_microfacet_type(const Json &child) {
    if (!child.contains("distribution")) return MicrofacetType::GGX;  // 기본값
    std::string d = parse_string(child, "distribution");
    if (d == "beckmann") return MicrofacetType::Beckmann;
    if (d == "ggx")      return MicrofacetType::GGX;
    CRM_ERROR("Unknown microfacet distribution: " + d);
}

// scene_parser.cpp의 parse_bsdf()에 추가:
else if (type == "roughconductor") {
    return BSDF::Create<RoughConductor>(
        parse_microfacet_type(child),
        parse_positive_float(child, "alpha"),
        parse_conductor(child, "material"),  // "Au", "Ag", "Al", "Cu"
        parse_positive_float(child, "ex_ior"));
}
else if (type == "roughdielectric") {
    return BSDF::Create<RoughDielectric>(
        parse_microfacet_type(child),
        parse_positive_float(child, "alpha"),
        parse_positive_float(child, "in_ior"),
        parse_positive_float(child, "ex_ior"));
}
```

### 9.6 사용 예시 (Scene JSON)

```json
{
    "bsdf": {
        "type": "roughconductor",
        "distribution": "ggx",
        "alpha": 0.15,
        "material": "Cu",
        "ex_ior": 1.0
    }
}
```

```json
{
    "bsdf": {
        "type": "roughdielectric",
        "distribution": "beckmann",
        "alpha": 0.1,
        "in_ior": 1.5,
        "ex_ior": 1.0
    }
}
```

`distribution` 필드를 생략하면 기본값은 `"ggx"`이다 (mitsuba3은 기본 Beckmann이지만,
GGX가 더 일반적으로 사용되므로 기본값으로 GGX를 채택).

### 9.7 주의 사항

| 함정 | 설명 | 대응 |
|------|------|------|
| **Half-vector 부호** | 투과용 `ω_m = normalize(η·ω_i + ω_o)` 계산 시 ω_m이 매크로 법선 반대편을 향할 수 있음 | `FaceForward` (wm[2] < 0이면 -wm) |
| **Jacobian 혼동** | 반사: `1/(4\|ω_o·ω_m\|)`, 투과: `\|ω_i·ω_m\|/(...)²` — 서로 완전히 다름 | 각각 별도 구현 |
| **η² 보정** | radiance mode에서 투과 시 필요 | `ft /= η²` |
| **Alpha clamping** | α=0이면 0 나눗셈 | `max(α, 1e-4)` |
| **Caramel 방향 convention** | `local_incoming_dir`가 표면을 향함 → `-local_incoming_dir`가 pbrt의 `wo` | 변환 필요 |
| **reflect 함수** | caramel의 `reflect()`는 `v + 2·(-v·n)·n` | 입력 방향 확인 필요 |
| **에너지 손실** | microfacet interreflection 무시로 rough 표면이 어두움 | 알려진 한계, 향후 개선 가능 |
| **backfacing check** | 투과 시 `wm·wi·cosθ_i < 0`이면 물리적으로 불가 | 무효 처리 |

---

## 부록: 수식 요약

### A. 반사 (Conductor / Dielectric 공통)

```
BRDF:  f_r = D(ω_m) · F(ω_o·ω_m) · G(ω_o,ω_i) / (4 · |cosθ_i| · |cosθ_o|)
PDF:   p   = D_ωo(ω_m) / (4 · |ω_o·ω_m|)
Half:  ω_m = normalize(ω_i + ω_o)
```

### B. 투과 (Dielectric 전용)

```
BTDF:  f_t = D(ω_m) · (1-F) · G(ω_i,ω_o) · |ω_i·ω_m| · |ω_o·ω_m|
             / (|cosθ_i| · |cosθ_o| · (ω_i·ω_m + ω_o·ω_m/η)²)

PDF:   p   = D_ωo(ω_m) · |ω_i·ω_m| / (ω_i·ω_m + ω_o·ω_m/η)²  ·  T/(R+T)

Half:  ω_m = normalize(η·ω_i + ω_o)    (FaceForward 적용)

Correction (radiance mode):  f_t /= η²
```

### C. 분포 함수

```
GGX:  D(ω_m) = 1 / (π·α²·cos⁴θ · (1 + tan²θ/α²)²)
      Λ(ω)   = (√(1 + α²·tan²θ) − 1) / 2
      G1(ω)  = 1 / (1 + Λ(ω))
      G(ω_o,ω_i) = 1 / (1 + Λ(ω_o) + Λ(ω_i))
```

### D. Fresnel

```
Dielectric:  F = (r_s² + r_p²) / 2
  r_s = (η_i·cosθ_i − η_t·cosθ_t) / (η_i·cosθ_i + η_t·cosθ_t)
  r_p = (η_t·cosθ_i − η_i·cosθ_t) / (η_t·cosθ_i + η_i·cosθ_t)

Conductor:   F = (|r_s|² + |r_p|²) / 2   (complex η + ik)
```

---

## 참고 파일 위치

| 내용 | pbrt-v4 | mitsuba3 |
|------|---------|----------|
| GGX Distribution | `src/pbrt/util/scattering.h` | `include/mitsuba/render/microfacet.h` |
| Conductor BxDF | `src/pbrt/bxdfs.h` + `bxdfs.cpp` | `src/bsdfs/roughconductor.cpp` |
| Dielectric BxDF | `src/pbrt/bxdfs.h` + `bxdfs.cpp` | `src/bsdfs/roughdielectric.cpp` |
| Fresnel | `src/pbrt/util/scattering.h` | `include/mitsuba/render/fresnel.h` |
| Tests | `src/pbrt/bsdfs_test.cpp` | (pytest based) |
