# Particle Tracing: 이론적 배경 및 구현 계획

> 참고: [pbrt-v3 The Path-Space Measurement Equation](https://www.pbr-book.org/3ed-2018/Light_Transport_III_Bidirectional_Methods/The_Path-Space_Measurement_Equation.html), [pbrt-v3 Bidirectional Path Tracing](https://www.pbr-book.org/3ed-2018/Light_Transport_III_Bidirectional_Methods/Bidirectional_Path_Tracing.html)

---

## 1. 이론적 배경

### 1.1 Path-Space Measurement Equation

픽셀 j의 값 I_j는 measurement equation으로 정의된다:

```
I_j = ∫∫ W_e^j(p, ω) · L_i(p, ω) · |cosθ| dω dA(p)
```

- `W_e^j`: 픽셀 j에 대한 카메라의 importance function
- `L_i(p, ω)`: 점 p에 방향 ω로 도달하는 radiance
- `|cosθ|`: 카메라 센서의 기하학적 cosine factor

핵심 관찰: **Le(emission)와 We(importance)는 수학적으로 대칭적이다.** 빛을 광원에서 카메라로 추적하든, 카메라에서 광원으로 추적하든 동일한 적분을 평가할 수 있다.

### 1.2 Path Integral 형태

Path integral 형태로 확장하면, 길이 n인 경로 x̄ = (x₀, x₁, ..., xₙ)에 대해:

```
I_j = Σ_n ∫ f_j(x̄) dμ(x̄)
```

여기서 measurement contribution function f_j는:

```
f_j(x̄) = Le(x₀→x₁) · [Π_{i=0}^{n-1} G(xᵢ, xᵢ₊₁)] · [Π_{i=1}^{n-1} f_s(xᵢ₋₁→xᵢ→xᵢ₊₁)] · W_e^j(xₙ→xₙ₋₁)
```

- `Le(x₀→x₁)`: x₀에서 x₁ 방향으로의 emitted radiance
- `G(xᵢ, xᵢ₊₁) = |cosθᵢ| · |cosθᵢ₊₁| / |xᵢ - xᵢ₊₁|²`: geometry term
- `f_s`: BSDF (bidirectional scattering distribution function)
- `W_e^j`: camera importance

### 1.3 Camera Importance Function (W_e)

#### Pinhole Camera

Pinhole 카메라는 하나의 점이므로 면적 적분이 소멸하고:

```
I_j = ∫ W_e^j(ω) · L_i(cam, ω) · |cosθ| dω
```

pbrt에 따르면, perspective 카메라의 importance function은:

```
W_e(p, ω) = 1 / (A_pixel · cos⁴θ)
```

여기서:
- `A_pixel`: 이미지 평면(z=1)에서 한 픽셀의 면적
- `θ`: 광선 방향과 카메라 광축 사이의 각도

**cos⁴θ의 유래:**
- `cos³θ`: 입체각(solid angle) → 이미지 평면 면적 변환 (`dΩ = dA · cos³θ`)
- `cos θ`: measurement equation의 `|cosθ|` 항과의 결합

#### A_pixel 계산

이미지 평면(z=1)에서:
- 가로: `2 · tan(fov_x / 2)`
- 세로: `2 · tan(fov_x / 2) / aspect` (aspect = W/H)
- 한 픽셀 면적: `A_pixel = (2 · tan(fov_x/2) / W)²`

#### cosθ 계산

카메라 로컬 좌표에서 방향 벡터 `d_cam`에 대해:
- `cosθ = dot(cam_forward, normalize(point - cam_pos))`
- 카메라 로컬 좌표에서: `cosθ = d_local.z / |d_local|`

### 1.4 Light Emission Sampling

Particle tracing에서는 광원으로부터 ray를 방출해야 한다. pbrt의 `Light::Sample_Le()` 개념:

#### Area Light
1. Shape 표면 위의 점 샘플링 → (position, normal, pdf_pos)
2. 그 점의 upper hemisphere에서 cosine-weighted 방향 샘플링 → (direction, pdf_dir)
3. `Le = L_e` (상수 radiance, 방향이 upper hemisphere일 때)
4. `pdf_dir = cosθ / π` (cosine-weighted hemisphere sampling)

#### Point Light
1. 위치는 고정 (delta distribution) → pdf_pos = 1
2. 균일 구면 방향 샘플링 → (direction, pdf_dir = 1/4π)
3. `Le = I` (radiant intensity)

### 1.5 Particle Tracing 알고리즘

Particle tracing은 광원에서 경로를 생성하여 카메라로 "연결(connect)"하는 방식이다. BDPT 프레임워크에서 **t=1 전략** (카메라 정점 1개)에 해당한다.

#### 알고리즘 개요

```
for each sample (총 N개):
    1. 광원 선택 (power-weighted) → (light, pdf_pick)
    2. 광원에서 ray 방출 (sample_Le) → (ray, Le, normal, pdf_pos, pdf_dir)
    3. beta = Le / (pdf_pick · pdf_pos)

    4. [Area light만] 광원 정점(x₀)을 카메라에 연결 → splat
    5. beta *= |cosθ₀| / pdf_dir

    6. for depth = 1 to max_depth:
        a. ray-scene intersection → (hit, info)
        b. if miss or hit light: break
        c. 현재 정점(xₖ)을 카메라에 연결 → splat
        d. Russian roulette
        e. BSDF sampling → 다음 ray, beta 갱신
```

#### 카메라 연결 (connect_to_camera)

정점 x (throughput beta, incoming direction ω_in)에서 카메라로의 연결:

```
1. ω_cam = normalize(cam_pos - x)
2. d² = |cam_pos - x|²
3. Visibility check: scene.is_visible(x, cam_pos)
4. Raster 좌표 계산: camera.world_to_raster(x) → (pixel_i, pixel_j)
5. 범위 확인: 0 ≤ pixel_i < W, 0 ≤ pixel_j < H
6. Delta BSDF이면 skip (확률 0인 방향)

7. f_s = bsdf.get_reflection(ω_in_local, ω_cam_local)
   (광원 정점이면 f_s = 1)
8. cos_surface = |dot(normal, ω_cam)|
9. cosθ_cam = dot(cam_forward, normalize(x - cam_pos))
10. We = 1 / (A_pixel · cosθ_cam⁴)

11. C = beta · f_s · cos_surface · cosθ_cam · We / d²
      = beta · f_s · cos_surface / (d² · A_pixel · cosθ_cam³)

12. image[pixel_i][pixel_j] += C
```

최종 이미지: 모든 sample 누적 후 N으로 나눈다.

#### Path Throughput (beta) 누적 규칙

| 단계 | beta 갱신 |
|------|----------|
| 초기 (광원 샘플링) | `Le / (pdf_pick · pdf_pos)` |
| 방향 샘플링 후 | `*= \|cosθ_light\| / pdf_dir` |
| 각 bounce에서 BSDF sampling | `*= f_s · \|cosθ\| / pdf_bsdf` (= `sample_recursive_dir`의 두 번째 반환값) |
| Russian roulette survival | `*= 1 / survival_prob` |

### 1.6 BDPT에서의 위치

BDPT는 카메라 부분경로(t개 정점)와 광원 부분경로(s개 정점)를 연결하여 경로를 구성한다:

| 전략 | 설명 | 특성 |
|------|------|------|
| s=0, t≥2 | 순수 path tracing | 카메라에서만 추적 |
| s=1, t≥1 | Next event estimation | 카메라 경로 + 직접 광원 연결 (현재 PathIntegrator) |
| **s≥1, t=1** | **Particle tracing** | **광원 경로 + 카메라 연결 (이번 구현 대상)** |
| s≥2, t≥2 | 일반 양방향 연결 | 두 부분경로 연결 |

Particle tracing (t=1)의 특징:
- 매 iteration마다 **다른 raster 좌표**에 기여 → splatting 필요
- **Caustic** 경로(light → specular → diffuse → camera)에 특히 효과적
- 직접 조명(direct illumination)에는 비효율적
- BDPT에서 MIS와 결합 시, 각 전략의 장점을 활용 가능

### 1.7 Non-Symmetric Scattering 보정

광원에서 추적할 때는 importance transport 모드를 사용한다. 두 가지 비대칭성을 고려해야 한다:

#### 굴절 비대칭 (Refraction)
```
L_i = (η_i² / η_t²) · L_t
```
- Importance transport 시 adjoint BTDF 사용: `f*(p, ω_o, ω_i) = (η_t² / η_i²) · f(p, ω_o, ω_i)`

#### Shading Normal 비대칭
```
보정 계수 = |n_s · ω_o| · |n_g · ω_i| / (|n_g · ω_o| · |n_s · ω_i|)
```

> 첫 구현에서는 굴절 비대칭 보정만 고려하고, shading normal 비대칭은 추후 추가한다. (현재 Caramel에서 shading normal ≈ geometric normal)

---

## 2. Caramel 코드베이스 분석

### 2.1 현재 구조

```
Integrator (abstract)
└── MCIntegrator (abstract, per-pixel 렌더링 루프)
    └── PathIntegrator (MIS path tracing)
```

- `MCIntegrator::render()`: pixel 단위 parallel_for, `get_pixel_value()` 호출
- `PathIntegrator`: camera ray 생성 → scene intersection → light sampling + BSDF sampling (MIS)

### 2.2 사용 가능한 기존 인터페이스

| 인터페이스 | 설명 | Particle Tracing에서의 용도 |
|-----------|------|---------------------------|
| `Scene::sample_light(sampler)` | Power-weighted 광원 선택 | 광원 선택 |
| `Shape::sample_point(sampler)` | 표면 위의 점 샘플링 | Area light 위치 샘플링 |
| `BSDF::sample_recursive_dir()` | BSDF 방향 + throughput 샘플링 | Random walk |
| `BSDF::get_reflection()` | BSDF 값 평가 | 카메라 연결 시 |
| `Scene::ray_intersect()` | Ray-scene intersection | Random walk |
| `Scene::is_visible()` | 가시성 테스트 | 카메라 연결 시 |
| `Image::set_pixel_value()` | 픽셀 값 설정 | (추가 필요: add) |

### 2.3 부족한 인터페이스

1. **`Light::sample_Le()`**: 광원에서 ray 방출 (위치 + 방향 동시 샘플링)
2. **`Camera::world_to_raster()`**: 월드 좌표 → 래스터 좌표 변환
3. **`Camera::We()`**: Importance function 평가
4. **`Image::add_pixel_value()`**: 픽셀 값 원자적 누적
5. **`ParticleTracingIntegrator`**: 새로운 integrator (MCIntegrator가 아닌 Integrator 직접 상속)

---

## 3. 구현 계획

### 3.1 단계 1: Light::sample_Le()

`Light` 기본 클래스에 가상 함수 추가:

```cpp
// light.h
struct LightLeSample {
    Ray ray;           // 방출된 ray
    Vector3f Le;       // emitted radiance (또는 radiant intensity)
    Vector3f normal;   // 광원 표면 법선
    Float pdf_pos;     // position의 area pdf
    Float pdf_dir;     // direction의 solid angle pdf
};

virtual LightLeSample sample_Le(Sampler &sampler) const = 0;
```

#### AreaLight::sample_Le()
```cpp
LightLeSample AreaLight::sample_Le(Sampler &sampler) const {
    // 1. 표면 위의 점 샘플링
    auto [pos, normal, pdf_pos] = m_shape->sample_point(sampler);

    // 2. 법선 기준 cosine-weighted hemisphere 샘플링
    auto [local_dir, pdf_dir] = sample_unit_hemisphere_cosine(sampler);

    // 3. 로컬 → 월드 변환 (normal 기준 좌표계)
    Coordinate coord(normal);
    Vector3f world_dir = coord.to_world(local_dir);

    return {Ray{pos, world_dir}, m_radiance, normal, pdf_pos, pdf_dir};
}
```

#### PointLight::sample_Le()
```cpp
LightLeSample PointLight::sample_Le(Sampler &sampler) const {
    // 균일 구면 방향 샘플링
    auto [dir, pdf_dir] = sample_unit_sphere_uniformly(sampler);

    return {Ray{m_pos, dir}, m_radiant_intensity, dir, Float1, pdf_dir};
}
```

### 3.2 단계 2: Camera Importance 인터페이스

`Camera` 클래스에 추가:

```cpp
// camera.h
struct CameraWiSample {
    Vector3f Wi;       // importance value
    Vector2f raster;   // raster 좌표 (pixel_x, pixel_y)
    Float cos_theta;   // 카메라 광축과의 cosine
    bool valid;        // 유효한 연결인지
};

virtual CameraWiSample eval_We(const Vector3f &world_point) const = 0;
```

#### Pinhole::eval_We()

```cpp
CameraWiSample Pinhole::eval_We(const Vector3f &world_point) const {
    // 1. 카메라 로컬 방향 계산
    Vector3f d_world = world_point - m_pos;
    // world_to_camera rotation = transpose of cam_to_world 3x3
    Vector3f d_local = world_to_cam_rotation * d_world;

    // 2. 카메라 뒤쪽이면 무효
    if (d_local[2] <= 0)
        return {{}, {}, 0, false};

    // 3. 래스터 좌표 계산
    //    이미지 평면(z=1)에서의 위치: (d_local.x/d_local.z, d_local.y/d_local.z)
    //    → camera_to_sample 변환 → [0,1]² → pixel 좌표
    Float inv_z = 1.0f / d_local[2];
    // ... camera_to_sample 적용 또는 직접 계산 ...
    Float raster_x = ...; // 0 ~ W
    Float raster_y = ...; // 0 ~ H

    // 4. 범위 확인
    if (raster_x < 0 || raster_x >= m_w || raster_y < 0 || raster_y >= m_h)
        return {{}, {}, 0, false};

    // 5. cosθ 및 We 계산
    Float d_len = d_world.length();
    Float cos_theta = d_local[2] / d_len;  // = dot(forward, normalize(d_world))
    Float cos4 = cos_theta * cos_theta * cos_theta * cos_theta;

    Float tan_half_fov = tan(deg_to_rad(m_fov_x * 0.5f));
    Float A_pixel = (2.0f * tan_half_fov / m_w) * (2.0f * tan_half_fov / m_w);
    Float We = 1.0f / (A_pixel * cos4);

    // Wi = We / d² (importance per solid angle at the reference point)
    Float d2 = d_world.dot(d_world);
    Vector3f Wi = vec3f_one * We / d2;

    return {Wi, {raster_x, raster_y}, cos_theta, true};
}
```

실제 구현 시, `m_cam_to_world`에서 3x3 rotation의 transpose를 추출하여 `world_to_cam_rotation`으로 사용한다. 또한 `camera_to_sample` 행렬을 멤버로 저장하여 래스터 변환에 사용한다.

### 3.3 단계 3: Image 누적 지원

```cpp
// image.h - 추가
void add_pixel_value(int w, int h, Float r, Float g, Float b);
```

Thread-safe 전략: **per-thread local image buffer** → 최종 merge.
각 스레드가 독립적인 Image를 사용하므로 lock이 불필요하다.

### 3.4 단계 4: ParticleTracingIntegrator

MCIntegrator를 상속하지 않고 Integrator를 직접 상속한다. (MCIntegrator는 per-pixel 루프를 가정하므로 splatting 기반 particle tracing과 맞지 않음)

```cpp
// integrators.h - 추가
class ParticleTracingIntegrator final : public Integrator {
public:
    ParticleTracingIntegrator(Index rr_depth, Index max_depth, Index spp);
    void pre_process(const Scene &scene) override;
    Image render(const Scene &scene) override;

private:
    void trace_light_path(const Scene &scene, Sampler &sampler, Image &image) const;
    void connect_to_camera(const Scene &scene,
                           const Vector3f &pos,
                           const Coordinate &coord,
                           const Vector3f &beta,
                           const Vector3f *incoming_dir_local,  // nullptr for light vertex
                           const BSDF *bsdf,                   // nullptr for light vertex
                           Image &image) const;

    Index m_rr_depth;
    Index m_max_depth;
    Index m_spp;        // spp * W * H = 총 light path 수
};
```

#### render() 구현 (의사코드)

```cpp
Image ParticleTracingIntegrator::render(const Scene &scene) {
    auto [W, H] = scene.m_cam->get_size();
    Index N = m_spp * W * H;  // 총 light path 수

    // 스레드 수만큼 local image 생성
    int num_threads = hardware_concurrency();
    std::vector<Image> thread_images(num_threads, Image(W, H));
    int paths_per_thread = N / num_threads;

    parallel_for(0, num_threads, [&](int tid) {
        UniformStdSampler sampler(tid);
        for (int i = 0; i < paths_per_thread; i++) {
            trace_light_path(scene, sampler, thread_images[tid]);
        }
    });

    // Merge 및 정규화
    Image result(W, H);
    for (int i = 0; i < W; i++) {
        for (int j = 0; j < H; j++) {
            Vector3f sum = vec3f_zero;
            for (auto &img : thread_images) {
                sum = sum + img.get_pixel_value(i, j);
            }
            sum = sum / static_cast<Float>(N);
            result.set_pixel_value(i, j, sum[0], sum[1], sum[2]);
        }
    }
    return result;
}
```

#### trace_light_path() 구현 (의사코드)

```cpp
void ParticleTracingIntegrator::trace_light_path(
    const Scene &scene, Sampler &sampler, Image &image) const
{
    // 1. 광원 선택 및 ray 방출
    auto [light, pdf_pick] = scene.sample_light(sampler);
    auto [ray, Le, light_normal, pdf_pos, pdf_dir] = light->sample_Le(sampler);

    Vector3f beta = Le / (pdf_pick * pdf_pos);

    // 2. 광원 정점 → 카메라 연결 (area light만, point light는 skip)
    if (!light->is_delta()) {
        Coordinate light_coord(light_normal);
        connect_to_camera(scene, ray.m_o, light_coord, beta,
                          nullptr, nullptr, image);
    }

    // 3. 방향 pdf 반영
    using std::abs;
    Float cos_at_light = abs(light_normal.dot(ray.m_d));
    beta = beta * cos_at_light / pdf_dir;

    // 4. Random walk
    bool from_specular = false;
    for (Index depth = 1; depth <= m_max_depth; depth++) {
        auto [is_hit, info] = scene.ray_intersect(ray);
        if (!is_hit) break;

        // 광원에 도달하면 중단 (light subpath에서는 무시)
        if (info.shape->is_light()) break;

        const BSDF *bsdf = info.shape->get_bsdf();
        const Vector3f local_incoming = info.sh_coord.to_local(ray.m_d);
        const bool is_specular = bsdf->is_discrete(local_incoming[2] < Float0);

        // 5. 현재 정점 → 카메라 연결 (non-specular만)
        if (!is_specular) {
            connect_to_camera(scene, info.p, info.sh_coord, beta,
                              &local_incoming, bsdf, image);
        }

        // 6. Russian roulette
        if (!is_specular && depth >= m_rr_depth) {
            if (beta.max() > sampler.sample_1d()) {
                beta = beta / beta.max();
            } else {
                break;
            }
        }

        // 7. BSDF sampling → 다음 bounce
        auto [local_dir, f_cos_pdf, pdf] = bsdf->sample_recursive_dir(
            local_incoming, info.tex_uv, sampler);
        beta = beta % f_cos_pdf;
        from_specular = is_specular;
        ray = info.recursive_ray_to(local_dir);
    }
}
```

#### connect_to_camera() 구현 (의사코드)

```cpp
void ParticleTracingIntegrator::connect_to_camera(
    const Scene &scene,
    const Vector3f &pos,
    const Coordinate &coord,
    const Vector3f &beta,
    const Vector3f *incoming_dir_local,
    const BSDF *bsdf,
    Image &image) const
{
    const Camera *cam = scene.m_cam;

    // 1. 카메라 importance 평가 (래스터 좌표 + We 계산)
    auto [Wi, raster, cos_theta, valid] = cam->eval_We(pos);
    if (!valid) return;

    // 2. Visibility 확인
    if (!scene.is_visible(pos, cam->get_position())) return;

    // 3. BSDF 평가 (광원 정점이면 1)
    Vector3f f_s;
    Float cos_surface;

    Vector3f dir_to_cam = (cam->get_position() - pos).normalize();
    Vector3f local_to_cam = coord.to_local(dir_to_cam);
    cos_surface = abs(local_to_cam[2]);

    if (bsdf != nullptr && incoming_dir_local != nullptr) {
        f_s = bsdf->get_reflection(*incoming_dir_local, local_to_cam, /* uv */ {});
    } else {
        // 광원 정점: emission은 beta에 이미 포함, 방향성은 cos로 처리
        f_s = vec3f_one;
    }

    if (is_zero(f_s)) return;

    // 4. Contribution 계산
    //    C = beta · f_s · cos_surface · Wi
    //    (Wi = We / d² 이므로 geometry term이 이미 포함)
    Vector3f C = beta % f_s * cos_surface % Wi;

    // 5. Splat to image
    int px = static_cast<int>(raster[0]);
    int py = static_cast<int>(raster[1]);
    image.add_pixel_value(px, py, C[0], C[1], C[2]);
}
```

### 3.5 단계 5: Scene Parser 연동

`scene_parser`에 `"particle_tracing"` integrator 타입 추가:

```json
{
    "integrator": {
        "type": "particle_tracing",
        "rr_depth": 5,
        "max_depth": 16,
        "spp": 64
    }
}
```

### 3.6 구현 순서 요약

| 순서 | 작업 | 파일 |
|------|------|------|
| 1 | `LightLeSample` 구조체 + `Light::sample_Le()` 가상 함수 | `include/light.h` |
| 2 | `AreaLight::sample_Le()` 구현 | `src/lights/area.cpp` |
| 3 | `PointLight::sample_Le()` 구현 | `src/lights/point.cpp` |
| 4 | `CameraWiSample` 구조체 + `Camera::eval_We()` + `Camera::get_position()` | `include/camera.h` |
| 5 | `Pinhole::eval_We()` 구현 (camera_to_sample 행렬 저장 포함) | `src/cameras/pinhole.cpp` |
| 6 | `Image::add_pixel_value()` 추가 | `include/image.h`, `src/image.cpp` |
| 7 | `ParticleTracingIntegrator` 클래스 선언 | `include/integrators.h` |
| 8 | `ParticleTracingIntegrator` 구현 | `src/integrators/particle_tracing.cpp` |
| 9 | Scene parser에 integrator 타입 추가 | `src/scene_parser.cpp` |

---

## 4. 검증 계획

### 4.1 단위 테스트

1. **Camera::eval_We()**: 이미지 중심(θ=0)에서 We = 1/A_pixel 확인, 래스터 좌표가 (W/2, H/2)인지 확인
2. **Light::sample_Le()**: 방출된 ray가 upper hemisphere에 있는지, pdf가 양수인지 확인
3. **Energy conservation**: 모든 light path contribution의 합이 유한한지 확인

### 4.2 렌더링 비교 테스트

1. **Diffuse-only Cornell Box**: Path tracing 결과와 비교 (충분한 spp에서 수렴 확인)
2. **Caustics 장면** (point light → glass sphere → diffuse floor): Particle tracing이 path tracing보다 효율적으로 caustic을 렌더링하는지 확인
3. **직접 조명 장면**: Particle tracing의 high variance 확인 (예상된 단점)

### 4.3 디버그 시각화

- Light path의 첫 번째 bounce만 활성화하여 직접 조명 확인
- Raster coordinate 분포 시각화 (splat 위치가 합리적인지)
- Beta 값 범위 확인 (firefly 원인 파악)

---

## 5. 향후 확장 (BDPT로의 발전)

Particle tracing은 BDPT의 t=1 전략에 해당하므로, 향후 확장 시:

1. **Vertex 추상화**: 카메라/광원/표면 정점을 통합하는 `Vertex` 구조체
2. **Camera subpath 생성**: 기존 path tracing의 random walk 재사용
3. **General connection**: 임의 (s, t) 쌍에 대한 연결
4. **MIS 가중치**: 모든 전략의 pdf를 계산하여 balance heuristic 적용
5. **ThinLens 카메라 지원**: lens area에 대한 적분 추가

이번 particle tracing 구현에서 만드는 `sample_Le()`, `eval_We()`, `add_pixel_value()` 인터페이스는 BDPT 구현 시 그대로 재사용된다.
