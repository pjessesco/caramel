# Matt Pharr 블로그 → caramel 적용 조사

> 출처: <https://pharr.org/matt/blog/>
> 목적: caramel 렌더러에 적용할 만한 아이디어 조사 (조사만, 코드 변경 없음)
> 조사 규모: 블로그 글 57편 수집 → 14편 정밀 정독 → 18개 에이전트, 822k 토큰
> 작성일: 2026-07-10

---

## 0. caramel 현황 요약

caramel은 **CPU 전용 오프라인 단방향 path tracer**다. 자작 헤더-온리 선형대수 라이브러리 "Peanut" 기반. JSON 씬을 2-level SAH BVH + MIS path tracing integrator + RGB(비-분광) radiometry로 렌더해 linear-HDR EXR을 출력. macOS 전용 ImGui/OpenGL GUI(래스터 프리뷰 + 프로그레시브 인터랙티브 렌더) 포함. MIT 라이선스, 1차 코드 약 8.8k LOC.

### 주요 gap (조사 근거)

- **분광 렌더링 없음** — RGB(`Vector3f`) 전용. conductor eta/k는 RGB triple로 mitsuba3 RGB 모드 대비 검증됨. `luminance()=(r+g+b)/3`(Rec.709 가중 아님 = naive 평균) → light-power 가중 + envmap MIS 보정 importance에 사용됨.
- **볼류메트릭/SSS/BDPT/MLT/photon/ReSTIR 없음** — 단방향 path tracer 하나뿐.
- `DirectIntegrator`는 죽은 코드 (`src/integrators/direct.cpp` 전체 주석 처리, `scene_parser.cpp`·`integrators.h`에서 비활성). path + AOV integrator만 실제 선택 가능.
- Octree·NaiveMeshAccel는 바이너리에 빌드되지만 인스턴스화 안 됨 (`TriangleMesh::finalize`가 BVHMesh 하드코딩) — 죽은/대체 경로.
- **sampler 하나뿐** (PCG32 uniform). stratification/QMC/blue-noise/adaptive 없음. 서브픽셀 필터는 box(지터링)뿐 — Gaussian/Mitchell 재구성 필터 없음.
- **고정 `EPSILON=1e-3` ray offset** (`common.h`) — `common.h:54`, `rayintersectinfo.cpp:37`에 adaptive/geometry-normal offset TODO 명시. 대규모 씬·grazing 각도에서 shadow acne / light leak 위험.
- Dielectric는 smooth 전용(rough/GGX dielectric 없음). `dielectric.cpp:59`에 비대칭 산란(eta² 방사전달 보정) 누락 TODO. Microfacet은 Beckmann + G1 근사식(정확한 Smith-GGX 아님).
- 출력 EXR 전용 — CLI에서 LDR/PNG/톤맵 없음, 오프라인 파이프라인에 감마/톤맵 연산 없음(GUI 표시만 sRGB 인코딩). 이미지 3채널 하드코딩, alpha/AOV 버퍼 없음.
- GUI macOS 전용 (`if(APPLE)`, `<OpenGL/gl3.h>`, Cocoa/IOKit). `ImageEnvLight::pdf_solidangle`에 자기의심 `// ???` 주석(`imageEnvLight.cpp:174`).
- OBJ 로더가 shape 2개 이상 파일 거부(`objmesh.cpp`). solid-angle area-light 샘플링은 convex·coplanar·단일경계 폴리곤(`MAX_POLYGON_VERTEX_COUNT=8`)만 지원, 아니면 조용히 area 샘플링으로 폴백.
- **out-of-the-box 빌드/테스트 불가** — 모든 서브모듈(peanut, glfw, imgui, hypothesis, ImGuiFileDialog, caramel-scenes) 미초기화.
- raw `new` 수동 메모리 관리, 정리 없음 — `build_scene`가 Scene/Integrator/Shapes/Lights/Camera 누수. 일회성 CLI엔 무해하나 GUI 씬 재로딩 경로에서 누적.

---

## 1. Top Picks (우선순위 순)

| # | 아이디어 | 노력 | 효과 | 핵심 |
|---|---------|------|------|------|
| 1 | 테스트가 전체 평균만 검사 → MSE/RMSE + furnace | 낮음 | **높음** | `mse`/`rmse` 이미 구현됨, 미사용 |
| 2 | 삼각형 커널에 Kahan difference-of-products | 낮음 | 중간 | double 폴백 축소/제거 |
| 3 | CHECK_RARE 통계 분기 카운터 | 중간 | 중간 | **실제 잠복 버그** 포착 |
| 4 | envmap `// ???` 해소 (chi-square) | 낮음 | 중간 | 수학은 이미 정확, 검증만 |
| 5 | 스레딩 독립 결정성 (per-pixel 해시 시드) | 낮음 | 중간 | 재현성 결합 해소 |
| 6 | 유닛 테스트 확장 (round-trip warp 등) | 중간 | 중간 | 기존 chi2 위에 확장 |
| 7 | Wavefront/SoA → SIMD/GPU | 높음 | **높음** | 장기 재설계, Amdahl 조각은 지금 |

### 1. 테스트에서 전체 이미지 평균 이상을 검사하라 (MSE/RMSE + furnace/해석해) ⭐
- **문제**: 모든 렌더 테스트(`simple_render_test.cpp` + `utils.h`의 `TEST_BODY`)가 `avg(rendered)/avg(ref)` — 전 픽셀·전 채널의 스칼라 평균 하나만 검사. 에너지 **재분포**에 불변 → light leak(고정 1e-3 EPSILON), BSDF 로브/부호 오류, geometry offset, envmap pdf 오류를 조용히 통과.
- **핵심**: `mse()`·`rmse()`·`diff()`·`square()`가 **`test/utils.cpp`에 이미 구현돼 있으나 호출된 적 없음** (`rmse`엔 `// TODO : add test`까지). `CHECK(mse(...) < tol)` 추가 = 몇 줄.
- white-furnace 테스트(ConstantEnvLight 아래 어떤 BSDF든 환경 radiance를 정확히 반환) = 부트스트랩 아닌 정확한 에너지보존 ground truth. 현재 모든 `gt.exr`는 caramel 자체 렌더의 부트스트랩 → 잠복 버그를 박제할 수 있음.
- conductor는 이미 mitsuba3 대비 검증됨 → mitsuba3 출처 `gt.exr`로 "신뢰 가능한 제3자 렌더러" 아이디어 실현. lego 케이스는 `// causes hang in github CI`로 주석 처리됨 → 해법은 삭제가 아니라 저해상도/저-spp 빠른 티어.
- **touchpoints**: `test/utils.cpp`(미사용 함수), `test/utils.h`(`TEST_BODY`), `test/simple_render_test.cpp`, `test/complex_render_test.cpp`; caramel-scenes 서브모듈에 furnace/해석해 씬 + 선택적 mitsuba3 `gt.exr`.
- 출처: [Debugging Your Renderer (4/n): End-to-end tests](https://pharr.org/matt/blog/2021/12/19/debugging-renderers-end-to-end-tests.html)

### 2. 삼각형 교차 커널·외적에 Kahan difference-of-products (FMA)
- **문제**: watertight 커널이 스케일된 barycentric `U=cx*by-cy*bx`, `V=ax*cy-ay*cx`, `W=bx*ay-by*ax`(`shape.cpp:113-115`)를 계산 — 전형적 difference-of-products. 값이 0이면 **현재 `Double`로 재계산**(`shape.cpp:118-130`) — Pharr가 제거하는 바로 그 "double 강제" 케이스.
- **기법**: `DifferenceOfProducts(a,b,c,d)=fma(a,b,-c*d)+fma(-c,d,c*d)` — ≤1.5 ulp 정확도, 1.09x 비용(double 폴백은 2.98x). `det=U+V+W`는 `SumOfProducts` 후보. `Vector3f::cross`(moller_trumbore, `triangle.cpp` 법선/면적)도 수혜. `std::fma`는 `polygon_sampling.h`에서 이미 관용구.
- **주의**: 교차 정확도는 개선하나 그 자체로 1e-3 EPSILON을 낮추진 못함(그건 pbrt식 running-error bound 필요). 보완재.
- **touchpoints**: `src/shapes/shape.cpp`(barycentric + double 폴백 + det, moller_trumbore), `src/shapes/triangle.cpp`(법선/면적), `ext/peanut`의 `Vector3f::cross`(서브모듈).
- 출처: [Accurate Differences of Products with Kahan's Algorithm](https://pharr.org/matt/blog/2019/11/03/difference-of-floats.html)

### 3. CHECK_RARE 통계 분기 카운터 — **실제 버그에 연결** ⭐
- **검증된 잠복 버그** (Pharr의 일화와 거의 동일): `Dielectric::sample_recursive_dir`가 Fresnel 비율로 반사/굴절 선택하지만, `refract()`(`bsdf.cpp:42-49`)는 `sin_t`를 **독립적으로 재계산하며 TIR 가드 없음** → 임계각 근처에서 두 `sin_t` 계산이 어긋나면 `cos_t=sqrt(1-sin_t²)=sqrt(음수)=NaN`. 조용한 NaN/firefly 원인, caramel엔 firefly 클램핑도 없음.
- **기법**: `CHECK_RARE(maxFrequency, condition)` 매크로 — thread_local 카운터 증가 후 종료 시 허용 빈도 초과할 때만 플래그. hard assert(가짜 크래시)와 silent clamp(숨은 버그)의 중간. `refract()` 안 `CHECK_RARE(1e-6, sin_t>=1)`가 정확히 이 버그 포착.
- 그 외 감시 대상: AreaLight solid-angle→area 폴백(`area.cpp`, 의도한 경로를 씬이 아예 안 타는지 = 조용한 분산 회귀), AABB near-zero-direction 분기, envmap pole 분모 `sin(uv[1]*PI)→0`(`imageEnvLight.cpp:174`).
- **touchpoints**: `CRM_ERROR`/`CRM_WARNING` 옆 새 매크로(`include/logger.h`), `src/bsdfs/bsdf.cpp`(refract TIR), `src/lights/area.cpp`, `src/lights/imageEnvLight.cpp:174`; thread_local은 `include/parallel_for.h`에 적합.
- 출처: [CHECK_RARE and making sense of unusual occurrences](https://pharr.org/matt/blog/2018/05/31/check-rare.html)

### 4. ImageEnvLight 등장방형 importance sampler 검증 (`// ???`를 chi-square로 해소)
- **검증 결과**: caramel 수학은 **이미 정확**, pbrt-v3 `InfiniteAreaLight`와 일치 — 빌드 시 `luminance × sin(theta)`(`get_data_for_sampling(true)`), `pdf_solidangle`는 `width*height*PMF/(2*pi²*sin(theta))` + 올바른 pole 가드 반환. 유도식이 `imageEnvLight.cpp:174` 위 주석에 그대로 적혀 있음에도 `// ???` 자기의심만 남아 있고 아무것도 이를 고정하지 않음.
- **최소 노력 액션**: 주석 확정 + `chi2_polygon_test.cpp`/`chi2_bsdf_test.cpp` 미러링한 chi-square 테스트 추가. 3개 글에 흩어진 envmap 우려를 한 액션으로 통합.
- 부수: importance가 `luminance()=(r+g+b)/3`(naive 평균 gap) 사용 → Rec.709 가중은 별개의 소규모 품질 개선. 등면적 octahedral map 마이그레이션은 더 큰 선택적 업그레이드(pole 특이점·극지 해상도 낭비 제거).
- **touchpoints**: `src/lights/imageEnvLight.cpp`, `include/distribution.h`(Distrib2D), `src/image.cpp:158`, `include/common.h`; `ext/hypothesis` 사용한 새 테스트.
- 출처: [Visualizing Warping Strategies for Sampling Environment Map Lights](https://pharr.org/matt/blog/2019/06/05/visualizing-env-light-warpings.html)

### 5. 스레딩 독립 결정성 (per-pixel-sample 해시 시드)
- **문제**: caramel은 현재 **우연히만** 결정적 — seed=열 인덱스, 한 열을 한 스레드가 끝까지, 순차 row/spp 루프가 RNG 소비·box-filter 합산 순서를 고정(`MCIntegrator.cpp` 검증). 결정성이 열 작업분할에 결합됨. perfNotes: width < hardware_concurrency일 때 열 granularity가 코어 저활용 → 향후 tile/row 재병렬화 시 재현성 조용히 깨짐.
- **기법**: `StartPixelSample(p,i)=SetSequence(Hash(x,y,seed))` 후 `Advance(i*k)` — PCG의 독립 (state, sequence) 활용. PCG32가 이미 (state, inc) 구조 + 생성자가 stream 인자 받음 → 작은 추가(단 `Advance()`는 아직 없음). single-pixel debug replay 가능(end-to-end만 있는 테스트가 결여한 것).
- **주의**: 재시드는 per-pixel 노이즈를 재생성 → tolerance 기반 `gt.exr` 재검증 필요.
- **touchpoints**: `include/sampler.h` + `src/samplers/uniformstd.cpp`, `src/integrators/MCIntegrator.cpp`(열 시딩 지점), `include/parallel_for.h`.
- 출처: [Debugging Your Renderer (5/n): Rendering Deterministically](https://pharr.org/matt/blog/2021/12/24/debugging-renderers-rendering-deterministically.html)

### 6. 유닛 테스트 확장 (round-trip warp, sampler↔pdf 일관성, degenerate 입력)
- caramel이 이미 잘하는 것(Diffuse/OrenNayar/Microfacet BSDF + Peters-2021 폴리곤 chi-square, watertight/degenerate 삼각형) 검증·확장.
- 채울 gap: cosine-hemisphere·Beckmann warp(`include/warp_sample.h`)·ThinLens disk warp의 round-trip(샘플→역변환, ~1e-3 내 복원); `Distrib1D`(전력가중 광원선택)·`Distrib2D` sampler↔pdf 일관성; chi-square 불가능한 delta/rough BSDF(Mirror/Dielectric/Conductor)의 white-furnace/eval-vs-pdf. envmap chi-square는 #4가 이미 커버.
- "절대 미루지 마라 / UBSan / 정확한 float 재현" 규율이 all-float 수학, watertight double 폴백, 1e-3 EPSILON에 대응.
- **touchpoints**: `test/unit_tests.cpp`, `test/chi2_bsdf_test.cpp`, `test/chi2_polygon_test.cpp` 확장; `include/warp_sample.h`, `include/distribution.h`, `src/cameras`; `ext/hypothesis`.
- 출처: [Debugging Your Renderer (2/n): Unit Tests](https://pharr.org/matt/blog/2021/11/26/debugging-renderers-unit-tests.html)

### 7. 장기: Wavefront/SoA path tracer → SIMD/GPU 경로, 렌더 빨라지면 startup 프로파일(Amdahl)
- **근거**: caramel perfNotes가 no-SIMD/no-GPU를 "가장 큰 미개척 속도향상 표면"으로 지목, ray packet/stream 없음·SoA vertex 패킹 없음 명시.
- **기법**: depth-first 재귀(`PathIntegrator::mis_sampling_path` + 열-병렬 `MCIntegrator`)를 breadth-first 스트리밍 스테이지 + SoA 레이아웃(현재 인덱스드 AoS 메시 = 삼각형마다 정점 3개 간접참조 gather)로 재구성. backend-agnostic C++ 코어(BSDF/light/sampler) + backend별 교차 커널만 특화.
- **지금 바로 가능한 조각**: Amdahl — caramel의 startup 비용(단일스레드 재귀 BVH 빌드 + 노드마다 새 vector, OBJ `std::map` weld, envmap `get_data_for_sampling` 두 번 빌드)이 렌더 가속되면 지배적 → 지금 싼 개선.
- **주의**: 전체 wavefront/GPU 재작성은 수개월 재설계(단연 최고 노력). Part 10 자체는 Amdahl/통합코드베이스 교훈이 핵심, SoA/커널 메커니즘은 시리즈 앞부분·책에 있음.
- **touchpoints**: `include/parallel_for.h`, `src/integrators/MCIntegrator.cpp` + `src/integrators/path.cpp`, `include/shape.h` + `src/shapes/triangle_mesh.cpp`(AoS→SoA), `src/bsdfs`/`src/lights`/`src/samplers`(코어), `src/shapes/shape.cpp`(커널); startup: `src/bvh_base.cpp`, `src/shapes/objmesh.cpp`, `src/lights/imageEnvLight.cpp`.
- 출처: [Swallowing the Elephant (Part 10): Rendering on the GPU—Finally](https://pharr.org/matt/blog/2021/07/29/moana-rendered-on-the-gpu.html)

---

## 2. Also Consider (차순위)

- **카메라 공간(ish) 렌더링** — geometry를 평행이동해 카메라를 원점에. float 정밀도가 원점에서 멀수록 저하(값 1 근처 gap ~6e-8 vs 1e6 근처 ~0.06) → 대규모 씬에서 1e-3 EPSILON의 acne/leak 절반 완화. caramel은 이미 로드 시 정점을 world로 굽고(`inline_triangle_mesh.cpp:43`, `plymesh.cpp:67`) 카메라 위치 저장(`camera.h m_cam_to_world`) → 삽입 지점 깔끔. **평행이동만**(BVH 축정렬 유지, Pharr의 ~20% 회귀 회피). 주의: 대규모 스케일에서만 효과(현재 테스트 씬은 미해당), grazing 각도 절반은 미해결(진짜 해법은 pbrt `OffsetRayOrigin`). [출처](https://pharr.org/matt/blog/2018/03/02/rendering-in-camera-space.html)
- **바이너리 지오메트리(PLY) 선호 + OBJ weld를 `unordered_map`으로 + 측정 후 최적화** — OBJ 텍스트 파싱이 pbrt 진짜 핫스팟이었음. caramel엔 이미 바이너리 PLYMesh(happly) 경로 있음, OBJ 로더는 단일-shape(`objmesh.cpp:59`). 정점 weld가 정렬된 `std::map<tuple<int,int,int>,Int>`(`objmesh.cpp:71`) → `unordered_map` + tuple 해시로 몇 줄 개선. 메타교훈: perfNotes가 프로파일 수치 없이 핫스팟 단정 → `perf`로 먼저 측정. 로드타임(렌더타임 아님). [출처](https://pharr.org/matt/blog/2018/07/08/moana-island-pbrt-1.html)
- **씬 전처리 가속: BVH 노드 arena/slab 할당자 + 메시별 병렬 빌드 + content-hash 지오메트리 dedup** — 인스턴싱 자체는 이미 있음(Instance shape). 적용 대상은 스케일링 기계. (1) BVH 빌드가 노드마다 새 `std::vector<Primitive>`(`bvh_base.cpp:135-136`) → arena가 churn 감소, 병렬화 전 선결(안 그러면 malloc-mutex 경합). (2) `Scene::build_accel`+`TriangleMesh::finalize`가 메시별 BVH 직렬 빌드 → `parallel_for`로 메시 간 병렬(GUI 재로딩 도움). (3) 진짜 gap: 명시적 Instance 없이 같은 OBJ/PLY 여러 번 로드 시 content-hash dedup 없음(각자 전체 복사). 로드타임, 중간 노력/효과. [출처](https://pharr.org/matt/blog/2021/07/27/moana-gpu-instances.html)
- **sampler `[0, 1-2^-24]` 불변식 문서화·assert** — 검증: caramel은 Pharr가 비판하는 `/2^32` 버그를 **안 씀**(`(next_uint32()>>8)*0x1.0p-24` 사용, 1.0 반환 불가·round-to-nearest 편향 없음) → 두 정확성 교훈 이미 충족. 잔여 가치: 이 "never-1.0" 불변식이 load-bearing인데 미문서화 — `Distrib1D::sample`의 `upper_bound`(`distribution.h:73`), `sample_beckmann_distrib`의 `log(1-s2)`가 누가 `/2^32`로 "단순화"하거나 1.0 방출 가능한 QMC sampler 추가 시 깨짐. 액션: 불변식 문서화/assert(+ `min(idx,size-1)` 방어 클램프). 낮은 노력/효과. [출처](https://pharr.org/matt/blog/2022/03/05/sampling-fp-unit-interval.html)
- **Basu-Owen 측도보존 삼각형 샘플링 (QMC sampler 추가 후에만)** — caramel은 Pharr가 비판하는 고전 sqrt-warp(`triangle.cpp:62-66`, `triangle_mesh.cpp:216-220`)를 그대로 씀(AreaLight uniform-area 폴백 + Instance emitter). 인터페이스는 맞음(`sample_1d()` 하나). **하드 선결조건**: 보고된 2.17x 분산 개선은 저불일치(low-discrepancy) 입력에서만 — caramel은 uniform PCG32 뿐(QMC는 gap), uniform 입력이면 개선 거의 없음. 게다가 주 area-light 경로는 이미 Peters-2021 solid-angle 샘플링 → 이득은 폴백+인스턴스 emitter로 축소. QMC 도입 후에만 가치. 중간 노력, 오늘은 낮은 효과. [출처](https://pharr.org/matt/blog/2019/02/27/triangle-sampling-1.html)

---

## 3. Not Applicable (해당 없음)

- **Sampling in Floating Point (2/3): 1D Intervals** — caramel의 세 importance 분포(Distrib1D, Distrib2D, 삼각형 면적)가 모두 **이산**(버킷 인덱스 반환, envmap 샘플을 픽셀 중심 배치) → canonical uniform을 연속 float 서브구간에 lerp하는 이 글의 주제를 아예 안 함. Pharr 본인도 잔여 효과 무시 가능이라 평가. envmap을 픽셀 내 연속 샘플링으로 업그레이드할 때만 해당. [출처](https://pharr.org/matt/blog/2022/03/14/sampling-float-intervals.html)
- **Let's Stop Calling it "GGX"** — 순수 저작권/용어 글. NDF 공식·Smith masking 유도 없음 → caramel의 Microfacet(Beckmann + G1 근사)·smooth-only Dielectric gap을 못 채움(그 수학은 Walter et al. 2007 / Heitz 2014). caramel 관련 takeaway는 미용적: 미래 분포를 "GGX" 대신 "TrowbridgeReitz"로 명명. [출처](https://pharr.org/matt/blog/2022/05/06/trowbridge-reitz.html)

---

## 4. 즉시 수정 가능한 검증된 항목 2개 (강조)

블로그 아이디어 "채택"과 무관하게 repo에 이미 잠복한 것:

1. **`refract()` TIR→NaN 버그** (#3) — `bsdf.cpp:42-49`, TIR 가드 없음 → 임계각 근처 NaN/firefly. 실제 결함, 지금 조치 가능.
2. **미사용 `mse`/`rmse`** (#1) — `test/utils.cpp`에 코드 이미 작성됨, 호출만 하면 테스트 사각지대 즉시 해소.
