# Matt Pharr 블로그 → caramel 적용 조사 (요약)

> **출처**: <https://pharr.org/matt/blog/>
> **목적**: caramel 렌더러에 적용할 만한 아이디어 조사 (조사만, 소스 코드 변경 없음)
> **검증**: 아래 각 pick은 실제 caramel 소스에 대조해 검증됨. 심층 근거·의사코드·설계는 `pharr-blog-deep-dive-ko.md` 참조.
> **주의**: 테스트 인프라 관련 pick(구 T1 end-to-end 테스트, 구 T6 unit 테스트)은 사용자 요청으로 제외(테스트는 별도 수정 예정). 섹션 ID(T2–T5, T7)는 deep-dive와 정합 위해 원본 유지.
> **갱신**: 심층 조사에서 소스 대조로 확인된 정정 사항 반영.

---

## 0. caramel 현황 요약

caramel은 **CPU 전용 오프라인 단방향 path tracer**다. 자작 헤더-온리 선형대수 라이브러리 "Peanut"(`ext/peanut`) 기반. JSON 씬을 2-level SAH BVH + MIS path tracing integrator + RGB(비-분광) radiometry로 렌더해 linear-HDR EXR을 출력. macOS 전용 ImGui/OpenGL GUI 포함. MIT, 1차 코드 약 8.8k LOC.

### 주요 gap (검증됨)

- **분광 렌더링 없음** — RGB(`Vector3f`) 전용. conductor eta/k는 RGB triple, mitsuba3 RGB 모드 대비 검증.
- **볼류메트릭/SSS/BDPT/MLT/photon/ReSTIR 없음** — 단방향 path tracer 하나.
- `DirectIntegrator`는 죽은 코드(`src/integrators/direct.cpp` 전체 주석). path + AOV만 선택 가능.
- Octree·NaiveMeshAccel는 빌드되나 인스턴스화 안 됨(`TriangleMesh::finalize`가 BVHMesh 하드코딩).
- **sampler 하나뿐**(PCG32 uniform). stratification/QMC/blue-noise/adaptive 없음. 서브픽셀 필터 box(지터링)뿐 — Gaussian/Mitchell 재구성 필터 없음.
- **고정 `EPSILON=1e-3` ray offset**(`common.h:55`, TODO 명시). offset을 geometry normal이 아닌 **ray 방향**으로 줌(`rayintersectinfo.cpp:38`, `scene.cpp:84`). → grazing 각도 shadow acne/leak 위험(각도 실패 모드). 단 좌표 크기 실패 모드는 caramel 씬 규모(O(1–10))에선 현재 위험 ~0 (임계 `|coord|~1.6e4`).
- Dielectric smooth 전용. Microfacet은 Beckmann + G1 근사(정확한 Smith-GGX 아님). Mirror/Dielectric/Conductor는 **전부 specular/delta**(`is_discrete()==true`) — rough BSDF은 Microfacet 하나뿐.
- 출력 EXR 전용. 오프라인 파이프라인에 감마/톤맵 없음(GUI 표시만 sRGB).
- GUI macOS 전용. `ImageEnvLight`에 자기의심 `// ???` 주석(`imageEnvLight.cpp:174`).
- OBJ 로더 단일-shape만(`objmesh.cpp:58-59`). solid-angle area-light 샘플링은 convex·coplanar·≤8각형만, 아니면 area 샘플링 폴백.
- raw `new` 수동 메모리, 정리 없음(`build_scene` 누수). GUI 재로딩서 누적.

> **정정 (이전 요약 대비)**
> - ~~"서브모듈 전부 uninitialized, 빌드/테스트 불가"~~ → **틀림**. 워킹트리에 전부 체크아웃됨. `caramel-scenes`는 `ext/`가 아니라 **repo 루트**에 위치(`gt.exr` 실존: `caramel-scenes/attic/gt.exr` 등), `ext/peanut`·`ext/hypothesis` 실존 → **빌드/테스트 가능**.
> - ~~"`luminance()=(r+g+b)/3`은 naive라 문제"~~ → **뉘앙스 정정**. env-**light** 샘플링의 이상 타깃은 지각 luminance가 아니라 **방사 에너지**이므로 `(r+g+b)/3`(합에 비례)이 오히려 물리적으로 타당. Rec.709는 지각 튜닝일 뿐 분산 개선 보장 없음(악화 가능).

---

## 1. Top Picks

| ID | 아이디어 | 노력 | 효과 | 핵심 |
|----|---------|------|------|------|
| T2 | 삼각형 커널·외적에 Kahan difference-of-products | 낮음~중 | 낮음~중 (정확도/견고성) | double 폴백 축소/제거 |
| T3 | CHECK_RARE 통계 분기 카운터 | 중 | 중 | **실제 잠복 버그** 계측 |
| T4 | envmap `// ???` 해소 + 이산 샘플링 결함 | 낮음~중 | 낮음~중 | 수학은 정확, 실결함은 이산 방향 |
| T5 | 스레딩 독립 결정성 (per-pixel 해시 시드) | 낮음 | 중 | 재현성 결합 해소 |
| T7 | Wavefront/SoA → SIMD/GPU (+ Amdahl 조각) | 높음(전체)/낮음(조각) | 장기 높음 | 재설계, startup은 지금 |

### T2. 삼각형 교차 커널·외적에 Kahan difference-of-products (FMA)
- watertight barycentric `U=cx*by-cy*bx` 등이 `shape.cpp:114-116`, 값이 0이면 **`Double`로 재계산**하는 폴백이 `shape.cpp:119-129` — Pharr가 제거하는 바로 그 "double 강제" 케이스. `Vector3f::cross`(`ext/peanut/.../matrix.h:394-399`, per-component naive `a*b-c*d`)도 직접 대상.
- 기법: `DifferenceOfProducts` — `cd=c*d`를 **한 번만** 계산해 `dop=fma(a,b,-cd)`, `err=fma(-c,d,cd)`, `return dop+err`. (⚠ `fma(a,b,-c*d)+fma(-c,d,c*d)`처럼 `c*d`를 두 번 쓰면 오차보정 무효 — 정확성의 핵심.) ≤1.5 ulp, 1.09x 비용(double 승격 2.98x). `std::fma`는 `polygon_sampling.h`서 이미 사용, CMake에 fast-math 플래그 없음.
- **정정**: `det=U+V+W`(`shape.cpp:138`)는 3-항 **합** → DOP 대상 **아님**(이전 오류). DOP 대상은 U/V/W 각각(2x2 행렬식)과 cross. `T=U*az+V*bz+W*cz`(`:147`)도 3-항 합이라 대상 아님.
- 주의: barycentric/법선 정확도만 개선, `EPSILON=1e-3`은 별개 이슈라 안 낮아짐.
- 출처: [Accurate Differences of Products with Kahan's Algorithm](https://pharr.org/matt/blog/2019/11/03/difference-of-floats.html)

### T3. CHECK_RARE 통계 분기 카운터 — 실제 잠복 버그에 연결
- **잠복 버그**: `refract()`(`bsdf.cpp:42-50`)가 `sin_t`를 독립 재계산(`:46`)하고 TIR 가드 없이 `cos_t=sqrt(1-sin_t*sin_t)`(`:47`) → `sqrt(음수)=NaN` 가능. caramel엔 firefly 클램프 없음.
- **정정 (도달성)**: 정통 TIR로는 **도달 불가** — `fresnel_dielectric()`가 `sin_t>=1`서 정확히 `Float1` 반환(`bsdf.cpp:70`) + `sample_1d()`∈[0,1)(`uniformstd.cpp:59`) → **항상 reflect 분기**, refract 미호출. NaN은 (a) fresnel/refract 두 float 식이 TIR 경계서 반올림으로 어긋남, 또는 (b) 미정규화 `dir[2]²>1`(fresnel는 `cos_i` clamp `:61-64`, refract는 안 함)의 **measure-zero/round-off**만. → hard assert 아닌 **통계적 CHECK_RARE가 정확히 맞는 도구**(결론 강화).
- 기법: `CHECK_RARE(maxFreq, cond)` — thread_local 카운터, 종료 시 허용빈도 초과만 플래그.
- **정정 (다른 후보)**: envmap 극점은 **이미 가드됨**(`imageEnvLight.cpp:165-167`, 0/1의 1e-6내면 0 반환) → CHECK_RARE 역할은 가드 발화 빈도 모니터. AABB near-zero(`aabb.cpp:84`)는 축정렬 광선서 일상 발화 → **rare 아님, 부적합**. 더 나은 후보: `area.cpp`의 `isnan(dir)`(`:76-78`), 광원 재교차 실패 `!hit`(`:84-86`), 퇴화 근접(`:96-98`).
- **정정 (스레딩)**: caramel `ThreadPool`은 `parallel_for`마다 `std::thread` 생성/join하는 **비영속** 풀 → worker thread_local이 join서 파괴됨. pbrt식 barrier 못 씀 → **self-registering `RareCounter`**(thread_local 소멸자가 전역 mutex accumulator로 병합) 필요. `render()`는 `parallel_for` 1회 호출(`MCIntegrator.cpp:53`) → 직후 report.
- 출처: [CHECK_RARE and making sense of unusual occurrences](https://pharr.org/matt/blog/2018/05/31/check-rare.html)

### T4. ImageEnvLight importance sampler 검증 (`// ???` 해소) + 이산 샘플링 결함
- 검증 결과: pdf 수학은 **이미 정확**, pbrt-v3 `InfiniteAreaLight`와 일치 — 빌드 시 `luminance×sin(theta)`(`image.cpp:166`, 호출 `imageEnvLight.cpp:48`/`:101`), `pdf_solidangle`에 올바른 pole 가드. `// ???`(`imageEnvLight.cpp:174`)는 근거 없음.
- **정정 (블로그)**: 이 2019 글엔 pdf 유도식·octahedral·pole 논의 **없음** — 두 warping "시각화 비교"(승자 없음)일 뿐. `// ???` 유도는 이 글 아닌 PBR book/InfiniteAreaLight에 있음. (이전 요약이 글 내용 오기)
- **실결함 (`// ???`보다 구체적)**: `Distrib2D::sample`이 정수 픽셀 인덱스만 반환(`distribution.h:99-107`), `sample_direct_contribution`이 `(idx+0.5)/size` **픽셀 중심**만 쓰고 소수 난수 버림(`imageEnvLight.cpp:132`) → 실질 **W×H개 이산 방향뿐**. `pdf_solidangle`이 주장하는 piecewise-constant 연속 밀도와 불일치 = **밴딩/편향 실원인**.
- 부수: 빌드 가중 `sin(h/H*PI)`가 픽셀 top edge 사용 → h=0 최상단 스캔라인 가중 정확히 0(pbrt는 `(v+0.5)/H`). 자기일관적이나 최상단 행이 light 샘플링서 제외 = 효율 손실 + midpoint 위반.
- 출처: [Visualizing Warping Strategies for Sampling Env Map Lights](https://pharr.org/matt/blog/2019/06/05/visualizing-env-light-warpings.html)

### T5. 스레딩 독립 결정성 (per-pixel-sample 해시 시드)
- caramel은 **우연히만** 결정적: seed=열 인덱스, 한 열을 한 스레드가 끝까지, 순차 row/spp 루프(`MCIntegrator.cpp`).
- **정정 (취약성 범위)**: 재병렬화뿐 아니라 더 넓음 — 한 열이 sampler 하나 공유 → 스레딩 안 바꿔도 `sample_1d()` **소비 개수/순서를 바꾸는 어떤 수정**(새 BSDF lobe, RR 조정, draw 재배열)이든 그 열 이후 모든 픽셀 노이즈 이동(consumption cascade). 재병렬화보다 자주 발현.
- **정정 (메커니즘)**: caramel seed는 PCG **initstate**(시작 오프셋)에 대응, sequence(`m_inc`)는 stream 기본 1 **고정** → 모든 열이 "같은 sequence, 다른 시작 오프셋". 블로그는 반대로 Hash로 **sequence를 픽셀별로** 바꿈. `SetSequence`는 신규 구현 불필요 — 기존 생성자 본문(`pcg32_srandom_r` 패턴)을 `set_sequence()`로 추출·재사용.
- **정정 (주의 철회)**: ~~"gt.exr 재베이스라인 필요"~~ → 렌더 테스트는 avg 휘도 비율 비교(`simple_render_test.cpp:53-54`)라 노이즈 재배치에 둔감 → **재베이스라인 불필요**. bit-exact 테스트 신규 추가 시에만 필요.
- 이득: single-pixel debug replay 가능.
- 출처: [Debugging Your Renderer (5/n): Rendering Deterministically](https://pharr.org/matt/blog/2021/12/24/debugging-renderers-rendering-deterministically.html)

### T7. 장기: Wavefront/SoA → SIMD/GPU, 그리고 Amdahl startup 조각
- no-SIMD/no-GPU 검증(단일 `Ray` `ray.h:30-40`, 단일 레이 BVH traversal `bvh_base.cpp:189-241`, CMake에 -march/AVX/CUDA/ISPC/OpenMP/fast-math 전무). ("perfNotes"는 파일 아님 — 코드로 직접 확인.)
- **정정**: `mis_sampling_path`는 재귀 아닌 **bounded iterative loop**(`path.cpp:52` depth 루프). 실제 재귀는 BVH build/flatten(`bvh_base.cpp:144-145,170-172`).
- **정정 (SoA)**: 속성(pos/normal/texcoord)은 이미 별도 `std::vector` = attribute-level SoA(`shape.h:158-160`). SIMD 병목은 (a) 각 `Vector3f`가 xyz-packed 스칼라라 lane 병렬 `x[]/y[]/z[]` 없음, (b) 삼각형마다 `m_face_indices` 통한 간접 gather(`triangle_mesh.cpp:238-241`).
- **지금 가능한 조각 (Amdahl)**: 렌더 가속되면 startup이 지배 → 단일스레드 재귀 BVH 빌드, OBJ weld, **envmap `get_data_for_sampling` 두 번 빌드**(확인: `imageEnvLight.cpp:48`+`:101`, `power()`는 `scene.cpp:113`) 등 지금 개선.
- 주의: 전체 wavefront/GPU는 수개월 재설계(최고 노력). Part 10은 Amdahl/통합코드 교훈이 핵심, SoA/커널 메커니즘은 아님.
- 출처: [Swallowing the Elephant (Part 10)](https://pharr.org/matt/blog/2021/07/29/moana-rendered-on-the-gpu.html)

---

## 2. Also Consider

### A1. 카메라 공간(ish) 렌더링 — geometry 평행이동해 카메라를 원점에
float 정밀도가 원점에서 멀수록 저하 → 대규모 씬서 `EPSILON` acne/leak의 **크기(magnitude) 실패 모드** 완화. **평행이동만**(BVH 축정렬 유지, 회전 시 ~20% 회귀 회피).
- **정정**: 카메라 위치는 `m_cam_to_world`(4x4)가 아니라 `Vector3f m_pos`(`camera.h:65`)이며 `camera.cpp:51`서 이미 `(0,0,0)`을 world로 변환해 저장 → **step-1 신규 코드 불필요**.
- **정정**: world-bake는 obj/ply/trianglemesh 로더만(`inline_triangle_mesh.cpp:43`, `plymesh.cpp:67-71`, `objmesh.cpp:82-87`). recenter 대상에 **`Instance` to_world**(`instance.cpp:98-120`, 의도적 비-bake)와 **standalone `Triangle` 정점**(`scene_parser.cpp:248-263`, raw p0/p1/p2)도 포함해야 함.
- **정량**: `EPSILON=1e-3` 위험 임계 `|coord|~1.6e4`, 1e6서 소멸. caramel/CLO 씬 O(1–10) → 8000+ ULP 여유 → **현재 실질 위험 ~0**(잠재적). grazing-angle 절반은 여전히 미해결(`rayintersectinfo.cpp:37` TODO, offset이 ray 방향).
- 출처: [Rendering in Camera Space(ish)](https://pharr.org/matt/blog/2018/03/02/rendering-in-camera-space.html)

### A2. 바이너리(PLY) 선호 + OBJ weld를 unordered_map + 측정 후 최적화
- **정정 (프레이밍)**: OBJ float 파싱은 caramel 코드 아니라 vendored `ext/tiny_obj_loader.h`가 담당, `strtod` 아닌 자체 고속 `tryParseDouble`(`:866`) 사용 → 블로그가 지목한 pbrt strtod 핫스팟은 **그대로는 없음**.
- **정정 (출처)**: 블로그 part 1은 weld/hashmap·strtod를 실측 안 함(part 3로 미룸). part 1 실측은 PLY 바이너리 변환 이득(1.3x, throughput ~8x)과 perf 프로파일링 방법론. unordered_map은 도입부 일화서 착안.
- 유효 적용: 정점 weld가 정렬된 `std::map<tuple<int,int,int>,Int>`(`objmesh.cpp:71`) → `unordered_map`+tuple 해시로 몇 줄 개선. 단일-shape 거부(`:58-59`), happly PLY(`plymesh.cpp:44`) 확인. 로드타임(렌더 무관), **측정 먼저**.
- 출처: [Swallowing the elephant (part 1)](https://pharr.org/matt/blog/2018/07/08/moana-island-pbrt-1.html)

### A3. 씬 전처리 가속: BVH arena + 병렬 빌드 + content-hash dedup
- **정정 (병렬 대상)**: per-mesh BVH는 `Scene::build_accel`이 아니라 `TriangleMesh::finalize`(`triangle_mesh.cpp:70-71`)서 빌드, `parse_shapes` 직렬 루프(`scene_parser.cpp:150-158`)로 실행. `build_accel`(`scene.cpp:100-103`)은 top-level 씬 BVH **하나**만. → 병렬화 대상은 **`parse_shapes` 루프**(build_accel 아님).
- **정정 (arena)**: **per-thread** arena여야 함(공유 arena는 lock 필요 = malloc-mutex 경합 재현). churn은 `bvh_base.cpp:135-136` 두 vector뿐 아니라 노드마다 `make_unique<BVHNode>`(`:142-143`) + flatten 후 폐기되는 pointer-tree 전체.
- **정정 (dedup)**: caramel은 to_world를 정점에 bake(`objmesh.cpp:82-83`) → raw-buffer 해시는 "동일 파일+동일 transform"만 dedup. Moana식 진짜 이득(한 지오메트리를 여러 transform 공유)은 **auto-instancing**(로더가 local-space 유지 → Instance 라우팅) 필요 = 훨씬 큰 변경.
- **누락 (hard prereq)**: `Logger`가 스레드 비안전(`logger.h:38,44,50` cout 체인 + `:57` `localtime` mutex 없음 → UB) → 병렬 로드 전 선결.
- 출처: [Swallowing the Elephant (Part 9)](https://pharr.org/matt/blog/2021/07/27/moana-gpu-instances.html)

### A4. sampler `[0, 1-2^-24]` 불변식 문서화·assert
- caramel은 버그 있는 `/2^32` 안 씀(`(next_uint32()>>8)*0x1.0p-24`) → 정확성 이미 충족. 단 never-1.0 불변식이 load-bearing인데 미문서화.
- **정정 (위치·범위)**: `Distrib1D::sample`은 `distribution.h:73-76`, load-bearing 지점은 `upper_bound`(`:74`)+index 반환(`:75`). envmap OOB는 **`Distrib1D` 아닌 `Distrib2D::sample`**(`:99-107`) 별도 지점: 1.0 입력 → `m_width_distrib`가 행수 `w` 반환 → `m_height_distrib_list[w]`(`:101`) OOB.
- **정정 (assert)**: 순수 `assert()`는 NDEBUG(Release/렌더)서 컴파일 아웃 → 무력. 동등 안전장치 = 컴파일타임 `static_assert` + `Distrib1D::sample` clamp.
- 출처: [Sampling in Floating Point (1/3)](https://pharr.org/matt/blog/2022/03/05/sampling-fp-unit-interval.html)

### A5. Basu-Owen 측도보존 삼각형 샘플링 (QMC sampler 추가 후에만)
- **하드 선결조건**: 2.17x 분산 이득은 저불일치 입력에서만 — caramel은 uniform PCG32뿐(QMC gap), uniform이면 이득 ~0.
- **정정 (도달 경로)**: sqrt-warp가 AreaLight fallback을 먹인다는 건 부정확. standalone `Triangle`은 절대 이미터 아님(`Shape(bsdf,nullptr)`, `triangle.cpp:35-46`) + `is_solid_angle_sampling_possible()==true`(`:117-119`)라 인스턴스 Triangle 라이트도 Peters-2021 탐. **실제 sqrt-warp 도달점은 `TriangleMesh::get_triangle_sample_point`(`triangle_mesh.cpp:219-220`) 뿐**. instanced emitter도 독립 경로 아님(같은 게이팅).
- **정정 (인터페이스)**: `sample_1d()`는 24비트뿐 → `u*2^32` 재구성 시 하위 8비트 0 → base-4가 4^12로 제한. 깔끔한 구현은 private `next_uint32()`(`uniformstd.cpp:43`)를 Sampler에 노출 필요.
- 출처: [Adventures in Sampling Points on Triangles (Part 1)](https://pharr.org/matt/blog/2019/02/27/triangle-sampling-1.html)

---

## 3. Not Applicable

- **Sampling in Floating Point (2/3): 1D Intervals** — caramel의 세 importance 분포가 모두 이산(버킷 인덱스, 픽셀 중심) → uniform을 연속 float 서브구간에 lerp하는 이 글 주제를 안 함. Pharr도 잔여 효과 무시 가능이라 평가. [출처](https://pharr.org/matt/blog/2022/03/14/sampling-float-intervals.html)
- **Let's Stop Calling it "GGX"** — NDF·Smith 유도 없는 용어 글 → Microfacet/Dielectric gap 못 채움(그 수학은 Walter 2007/Heitz 2014). takeaway는 미용적(명명). [출처](https://pharr.org/matt/blog/2022/05/06/trowbridge-reitz.html)

---

## 4. 즉시 수정 가능한 검증된 항목 (테스트 무관, 정확성)

블로그 아이디어 "채택"과 무관하게 repo에 잠복한 **정확성 버그**:

1. **`refract()` NaN** (T3) — `bsdf.cpp:46-47`, TIR 가드 없음. 정통 TIR로는 도달 불가(항상 reflect)이나, 반올림 어긋남/미정규화 방향의 **measure-zero/round-off**로 발생 가능. 실재하나 희귀 → hard assert 아닌 CHECK_RARE로 계측이 정답.
2. **`Distrib1D`/`Distrib2D` out-of-bounds read** (A4) — 빈/전부-0 분포 또는 1.0 샘플 입력 시. **≥3개 독립 크래시 지점**: 광원 선택(`scene.cpp:93`), envmap `Distrib2D`(`distribution.h:101` `m_height_distrib_list[w]`), 삼각형 면적(`triangle_mesh.cpp:211`). `distribution.h:74-75`의 `upper_bound`가 `end()`→`index==size()` 반환. 방어: `static_assert` + clamp.

> 상세 근거·의사코드·설계: `pharr-blog-deep-dive-ko.md` 참조.
