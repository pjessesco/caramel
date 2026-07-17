# caramel + peanut 성능 감사 (2026-07-15)

6개 서브시스템 (peanut / accel / geometry / render loop / scene graph / 패턴 스윕) 병렬 감사 결과. 51개 finding 을 중복 제거·클러스터링해 정리했다.

전제 조건:
- 코드 레벨 개선만 다룬다. 빌드 플래그·LTO 류는 제외.
- watertight triangle intersection 알고리즘 자체는 유지. 주변 비용 구조만 개선.
- std 관용구 우선. ext/ 서드파티는 제외, peanut 은 포함.

---

## 핵심 요약

렌더 hot path 의 최대 비용은 BVH 순회 구조에 몰려 있다:

1. **leaf 에서 후보 삼각형마다 80바이트 shading payload 전체 생성** — 승자가 아니면 전부 폐기
2. **shadow ray 용 any-hit 쿼리 부재** — 전체 ray 의 절반이 closest-hit 완주
3. **watertight shear 상수를 삼각형마다 재계산** — ray 불변량인데 3 fdiv 반복
4. **모든 random draw 가 devirtualize 불가능한 가상 호출**
5. **light sample 직후 같은 pdf 를 처음부터 재계산**

메모리 쪽 최대 항목: 비발광 mesh 의 죽은 triangle CDF, envmap 분포 테이블 2배 중복, 강제 singleton leaf 로 인한 노드 수 2N.

---

## HIGH

### H1. BVH leaf: 후보 삼각형마다 full RayIntersectInfo 생성
`src/bvh_base.cpp:205`, `src/shapes/triangle_mesh.cpp:252,271,282` — effort: medium

leaf 루프의 `m_traits.ray_intersect()` 가 `TriangleMesh::get_triangle_ray_intersect` 를 호출하는데, watertight 테스트 통과 직후 매번 full shading payload 를 만든다: uv 보간 + `floor()` 2회, hit 위치 보간, normal 보간 + normalize (sqrt + 3 fdiv), 그리고 `Coordinate` 생성자에서 또 한 번 normalize + 3벡터 정규직교 기저 구축 (sqrt + 3 fdiv + cross). 더 가까운 삼각형이 나중에 나오면 전부 버려진다. 반환도 `std::pair<bool, RayIntersectInfo>` (88바이트, cross-TU 라 인라인 불가) 값 전달이라 **miss 조차** 기본 생성자가 ~80바이트를 스토어하고, hit 채택마다 `info = tmp_info` 80바이트 복사가 또 발생한다. ray 하나가 수십 개 삼각형을 테스트하므로 승자 외 후보 전원의 sqrt/fdiv/store 가 전부 낭비.

**개선**: 교차와 shading 분리. 순회 내부 primitive 테스트는 `struct TriHit { Float t, u, v; Index prim; }` (16바이트, 레지스터 반환) 또는 `std::optional<TriHit>` 를 반환. `BVHTree::ray_intersect` 는 best TriHit 만 유지하다가 루프 종료 후 승자 1개에 대해서만 `fill_intersect_info` 를 호출해 보간/normalize/Coordinate 구축을 정확히 1회 수행. 기존 `get_triangle_ray_intersect` 는 비순회 호출자용 thin wrapper 로 유지.

### H2. any-hit(occlusion) 쿼리 부재 — shadow ray 가 closest-hit 완주
`include/bvh_base.h:82`, `src/scene.cpp:86`, `include/shape.h:52` — effort: medium

`Scene::is_visible` 이 NEE light sample 마다 호출되는데 (area/point/env light 전부, 전체 ray 의 대략 절반) boolean 답만 필요하면서 full closest-hit 쿼리를 쓴다. closest-hit 은 첫 hit 에서 종료 못 하고 near-child-first 순회를 t 축소하며 끝까지 진행 + 개선 hit 마다 H1 의 shading frame 비용까지 지불. `is_visible` 은 `.t` 외 전부 버린다.

**개선**: `bool BVHTree::ray_occluded(const Ray&, Float maxt) const` 추가 — `t < maxt` 인 **첫** primitive hit 에서 즉시 true 반환 (순서 순회 불필요, info 생성 없음, 스택 재검사 없음). SceneAccel/Shape/MeshAccel 로 관통시키고 `Scene::is_visible` 은 `maxt = len - EPSILON*1.1` 로 호출해 기존 tolerance 의미 유지.

### H3. watertight shear 상수를 삼각형 테스트마다 재계산
`src/shapes/shape.cpp:96` (83–93 포함) — effort: small

`watertight_intersection` 이 ray 에만 의존하는 값을 매번 재계산한다: max-dimension 축 선택 (3 fabs + 비교 + winding swap 분기) 과 shear 상수 `sx, sy, sz` — fdiv 3회 (Apple M 시리즈에서 ~10+ 사이클, 파이프라인 불량). Woop 논문 자체가 ray 당 공유 가능하다고 명시. 현재 leaf 가 1–2 삼각형이라 ray 당 수십 회 낭비.

**개선**: `Ray` 생성자에서 1회 계산 (`m_d_recip` 캐시하는 기존 패턴 그대로): `std::array<Index,3> m_shear_axis` + `Vector3f m_shear` 멤버 추가, `watertight_intersection` 은 읽기만. Instance 는 로컬 Ray 를 새로 만들므로 자동으로 올바른 로컬 상수 획득. **알고리즘 무변경, ray 불변량 hoist 만.**

### H4. 비발광 mesh 에도 triangle-sampling CDF 빌드
`src/shapes/triangle_mesh.cpp:68` — effort: small

`finalize` 가 무조건 `m_triangle_pdf = Distrib1D(...)` 빌드 — 삼각형당 8바이트 (pdf + cdf 벡터). 소비처는 `AreaLight::sample_pos_nee` 경로뿐이고 Instance emitter 는 자체 `m_world_triangle_pdf` 를 쓴다. 비발광 mesh 에선 전부 죽은 상주 메모리.

**개선**: `if (arealight != nullptr)` 게이트로 감싸기 — `finalize` 가 이미 arealight 포인터를 받는다. solid-angle-polygon 게이트와 같은 패턴.

### H5. Sampler 가상 호출 — draw 마다 vtable indirect
`include/sampler.h:36`, `src/samplers/uniformstd.cpp:43-60` — effort: small

모든 난수가 `virtual Float sample_1d() = 0` 을 거치고 PCG32 본체는 별도 TU 라 어떤 call site 도 인라인 불가. 코어는 정수 ALU ~6개 명령인데 draw 마다 vtable 로드 + indirect branch + call frame (~15–30 사이클) + 주변 최적화 차단. hot call site 24곳 (픽셀 jitter, light pick, warp, RR, BSDF 샘플링 전부).

**개선**: `next_uint32()`/`sample_1d()` 를 sampler.h inline 멤버로 이동. 구현이 하나뿐이므로 `Sampler` 를 구체 PCG32 클래스로 (또는 `using Sampler = UniformStdSampler;`) 접어서 직접 인라인 호출로.

### H6. light sample 직후 pdf 를 처음부터 재계산
`src/lights/area.cpp:134`, `src/integrators/path.cpp:106` (+`:57,73` 관련) — effort: medium

`sample_direct_contribution` 이 {radiance, pos, normal} 만 반환해서 MIS 루프가 방금 뽑은 샘플의 pdf 를 얻으려고 `pdf_solidangle` 을 다시 호출한다. AreaLight 는 동일 polygon 에 대해 `prepare_solid_angle_polygon` 재실행 (vertex 당 normalize, fan 삼각형당 atan) — pdf 는 이미 손에 있던 `1/polygon.solid_angle`. ImageEnvLight 도 동일 패턴.

**개선**: `sample_direct_contribution` 이 pdf 를 함께 반환 (tuple 확장). AreaLight 는 이미 구축한 polygon 에서, ImageEnvLight 는 방금 뽑은 pmf 에서 직접 계산.

관련: `path.cpp:57,73` — escape/light-hit 분기에서 `from_specular == true` 면 weight 가 버려지는데 `pdf_solidangle` + `pdf_light` (hash map 조회) 를 무조건 계산. 분기를 먼저 하고 필요할 때만 계산 (M6).

---

## MEDIUM

### M1. BVH 빌드 품질·노드 레이아웃 묶음
- `src/shapes/triangle_mesh.cpp:70` — `max_primitive_num = 1` 이라 SAH 종료 비교가 무력화되어 전부 1–2 삼각형 leaf 로 쪼개짐. 노드 ~2N개, 삼각형당 ~72B 순수 노드 오버헤드 (1M tri ≈ 72 MB). scene BVH 처럼 4로 올려 SAH 가 실제로 leaf 를 결정하게. per-tri 테스트가 싸진 뒤 (H1/H3) cost 상수 재튜닝.
- `include/bvh_base.h:40` — `LinearBVHNode` 36바이트: 128B 캐시라인에 3.56개, ~1/4 fetch 가 라인 경계 걸침. `alignas(32)` + `std::int32_t offset; std::uint16_t n_primitives; std::uint8_t split_axis;` 로 32바이트 패킹 → 라인당 정확히 4개.
- `src/bvh_base.cpp:74` — 빌드 중 triangle AABB/centroid 를 O(depth) 회 재계산 (binning + partition predicate + 자식 노드 생성자 각각) + 노드당 벡터 ~5개 할당. `PrimInfo {aabb, center}` 배열을 재귀 전 1회 계산, `std::partition` in-place 로 subrange 전달, binning 버퍼는 `std::array` 로 hoist.

### M2. AABB slab test out-of-line + branchy
`src/aabb.cpp:76` — 노드 방문마다 cross-TU 호출, ray 필드 매번 리로드, 최대 9분기. 헤더 inline 이동 + `std::min`/`std::max` 기반 branchless 슬랩으로.

### M3. framebuffer 열 병렬 × row-major 저장 = false sharing
`src/integrators/MCIntegrator.cpp:53` — task 가 열(column) 단위인데 저장은 row-major: 픽셀 write 마다 새 캐시라인 RFO (12바이트 쓰려고 128바이트), 인접 열 ~10개가 한 라인 공유. `parallel_for` 를 행(row) 단위로 스왑 — write 연속화, 스레드 간 공유는 행 경계뿐.

### M4. Distrib1D/2D — pdf 배열이 cdf 와 완전 중복 + 힙 파편화
`include/distribution.h:83,115` — `pdf[i] == cdf[i] - cdf[i-1]` 이므로 pdf 벡터는 순수 중복. envmap 4096×2048 기준 분포 테이블 ~67 MB → ~34 MB. 게다가 열마다 Distrib1D 2개 힙 블록 = 2W+2 개 산재 할당. pdf 는 cdf 차분으로 계산 (binary search 가 이미 만진 라인의 인접 로드 2개), Distrib2D 는 단일 연속 `std::vector<Float>` (row pitch H) + marginal cdf 로 평탄화.

### M5. warp_sample 역삼각함수 왕복
`include/warp_sample.h:56,80,133` — `acos` 직후 `sin/cos` (cos(acos(x))==x), Beckmann 은 `atan(sqrt(...))` 직후 cos/sin + pdf 에서 `exp(log(...))` 왕복. 전부 대수적으로 제거: sphere 는 `cos_theta = 1-2s; sin_theta = sqrt(1-cos²)`, Beckmann 은 `t2 = -α²log(1-s2); cos_theta = 1/sqrt(1+t2); sin_theta = sqrt(t2)·cos_theta`, pdf 의 exp 항은 `(1-s2)` 재사용.

### M6. MIS weight 를 specular/primary 경로에서 계산 후 폐기
`src/integrators/path.cpp:57,73` — H6 참조. 분기 먼저: `from_specular ? Float1 : balance_heuristic(...)`.

### M7. Microfacet — 중복 normalize ~6회 + Beckmann D 2회
`src/bsdfs/microfacet.cpp:58,66,93` — `sample_recursive_dir` 가 public `pdf()` → `get_reflection()` 을 체이닝해서 이미 unit 인 방향들을 반복 normalize, 같은 half-vector 와 Beckmann D 를 2회 계산. `{wh, D, dot}` 1회 계산하는 private helper 로 공유, 로컬 unit 입력의 방어적 normalize 삭제.

### M8. transform_point/vector — Peanut MatrixMult 가 피연산자 전체 eager 복사
`include/transform.h:34`, `ext/peanut/.../matrix_mult.h:47` — `MatrixMult` 생성자가 양쪽 피연산자를 멤버로 eval 하므로 `mat * Vector4f` 마다 64바이트 Matrix44f 복사 + Vector4f 임시. affine 직접 계산 (`mat(0,0)*p[0] + ... + mat(0,3)`, 9 fma, 복사 0) 으로 교체.

### M9. leaf 삼각형 fetch 가 4단계 의존 간접참조
`src/shapes/triangle_mesh.cpp:238` — `m_ordered_primitives[...]` → `m_face_indices[...]` → 산재한 vertex 3개: 직렬 주소 의존으로 4–5개 라인. leaf 순서로 위치 배열 (`std::array<Vector3f,3>`/tri) 을 구워서 watertight 테스트가 연속 36바이트만 읽게 (+36B/tri 비용, H1 의 deferred shading 과 조합).

### M10. area light — 평면임을 알면서 BVH 순회로 표면점 복원
`src/lights/area.cpp:82` — solid-angle 경로는 planar convex polygon 일 때만 타는데, 방향 샘플 후 표면점을 full `ray_intersect` 로 복원. 평면 (점 + normal) 을 finalize 에서 저장하고 ray-plane 교차 직접 계산 — BVH 순회 통째로 제거.

### M11. Peanut normalize — fdiv 3회
`ext/peanut/.../matrix.h:367` — 원소마다 `/len`. M 시리즈 FP div/sqrt 유닛 1개라 sqrt 뒤에 직렬화. `inv_len = 1/length()` 1회 + fmul 3회로.

### M12. Coordinate 이중(instance 는 삼중) normalize
`include/coordinate.h:46` — 호출부가 `.normalize()` 한 벡터를 `Coordinate` 생성자가 또 normalize. 호출부 3곳 (`triangle_mesh.cpp:278`, `triangle.cpp:110`, `instance.cpp:115`) 의 `.normalize()` 삭제, 생성자 1회로.

### M13. scene_parser — nlohmann::json 서브트리 deep copy
`src/scene_parser.cpp:413` — `get_unique_first_elem` 이 `Json` 값 반환이라 노드 단위 힙 할당 deep clone. `parse_shapes` 는 "shape" 배열 전체를 clone — inline geometry (Moana 스케일 경로) 에서 치명적. `const Json&` 반환으로.

---

## LOW (한 줄씩)

| 위치 | 문제 → 개선 |
|---|---|
| `src/shapes/instance.cpp:103,114` | hit 마다 direction length 2회 + normal matrix `T(m_to_local)` 재구축 → 멤버로 캐시, pre-normalized Ray factory |
| `include/ray.h:32` | 이미 unit 인 방향 재정규화 (recursive_ray_to, is_visible 등) → `from_unit` factory |
| `include/rayintersectinfo.h:41` | 멤버 순서 패딩 8바이트 → `shape*` 를 끝으로 재배열, 80→72B |
| `include/shape.h:119` | 단독 Triangle 이 3점/3normal 을 heap `std::vector` 2개로 → `std::array<Vector3f,3>` inline |
| `src/scene_accel/bvh_scene.cpp:35` | scene-BVH traits wrapper 에서 RayIntersectInfo 80B 복사 1회 추가 → H1 구조로 흡수 |
| `src/scene.cpp:97` | `pdf_light` 가 포인터 키 unordered_map 조회/query → light 에 index 멤버 저장 |
| `src/cameras/pinhole.cpp:41` | ray 마다 4×4 곱 2회 + normalize 2회 → `d_base + w·d_dx + h·d_dy` 사전계산 |
| `include/warp_sample.h` 외 | (M5 에 포함) |
| `src/integrators/path.cpp:122` | RR 에서 `current_brdf.max()` 2회 → 1회 |
| `src/integrators/MCIntegrator.cpp:55` | task 마다 `std::random_device` 생성 (미사용이어도) → seed 산식으로 |
| `include/parallel_for.h:71` | 호출마다 스레드 생성/join + 중복 completion CV/atomic → CV 삭제, join 으로 충분; std::function 소거 |
| `src/progress.cpp:50` | 모든 렌더 스레드가 mutex 직렬화 + 락 잡고 string 할당/flush → `std::atomic` fetch_add, 출력 스레드만 락 |
| `src/image.cpp:161` | envmap luminance 테이블 2회 구축 + row-major 픽셀을 column-major 로 순회 → 1회 구축 + 합계 캐시 |
| `include/transform.h:42` | `transform_normal` 이 vertex 마다 4×4 inverse (cofactor 16개) — 로드 타임이지만 O(V) → 루프 밖 hoist |
| `src/shapes/objmesh.cpp:71` | OBJ weld `std::map` (RB tree) → `unordered_map` + reserve |
| `include/shape.h:78` | `Shape::Create` 파라미터 팩 값 전달 — mesh 배열 로드 시 2회 복사 → forwarding reference |
| `ext/peanut/.../matrix_div_scalar.h:61` | 원소별 fdiv + 생성자 is_zero 체크/throw → reciprocal 1회 + fmul |
| `src/scene_accel/bvh_scene.cpp:42` | scene BVH raw owning pointer + virtual dtor 부재 (릭) → `std::unique_ptr` |
| `include/mesh_accel.h:80` | Octree/Naive 죽은 코드 (부활 시 레이아웃 캐시 불량) |
| `src/shapes/triangle_mesh.cpp:234` | light-sampling 경로에서 모든 호출자가 버리는 area pdf (cross+sqrt+div) 계산 |

---

## 착수 순서 제안

효과/노력 비 기준:

1. **H3** shear 상수 Ray hoist — small effort, 순회 최내곽 직격
2. **H4** CDF 게이트 — small, 메모리 즉효
3. **H5** sampler devirt — small
4. **M5, M6, M11, M12** — 전부 small, 국소 수정
5. **H1** intersect/shading 분리 — 구조 변경 중 최대 효과, H2·M9 의 토대
6. **H2** any-hit 쿼리 — H1 구조 위에서
7. **H6** pdf 반환 API — light 인터페이스 변경
8. **M1** BVH 빌드/노드 (H1·H3 이후 cost 재튜닝), **M3** row 병렬화, **M4** 분포 평탄화, **M8** affine transform
9. LOW 목록은 인접 파일 작업 시 함께

Moana island import (대형 씬) 관점 직결 항목: M1 (노드 메모리), M4 (envmap 테이블), H4 (죽은 CDF), M13 (parser deep copy), LOW 의 `Shape::Create` 복사.

---

*원본: workflow 6-finder 감사, 51 findings. 세부 근거 스니펫은 각 finder 가 현재 코드에서 발췌 — 라인 번호는 2026-07-15 HEAD (`6fc8930`) 기준.*
