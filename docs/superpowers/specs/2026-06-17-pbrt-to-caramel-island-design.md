# pbrt-v4 → Caramel: Moana Island 렌더 설계 / 참고 문서

- 작성일: 2026-06-17
- 대상 씬: `/Users/jino/Downloads/island/pbrt-v4` (Disney/WDAS *Moana Island*, pbrt-v4 변환본)
- 레퍼런스 파서: `/Users/jino/repo/pbrt-v4`
- 타깃 렌더러: `/Users/jino/repo/caramel`
- 상태: **설계 문서. 아직 구현 안 함.** 나중에 참고용.

---

## 0. 목표 & 범위 (확정)

- **목표**: island 씬을 caramel에서 렌더.
- **전략 (A)**: caramel 코어 확장 허용. converter 단독으로는 불가능 (인스턴스 31M 확장 = 메모리 사망).
- **포함**:
  - 일반 geometry: triangle mesh, ply mesh, **curve**.
  - **instancing** (필수, 코어에 추가).
- **제외 (일단)**:
  - **ptex** → 텍스처 없이 flat-color fallback.
  - volumetric (`volpath` → surface path만).
  - spectral, measured BRDF.
- **자연 귀결**: instancing + curve 는 caramel `Shape` 서브클래스로 추가. 나머지 비대응 기능은 converter 단계에서 degrade.

---

## 1. 현황 분석

### 1.1 caramel 능력 (타깃)

JSON 씬 포맷. path tracer, EXR 출력. **Y-up.** (핸디드니스는 §3.2.3 — 소스로 미확정, 재확인 필요)

| 영역 | 지원 | 비고 (file:line) |
|---|---|---|
| Geometry | triangle mesh (obj/ply), 단일 triangle | `shape.h:88,140,185` |
| Geometry — 없음 | **instancing 없음, curve 없음, parametric(sphere 등) 없음** | — |
| BSDF | diffuse, mirror, dielectric, conductor(Au/Ag/Al/Cu preset), microfacet, oren-nayar, twosided | `scene_parser.cpp:246-301` |
| Texture | image only, **diffuse albedo 에만** | `scene_parser.cpp:303-310` (ptex 없음) |
| Light | point, area(emissive mesh), constant_env, image_env(MIS) | `scene_parser.cpp:220-243` |
| Light — 없음 | distant/spot 없음 | — |
| Camera | pinhole, thinlens(DoF) | `scene_parser.cpp:101-142` |
| Integrator | path (depth_rr, depth_max, spp), + 디버그(normal/uv/depth/hitpos) | `integrators.h` |
| Sampler | uniform PCG32 only | `sampler.h:33-51` |
| Film | EXR 16-bit half | `image.cpp:75-101` |

핵심 클래스 구조:

```
Shape (abstract, shape.h:46)
 ├─ Triangle (final)
 └─ TriangleMesh (interface, 누적 BVH 보유)
     ├─ OBJMesh (final)
     └─ PLYMesh (final)
```

가속 구조 2단계 **이미 존재**:
- `MeshAccel`/`BVHMesh` (`mesh_accel.h`): 메쉬 내부 triangle BVH (BLAS 역할).
- `SceneAccel`/`BVHScene` (`scene_accel.h`): scene 의 `Shape*` 위 BVH (TLAS 역할). `build(const std::vector<const Shape*>&)`.

→ **instancing 추가에 결정적**: TLAS 가 이미 `Shape*` 다형성으로 dispatch. `Shape` 서브클래스만 만들면 accel 변경 0.

### 1.2 pbrt-v4 포맷 (소스)

- **왼손 좌표계** 확정 (`util/transform.h:382` 주석 "left-handed", `LookAt` basis). caramel 과의 관계는 **변환 필요 여부 미확정** (§3.2.3).
- directive: `Camera/Film/Sampler/Integrator/Accelerator`, `WorldBegin`, `AttributeBegin/End`, `Transform/ConcatTransform/Translate/Rotate/Scale/LookAt`, `Shape`, `Material/MakeNamedMaterial/NamedMaterial`, `Texture`, `LightSource/AreaLightSource`, `ObjectBegin/ObjectEnd/ObjectInstance`, `Include/Import`, `ReverseOrientation`.
- parameter: `"type name" [values]` 타입드 (float/integer/point3/normal/rgb/spectrum/string/bool/texture).
- instancing 데이터 모델: `ObjectBegin "name"` 가 graphics state(material 포함) 캡처한 **템플릿** 생성. `ObjectInstance "name"` 이 현재 CTM 으로 **복제 배치**. 머티리얼은 템플릿에 baked. (`scene.cpp:309-395`)
- 파서 진입점: `parser.cpp:601-987` (directive dispatch), `parser.cpp:272-342` (tokenizer).

### 1.3 island 씬 실측 인벤토리

29 GB. entry `island.pbrt` → `materials.pbrt` include + 19개 asset group `Import`.

| 항목 | 수치 | caramel 대응 |
|---|---|---|
| **ObjectInstance** | **30.9M** (313 def 에서) | ❌ → 코어에 instancing 추가 |
| **ptex texture** | **18,400** (imagemap 0개) | ❌ → flat color fallback |
| **curve** prim | **5.4M** (잔디/잎/hair, flat B-spline) | ❌ → curve 추가 or tessellate |
| trianglemesh | 1.49M | ✅ |
| plymesh / .ply | 14,222 (~22GB) | ✅ ply 로더 있음 |
| material (named) | 97개: diffuse 57, diffusetransmission 22, coateddiffuse 16, dielectric 1, interface 1 | ⚠️ 부분 매핑 (§3.3.4) |
| light | area 22 + infinite 1 (sky equiarea PNG 52MB) | ✅ 매핑 가능 |
| camera | perspective, fov 69.5, lensradius 0.003125, focaldistance 1675.34 | ✅ thinlens |
| film | 1920×804, maxcomponentvalue 10 | ✅ (clamp 미지원) |
| sampler / integrator | zsobol 1024spp / volpath maxdepth 10 | ⚠️ uniform 1024 / path (volume drop) |

> conductor/metal 머티리얼 **0개** → 코어의 conductor preset 한계는 이 씬에선 문제 안 됨.

---

## 2. Gap 분석 — 핵심 blocker

1. **Instancing (P0).** 30.9M 배치. naive 확장 → 수십억 tri, 수백 GB. caramel 인스턴싱 0. → 코어에 `Instance` shape + TLAS 재사용 (TLAS 는 이미 `Shape*` 기반이라 그대로). §3.2.1.
2. **Ptex (P0, 회피).** 씬 100% ptex, 0% imagemap. caramel ptex 못 읽음. → **일단 텍스처 제거, material 상수색 fallback.** ptex 베이크는 후순위(M4). §3.3.4.
3. **Curve (P1).** 5.4M curve. → (권장) converter 에서 ribbon tri 로 tessellate, or 네이티브 `Curve` shape. §3.2.2.
4. **Handedness (P0, watertight, 미확정).** pbrt 왼손 확정. caramel 카메라 basis 가 pbrt 와 동일 구성(`cross(up,dir)`) → **flip 필요 여부 불명**. 경험적 확인(알려진 에셋 렌더 비교). §3.2.3.
5. **volpath → path (degrade).** 참여매질 없음. `interface` material(매질 경계) 은 무시. 결과는 레퍼런스와 다름 (수용).
6. **maxcomponentvalue (firefly clamp).** caramel film clamp 미지원 → 옵션 추가 or 무시.
7. **Sampler 품질.** zsobol → uniform PCG. 같은 spp 에서 노이즈 더 큼 (수용, 후순위).

---

## 3. 아키텍처

### 3.1 전체 워크플로우

```
island.pbrt (+ Import/Include 트리)
        │  ┌─────────────────────────────────────────┐
        ▼  │  pbrt2caramel (신규 standalone 변환기)    │
  [pbrt parse] ─→ [중간표현(IR)] ─→ [handedness 변환] ─→ [emit]
        │                                               │
        │   ObjectBegin → template 정의                  ├─→ scene.json (+ instance/curve)
        │   ObjectInstance → instance + CTM             ├─→ geometry/*.ply  (필요시 복사/참조)
        │   curve → ribbon tri (or curve node)          └─→ env/*.exr (equiarea→equirect 변환)
        ▼
   caramel (코어 확장: Instance, Curve, 좌표 변환)
        ▼
   render.exr
```

변환기는 **별도 프로그램** (caramel 빌드와 분리). pbrt 파서는 레퍼런스(`/Users/jino/repo/pbrt-v4`) 문법을 따르되, 풀 파서 재구현 대신 **필요 directive 만** 처리하는 경량 파서로 시작.

### 3.2 caramel 코어 변경

#### 3.2.1 `Instance : public Shape` (P0)

```cpp
// bsdf = 이 Instance(=템플릿의 한 shape)의 머티리얼. base Shape(bsdf, nullptr) 로 전달.
//   arealight 는 항상 nullptr — pbrt 도 instancing+arealight 금지(scene.cpp 경고).
class Instance final : public Shape {
public:
    Instance(const Shape *geometry, const Matrix44f &to_world, BSDF *bsdf);

    std::pair<bool, RayIntersectInfo> ray_intersect(const Ray &ray, Float maxt) const override;
    AABB get_aabb() const override;            // m_world_aabb (생성 시 to_world·geometry AABB)
    Float get_area() const override;           // 미지원 (instance ≠ light)
    std::tuple<Vector3f, Vector3f, Float> sample_point(Sampler &) const override;   // 미지원
    Float pdf_solidangle(const Vector3f&, const Vector3f&, const Vector3f&) const override; // 미지원
    bool is_solid_angle_sampling_possible() const override { return false; }
    const std::vector<Vector3f>& get_polygon_vertices() const override;             // 미지원

private:
    const Shape *m_geometry;                   // 공유 템플릿 geometry, LOCAL space (intersection 전용)
    Matrix44f m_to_world, m_to_local;          // to_local = Inverse(to_world), 생성 시 1회
    AABB m_world_aabb;
};
```

`ray_intersect` — ray 를 local 로 변환, 공유 geometry 재귀, hit 을 world 로 역변환:

```cpp
const Vector3f d_local = transform_vector(ray.m_d, m_to_local);   // 비정규(스케일 반영)
const Float    k       = d_local.length();                        // |M_inv·d|  (Vector3f::length)
const Ray local{transform_point(ray.m_o, m_to_local), d_local};   // Ray ctor 가 dir 재정규화
auto [hit, info] = m_geometry->ray_intersect(local, maxt * k);    // maxt 를 local 단위로
if (!hit) return {false, info};
info.p        = transform_point(info.p, m_to_world);
info.sh_coord = Coordinate{transform_vector(info.sh_coord.m_world_n, T(m_to_local)).normalize()};
info.t       /= k;                                                // local t → world t
return {true, info};
// info.shape 는 상위 TLAS(BVHSceneTraits::ray_intersect, bvh_scene.cpp:36)가 이 Instance* 로
//   덮어씀 → BSDF/arealight 는 Instance 가 보유해야 함 (템플릿 것은 안 읽힘). gotcha 3.
```

**watertight gotcha 3개:**
1. `Ray` 생성자가 dir 강제 정규화 (`ray.h:32` `m_d{d.normalize()}`). → scale 있으면 local `t`/`maxt` 불일치. **`k = |M_inv·d|` 로 스케일** (위 코드). Ray 수정 불필요.
2. 템플릿은 **LOCAL space 로딩 필수.** 현재 `OBJMesh/PLYMesh(path, bsdf, arealight, transform)` 는 transform 을 정점에 bake (`objmesh.cpp:79,95`, `plymesh.cpp:77,100`). instancing 은 각 템플릿 shape 를 **identity 로 1회 로드**하고 `Instance` 가 placement transform 보유.
3. **`info.shape` 는 TLAS 가 top-level `Shape*`(=Instance)로 덮어씀** (`bvh_scene.cpp:36` `info.shape = s`). integrator 는 `info.shape->get_bsdf()`/`get_arealight()` 를 읽음 (`path.cpp:67-68,71`). → **Instance 가 bsdf 보유 필수** (템플릿 bsdf 안 읽힘). ctor 가 base `Shape(bsdf, nullptr)` 호출. 빠뜨리면 `get_bsdf()==nullptr` → null deref.

노멀 변환 perf: `transform.h::transform_normal` 은 `Inverse(T(mat))` 를 **매 호출 재계산** → 31M 인스턴스에서 per-ray inverse = perf 폭탄. 대신 이미 가진 `m_to_local` 의 transpose `T(m_to_local)` (= inverse-transpose, transpose 만, inverse 없음)를 vector 변환에 사용 (위 코드). `transform_point/vector` 는 `transform.h` 에 존재.

**메모리:** `Instance` 1개 ≈ 184B (vptr8 + bsdf8 + arealight8 + geom8 + to_world64 + to_local64 + aabb24; `Float=float`→Matrix44f=64B `common.h:32`). + TLAS `LinearBVHNode` 배열 + 재정렬 `const Shape*` 배열(`bvh_base.h`).
**Instance 객체 수 = Σ(ObjectInstance × 템플릿 내 shape 수) ≥ 30.9M** — 템플릿이 multi-shape 면 30.9M 초과 (pbrt 템플릿 = (shape,material) 리스트). 30.9M 기준 레코드만 ~5.7GB. 템플릿당 shape 수 실측 필수.

설계 메모: `Scene::m_meshes`(`vector<const Shape*>`) 에 `Instance*` 그대로 push. 템플릿 geometry 는 별도 풀(`vector<unique_ptr<Shape>>`)이 소유, `Instance` 는 비소유 포인터. `d_local` 은 ray 당 1회만 계산(위 코드 재사용).

#### 3.2.2 Curve (P1)

두 길:

- **(권장, M3 초기) converter tessellation**: curve → 카메라/노멀 무관 flat ribbon (segment 당 quad 2 tri), width0/width1 taper. caramel 코어 무변경. 단점: 5.4M curve → tri 폭증(메모리). 잔디/잎엔 충분.
- **(고충실/저메모리) 네이티브 `Curve : public Shape`**: pbrt 식 B-spline/ribbon ray-curve intersection. 메모리 절감, 구현 큼. instancing 과 결합 시(잔디 클럼프가 인스턴스면) 메모리 크게 절감.

→ island 의 curve 5.4M 중 상당수가 인스턴스 템플릿 내부일 가능성 큼. 그렇다면 **네이티브 Curve + instancing** 조합이 메모리상 유리. 구현 시 인스턴스/curve 분포 먼저 측정해 결정. 문서 단계에선 둘 다 열어둠.

#### 3.2.3 좌표계 handedness (P0, watertight, **미확정**)

pbrt = **왼손** 확정 (`pbrt-v4 util/transform.h:382` 주석 "left-handed", `LookAt` basis `right=cross(up,dir)`).
caramel = ? — 카메라 basis(`camera.cpp`: `left=cross(up,dir)`, 컬럼 `[left,up,dir,pos]`)가 **pbrt 와 동일 구성**이고 rotate 행렬도 표준. 즉 "caramel = 오른손" 단정은 **소스로 확정 안 됨** → flip 필요 여부 불명.

→ **경험적 확인 (M0 최우선)**: 좌우/전후 비대칭 단일 에셋을, 변환 없이(`C=I`) vs `C=diag(1,1,-1)` 적용해 렌더 → pbrt 레퍼런스와 비교해 결정. 가능한 결론:
- flip 불필요(두 렌더러 basis 동일) → `C=I`, winding 그대로.
- flip 필요 → `C` 를 **카메라 + 모든 transform 에 일관 적용**. `det(C)<0` 이면 triangle **winding 역순/노멀 반전**으로 facing 정합 (caramel watertight intersection + shading normal).

> 잘못 잡으면 좌우/전후 반전 또는 inside-out(노멀 뒤집힘) 렌더.

### 3.3 converter (pbrt2caramel)

#### 3.3.1 파서
- 레퍼런스 문법 따르되 경량. 필요 directive: `Camera/Film/Sampler/Integrator`, transform 계열, `WorldBegin`, `AttributeBegin/End`, `Shape("trianglemesh"/"plymesh"/"curve")`, `Material/MakeNamedMaterial/NamedMaterial`, `LightSource("infinite")/AreaLightSource("diffuse")`, `ObjectBegin/End/Instance`, `Include/Import`, `ReverseOrientation`, `Texture`(파싱은 하되 ptex 값은 버림).
- `Include`(블로킹) + `Import`(지연/병렬) 모두 파일 트리 따라 처리.

#### 3.3.2 중간표현(IR)
- `Template{ name, shapes[] }`, `InstanceRef{ template_name, to_world }`, `MeshRef{ ply_path, to_world, material }`, `CurveSet{...}`, `MaterialDef`, `LightDef`, `CameraDef`.
- 메모리: 31M instance ref 를 IR 로 들고 있어야 함 → IR 도 경량(ref = template_id + 4x4) 유지. 스트리밍/청크 emit 고려.

#### 3.3.3 emit
- `scene.json` + 신규 노드(instance/curve, §3.4). geometry 는 원본 `.ply` 경로 **참조** (복사 안 함; 22GB 중복 회피). cwd 기준 상대경로.
- env: equiarea PNG → **equirect EXR 변환** (caramel `image_env` 는 구면/equirect EXR 기대). 별도 변환 스텝.
- scene split: 단일 거대 JSON 회피 — caramel JSON 이 `Include` 류 지원 안 하면, asset group 별 분할 로딩을 caramel 파서에 추가하거나 단일 JSON 스트리밍 파싱.

#### 3.3.4 material 매핑 (ptex 제거 → flat color)

| pbrt | 수 | caramel | 비고 |
|---|---|---|---|
| `diffuse` (reflectance=ptex) | 57 | `diffuse` (albedo = 상수 fallback) | ptex 평균색 미산출 → 기본 상수(예: 0.5 회색) or material 별 하드코딩. M4 에서 ptex 평균 베이크로 개선 |
| `coateddiffuse` | 16 | `diffuse` 근사 (coat drop) | 또는 microfacet. 코트 반사 손실 |
| `diffusetransmission` | 22 | `twosided{diffuse}` 근사 | 잎의 투과 손실. 양면 diffuse 로 대체 |
| `dielectric` | 1 | `dielectric` (eta → in_ior/ex_ior) | |
| `interface` | 1 | (스킵 / no-op) | 매질 경계, caramel 매질 없음 |

> ptex 제거 = 색/디테일 대량 손실. flat fallback 은 형상 검증용. 컬러 정합은 M4(ptex 평균색 or 베이크) 과제.

#### 3.3.5 light 매핑
- `AreaLightSource "diffuse" L` ×22 → emitter quad shape 에 caramel `arealight{radiance=L}` 부착.
- `LightSource "infinite"` (equiarea PNG) → `image_env{path=변환된 .exr, scale, to_world}`. equiarea→equirect 변환 필수. caramel 은 env 1개만 허용(island 도 1개 → OK).

#### 3.3.6 camera 매핑 (watertight 주의)
- pbrt `fov` = **짧은 축** 기준. island 1920×804 → 짧은 축=세로(804) → fov 69.5 = 세로 fov. caramel `fov` = **가로 fov** (`camera.h`). → **aspect 비로 변환** (`fov_h = 2·atan(tan(fov_v/2)·(1920/804))`). 미변환 시 화각 틀림.
- `lensradius 0.003125` → `thinlens.lens_radius`; `focaldistance 1675.34` → `focal_dist`.
- `LookAt`/CTM → `pos/dir/up` (handedness `C` 적용, §3.2.3).
- film 1920×804 → `width/height`. `maxcomponentvalue 10` → caramel clamp 없음 (무시 or 옵션 추가).
- integrator volpath maxdepth10 → `path{depth_max:10}`. sampler zsobol1024 → `spp:1024` (uniform).

### 3.4 JSON schema (구현됨 — 2026-06-22)

전략 B (parse-time expansion): 런타임 클래스 없음, 엔진 무수정. `shape` 리스트 안 `type:"instance"` 엔트리 하나가 template(단일 mesh 또는 multi-mesh) + placements 를 인라인으로 가짐. 별도 top-level registry/`group` 참조 없음. 2형식:

```jsonc
// (a) 단일 mesh — bsdf 1개(entry-level) 공유, placement 은 transform only
{ "type":"instance", "path":"x.obj",
  "bsdf": { "type":"diffuse", "albedo":[0.2,0.3,0.5] },   // 또는 "id" 문자열
  "instances": [ { "to_world":[ ... ] }, ... ] }

// (b) multi-mesh template — "shapes" 가 template(각 sub-shape 자기 bsdf), entry bsdf 없음
{ "type":"instance",
  "shapes": [
    { "type":"ply", "path":"trunk.ply", "bsdf":"bark" },
    { "type":"ply", "path":"leaf.ply",  "bsdf":{ "type":"diffuse","albedo":[0.1,0.4,0.1] } } ],
  "instances": [ { "to_world":[ ... ] }, ... ] }
```

규칙 (parser enforce): `path`/`shapes` **정확히 1개**; `path`형은 entry `bsdf` 필수; `shapes`형은 entry `bsdf` 금지 + sub-shape `arealight` 금지(인스턴스 emit 불가); 어느 형식이든 per-placement `bsdf` 금지. sub-shape 는 기존 `parse_shape` 문법(obj/ply/triangle, bsdf inline-or-id, optional group-local to_world). `to_world` = 16-float 또는 transform 객체 배열.

파서 (`scene_parser.cpp::parse_instanced_shapes`): template(`path` 1개 또는 `shapes` 리스트)의 geometry+bsdf 를 `parse_shape` 로 **1회 로드**(group-local). placement마다 sub-shape마다 `Instance(geometry, to_world, sub_bsdf)` emit → 각 Instance single-material → TLAS `info.shape` clobber 가 그대로 정답(엔진 무수정).

**비용**: TLAS leaf = Σ(sub-shape × placement). shaderballs/소규모 OK. island hero-plant(ironwood 119,967 sub-shape × 30 = 3.6M Instance)엔 부적합 → 그땐 전략 A(런타임 `ShapeGroup` + inner BVH, placement당 1 leaf). **스키마 A 와 wire-compatible** — scene 파일 안 바꾸고 파서만 교체. (curve/native `Curve`, ptex 는 별도 M3/M4.)

---

## 4. 단계별 구현 계획 (milestones)

| 단계 | 내용 | caramel 코어 | 산출물 |
|---|---|---|---|
| **M0** | converter 스켈레톤: pbrt 경량 파서 + 단일 trianglemesh/plymesh → scene.json. handedness 변환. | 무변경 | 작은 단일 메쉬 렌더 일치 |
| **M1** | **instancing**: `Instance:Shape` + JSON `templates`/`instances` + 템플릿 LOCAL 로드. 작은 asset(예: isPalmDead 48MB) 인스턴스 렌더. | `Instance` 추가 | 인스턴스 씬 렌더 |
| **M2** | **full island geometry** (ptex 제거, flat material, curve 제외): 19 asset group, 31M instance, material 매핑. 메모리/시간 측정. | (M1) | 섬 형상 렌더 (무텍스처) |
| **M3** | **curve**: tessellate or 네이티브 `Curve`. 잔디/잎. | `Curve`(택1) | 식생 포함 |
| **M4 (옵션)** | ptex 평균색/베이크, equiarea env 정밀화, firefly clamp, 컬러 정합. | film clamp 등 | 컬러 근접 |
| **M5 (옵션)** | volumetric, sampler 고도화 등 풀 충실도. | 대공사 | 레퍼런스 근접 |

검증: 각 단계 caramel 디버그 integrator(normal/depth) + EXR diff 로 형상/카메라 일치 확인 후 진행.

---

## 5. 리스크 / open questions

1. **메모리**: 31M instance(~5.7GB) + TLAS + 22GB ply(부분 로드?). 전체 동시 로드 가능한지 측정 필요. ply 지연/스트리밍 로드 검토.
2. **TLAS 빌드 시간**: 31M prim BVH 빌드. 현재 `BVHScene::build` 성능 확인 필요 (최근 BVH flatten 있음).
3. **handedness 정확 축**: pbrt 카메라 viewing axis 부호 소스 재확인 (§3.2.3 검증 항목).
4. **camera fov 축**: pbrt 짧은축 fov ↔ caramel 가로 fov 변환식 검증.
5. **curve 분포**: 5.4M curve 가 인스턴스 내부인지 측정 → tessellate vs 네이티브 결정.
6. **scene split**: 단일 JSON 31M instance 파싱 메모리/속도. caramel JSON 분할 로딩 필요 여부.
7. **ptex 평균색**: flat fallback 색 출처 (기본 상수 vs ptex 평균 베이크).
8. **equiarea env**: PNG equiarea → EXR equirect 변환 정확도. (caramel env = equirect lat-long, `imageEnvLight.cpp:124`.)
9. **material 근사 손실**: coateddiffuse/diffusetransmission 근사로 룩 차이 — 수용 범위 합의 필요.
10. **Instance 객체 수 폭증**: multi-shape 템플릿 → ObjectInstance 30.9M 가 그 이상의 `Instance` 로 확장 (§3.2.1). 템플릿당 shape 수 먼저 측정.

---

## 6. 참고 file:line

**caramel**
- `include/shape.h:46` `Shape` 추상 베이스 / `:88` Triangle / `:127` TriangleMesh / `:140` OBJMesh / `:185` PLYMesh
- `include/scene_accel.h:42` `SceneAccel`(TLAS), `:44` `build(vector<const Shape*>)`
- `include/scene.h:51` `add_mesh_and_arealight`, `:64` `m_meshes`, `:69` `m_accel`
- `include/mesh_accel.h` `MeshAccel`(BLAS)
- `include/ray.h:32` Ray dir 강제 정규화
- `include/transform.h:33-43` transform_point/vector/normal
- `include/rayintersectinfo.h:34` RayIntersectInfo (p, sh_coord, t, shape, tex_uv, tri_index)
- `src/scene_parser.cpp:101-142` camera, `:176-217` shape, `:220-243` light, `:246-301` bsdf, `:303-310` texture, `:381-423` transform

**pbrt-v4**
- `src/pbrt/parser.cpp:272-342` tokenizer, `:601-987` directive dispatch
- `src/pbrt/scene.h:159-203` Instance 데이터모델, `scene.cpp:309-395` ObjectBegin/Instance
- `src/pbrt/shapes.cpp:1388-1492` shape create
- `src/pbrt/materials.cpp:633-691` material create
- `src/pbrt/lights.cpp:1507-1691` light create
- `src/pbrt/cameras.cpp:242-266` camera create

**island**
- entry `island.pbrt`, `materials.pbrt`
- 29GB / 507 .pbrt / 14,222 .ply / 18,400 ptex ref / 30.9M ObjectInstance(313 def) / 5.4M curve

---

## 7. 리뷰 반영 (2026-06-17, fresh-agent 적대적 검증)

소스 대조로 초안 결함 수정:

- **`Instance` 코드 컴파일 불가 2건 수정**: caramel 에 `Vector3f::norm()` 없음 → `.length()`. `Coordinate::n()` 없음 → 공개 멤버 `m_world_n` (`coordinate.h:66`). `Coordinate(const Vector3f&)` explicit ctor 는 존재.
- **`info.shape` 클로버 (핵심)**: TLAS `BVHSceneTraits::ray_intersect` 가 `info.shape = s`(=Instance) 로 덮어쓰고(`bvh_scene.cpp:36`), integrator 가 거기서 BSDF 를 읽음(`path.cpp:67-68`). → "BSDF 는 템플릿에 있어 OK" 는 **틀림**. Instance 가 bsdf 보유하도록 ctor 수정.
- **multi-shape 템플릿**: pbrt 템플릿 = (shape,material) 리스트(`pbrt scene.cpp:299-303`). 단일 Instance 는 단일 bsdf 만 → **template-shape 당 Instance 1개** 로 확장하도록 §3.4 정정. 인스턴스 수 ≥ 30.9M.
- **handedness 다운그레이드**: caramel 카메라 basis 가 pbrt 와 동일 구성 → "caramel=오른손" 단정 철회, **경험적 확인**으로 변경 (§3.2.3).
- **perf**: per-ray `Inverse()` 회피 — 노멀은 `T(m_to_local)` 로 (§3.2.1).

검증 OK 항목: k-스케일 수학, 노멀 inverse-transpose 방향, TLAS 다형 dispatch(accel 무변경), fov 축 규약(양쪽), pbrt 머티리얼-인-템플릿, 메모리 산식(`Float=float`), mesh 로더 transform bake, env equirect 기대.
