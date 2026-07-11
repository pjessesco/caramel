# Matt Pharr 블로그 → caramel 심층 적용 조사 (Deep Dive)

> **출처**: https://pharr.org/matt/blog/  
> **목적**: 조사(investigation)만 수행하며, caramel 소스 코드는 변경하지 않는다.  
> **검증**: 아래 각 항목(pick)은 실제 caramel 소스 코드에 대조하여 검증되었다.  
> **섹션 수**: 총 10개 심층 섹션 — Top Picks 5개 (T2–T5, T7), Also Consider 5개 (A1–A5). 추가로 Not Applicable 2개.
> **주의**: 테스트 인프라 관련 pick(구 T1 end-to-end 테스트, 구 T6 unit 테스트)은 사용자 요청으로 이 보고서에서 제외했다(테스트는 별도 수정 예정). 섹션 ID(T2–T5, T7)는 원본 유지 — deep-dive/summary 상호참조 정합용.

## 목차

**Top Picks**

- [T2. 삼각형 교차 커널·외적에 Kahan difference-of-products (FMA)](#t2-삼각형-교차-커널외적에-kahan-difference-of-products-fma)
- [T3. CHECK_RARE 통계 분기 카운터 — 실제 잠복 버그에 연결](#t3-check_rare-통계-분기-카운터--실제-잠복-버그에-연결)
- [T4. ImageEnvLight 등장방형 importance sampler 검증 (// ??? 해소)](#t4-imageenvlight-등장방형-importance-sampler-검증---해소)
- [T5. 스레딩 독립 결정성 (per-pixel-sample 해시 시드)](#t5-스레딩-독립-결정성-per-pixel-sample-해시-시드)
- [T7. 장기: Wavefront/SoA path tracer → SIMD/GPU, 그리고 Amdahl startup 조각](#t7-장기-wavefrontsoa-path-tracer--simdgpu-그리고-amdahl-startup-조각)

**Also Consider**

- [A1. 카메라 공간(ish) 렌더링 — geometry를 평행이동해 카메라를 원점에](#a1-카메라-공간ish-렌더링--geometry를-평행이동해-카메라를-원점에)
- [A2. 바이너리 지오메트리(PLY) 선호 + OBJ weld를 unordered_map + 측정 후 최적화](#a2-바이너리-지오메트리ply-선호--obj-weld를-unordered_map--측정-후-최적화)
- [A3. 씬 전처리 가속: BVH arena 할당자 + 메시별 병렬 빌드 + content-hash dedup](#a3-씬-전처리-가속-bvh-arena-할당자--메시별-병렬-빌드--content-hash-dedup)
- [A4. sampler \[0, 1-2^-24\] 불변식 문서화·assert](#a4-sampler-0-1-2-24-불변식-문서화assert)
- [A5. Basu-Owen 측도보존 삼각형 샘플링 (QMC sampler 추가 후에만)](#a5-basu-owen-측도보존-삼각형-샘플링-qmc-sampler-추가-후에만)

## 이전 보고서 대비 정정 사항

아래는 심층 조사 과정에서 실제 caramel 소스에 대조해 확인한, 이전(shallow) 보고서 대비 정정 사항이다. 각 항목 앞의 대괄호 태그는 관련 섹션을 가리킨다.

- [T2] U/V/W 계산의 실제 위치는 src/shapes/shape.cpp:114-116 이다 (113번 줄은 주석 // Calculate scaled barycentric coordinates). 이전 패스는 113-115로 적었다 — 한 줄 밀림.
- [T2] Double 재계산 fallback의 if 블록은 shape.cpp:119-129 이다 (118은 주석, 130은 빈 줄). 이전 패스는 118-130으로 적었다.
- [T2] 이전 패스가 'det=U+V+W is a SumOfProducts candidate'라 하고 shape.cpp:118로 지목한 것은 부정확하다. det은 shape.cpp:138의 3-항 덧셈(U+V+W)이며 이 블로그가 다루는 '두 곱의 차(a*b-c*d)'가 아니다. 실제 DOP 대상은 U/V/W 각각(2x2 행렬식)이고, T=U*az+V*bz+W*cz(shape.cpp:147)도 3-항 합이라 대상이 아니다.
- [T2] task/이전 패스가 ext/peanut을 'EMPTY/unreadable'로 전제했으나, 이 워킹트리에서는 실제로 채워져 있어 Vector3f::cross를 직접 읽을 수 있다. 구현은 ext/peanut/include/Peanut/impl/matrix.h:394-399의 per-component naive a*b-c*d이며, 그 자체로 DOP 직접 적용 대상임을 소스에서 확인했다.
- [T2] DOP 공식은 cd=c*d를 한 번만 계산해 두 fma에서 동일한 값을 재사용해야 한다: dop=fma(a,b,-cd), err=fma(-c,d,cd), return dop+err. 이전 표기 'fma(a,b,-c*d)+fma(-c,d,c*d)'처럼 -c*d를 fma 안에서 다시 곱으로 쓰면 오차 보정(err)이 성립하지 않으므로 주의 — 미세하지만 정확성의 핵심.
- [T3] refract() 위치: 이전 패스의 'bsdf.cpp:42-49'는 부정확. 실제 함수는 src/bsdfs/bsdf.cpp:42-50 이고, TIR 가드가 없는 핵심 라인은 cos_t = sqrt(1 - (sin_t * sin_t)) 인 line 47, 독립적인 sin_t 재계산은 line 46 이다.
- [T3] NaN 도달성 과장: 이전 패스는 '임계각 근처에서 silent NaN/firefly'라고 했지만, 정통 TIR 케이스는 실제로는 도달 불가. fresnel_dielectric()가 sin_t>=1 일 때 정확히 Float1 을 반환하고(bsdf.cpp:70), sample_1d()가 [0,1) 이므로(uniformstd.cpp:59) sample_1d() <= 1.0 이 항상 참 → 항상 reflect 분기를 타서 refract()가 호출되지 않는다. NaN 은 (a) fresnel 과 refract 의 두 개의 별도 float 식이 TIR 경계에서 반올림으로 어긋나는 경우(블로그가 문서화한 바로 그 버그), 또는 (b) local_incoming_dir 가 완벽히 정규화되지 않아 dir[2]^2>1 이라 fresnel 은 cos_i 를 clamp 하지만(bsdf.cpp:61-64) refract 는 clamp 하지 않아 sin_i=sqrt(negative)=NaN 이 되는 경우에만 발생. 즉 measure-zero/round-off 이벤트이지 일상적 near-critical NaN 이 아니며, 그래서 hard assert 가 아니라 통계적 CHECK_RARE 가 정확히 맞는 도구다.
- [T3] envmap 극점: 이전 패스는 imageEnvLight.cpp:174 의 sin(uv[1]*PI) 분모를 미가드 division-by-zero 처럼 기술. 실제로는 line 165-167 에서 uv[1]이 0/1 의 1e-6 이내면 0 을 반환하는 가드가 이미 있음. 따라서 CHECK_RARE 의 역할은 버그 수정이 아니라 '가드 발화 빈도 모니터링'(1e-6 임계값이 적정한지 검증).
- [T3] AABB near-zero direction: 이전 패스가 rare 분기로 나열했으나 aabb.cpp:84 의 ray.m_d_near_zero[i] 검사는 축당·광선당·박스당 실행되며 축정렬 광선에서 일상적으로 발화 → rare 이벤트가 아니라 결정론적 정확성 가드. CHECK_RARE 대상으로 부적합. 더 나은 rare 후보는 area.cpp 의 isnan(dir) 가드(line 76-78), 광원 메시 재교차 실패 !hit(line 84-86), 퇴화 근접 light_to_hitpos.dot < EPSILON*EPSILON(line 96-98).
- [T3] AreaLight 'solid-angle->uniform-area fallback'(area.cpp:110 else): 이 분기는 static TRY_SOLID_ANGLE_SAMPLING(light.h:118) 과 shape 능력으로 결정되어 shape 당 한 번 정해지는 것이지 rare fp 이벤트가 아님. rare 이벤트는 solid-angle 경로 내부의 조기 return 가드들이다.
- [T3] 쓰레딩 메커니즘: 이전 패스의 'thread_local fits parallel_for.h'는 방향은 맞으나 핵심 제약 누락. caramel ThreadPool(parallel_for.h)은 parallel_for 호출마다 std::thread 를 생성·join 하는 비영속 풀이고 pbrt 처럼 '각 쓰레드에서 콜백 실행' 메커니즘이 없다. 따라서 worker thread_local 카운터는 join 시 파괴되므로, pbrt 의 StatRegisterer barrier 방식을 그대로 못 쓰고 self-registering RareCounter(thread_local 소멸자가 전역 mutex-guarded accumulator 로 병합) 패턴이 필요하다. render()는 parallel_for 를 정확히 한 번 호출하므로(MCIntegrator.cpp:53) 그 직후 report 호출이 적절.
- [T4] 환경 전제 정정: ext/ 서브모듈은 더 이상 비어 있지 않다. git submodule status 상 peanut/glfw/imgui/ImGuiFileDialog/caramel-scenes/hypothesis 전부 체크아웃되어 있고 ext/hypothesis/hypothesis.h(+cephes.h)가 실제로 존재한다. 따라서 이전 pass의 'ext/hypothesis, EMPTY submodule' 및 과제 전제는 stale하며, chi-square 테스트를 실제로 빌드/검증할 수 있다.
- [T4] 블로그 내용 정정: 이 Pharr(2019) 글에는 pdf 유도식·수식·octahedral/equal-area/pole-singularity 논의가 없다. 글은 두 warping(파브르트 Distribution2D 2-step vs Clarberg 'Wavelet Importance Sampling'식 MIP-map 계층 warp f(x)=a(x-b))의 '시각화 비교'이고 결론은 '오차 비슷, 승자 없음'이다. // ??? 를 pin down하는 유도는 이 글이 아니라 PBR book/InfiniteAreaLight에 있다. octahedral equal-area는 이 글과 무관한 별개 아이디어(pbrt-v4)다.
- [T4] 라인번호 정정: 빌드시 sin(theta) 가중은 src/image.cpp:158이 아니라 :166에서 곱해진다(158은 get_data_for_sampling 함수 시작). true 인자 호출부는 imageEnvLight.cpp:48(build_sampling_distrib)와 :101(power).
- [T4] 이전 pass가 놓친 점: Distrib2D::sample는 정수 픽셀 인덱스만 반환(distribution.h:99-107)하고 sample_direct_contribution은 (idx+0.5)/size로 픽셀 '중심'만 사용(imageEnvLight.cpp:132)하며 남은 소수 난수를 버린다 -> 실질적으로 W*H개 이산 방향만 샘플. 이는 pdf_solidangle이 주장하는 piecewise-constant 연속 밀도와 불일치하는 실제 편향/밴딩 원인으로, // ??? 유도 자체보다 더 구체적인 개선 지점이다.
- [T4] 이전 pass가 놓친 점: 빌드 가중이 sin(h/H*PI)로 픽셀 top edge를 써서 h=0 최상단 스캔라인 가중이 정확히 0이 된다(pbrt는 (v+0.5)/H). pdf_solidangle도 그 행에서 0을 반환하므로 편향은 아니고 자기일관적이지만, 최상단 행이 light sampling에서 완전히 제외되는 효율 손실이자 midpoint 규약 위반이다.
- [T4] Rec.709 튜닝 뉘앙스 정정: env-'light' 샘플링의 이상적 타깃은 지각 luminance가 아니라 방사 에너지이므로 (r+g+b)/3(합에 비례)이 오히려 물리적으로 타당하다. Rec.709로 바꾸는 것은 지각적 튜닝일 뿐 분산을 개선한다는 보장이 없고 경우에 따라 악화될 수 있다.
- [T5] 사전 조사의 주의사항 "재시드 -> gt.exr 재베이스라인 필요"는 부정확하다. 실제 render 테스트는 avg 휘도 비율을 tolerance로 비교(simple_render_test.cpp:53-54, utils.h:44-45의 TEST_BODY)하므로 per-pixel 노이즈 재배치에 통계적으로 둔감하며, 재베이스라인 없이 그대로 통과할 것이다. 새로 bit-exact 테스트를 추가하는 경우에만 baseline이 필요하다.
- [T5] 사전 조사는 결정성 파괴 트리거를 '향후 tile/row 재병렬화'로만 한정했으나, 실제 취약성은 더 넓다. 한 열 전체(모든 row x 모든 spp)가 sampler 인스턴스 하나를 공유하므로, 스레딩을 전혀 바꾸지 않아도 integrator/BSDF의 sample_1d() 소비 개수나 순서를 바꾸는 어떤 수정(새 BSDF lobe 추가, RR 조정, draw 재배열)이든 그 열의 이후 모든 픽셀 노이즈를 이동시킨다. 이 소비-cascade 취약성이 재병렬화 위험보다 더 자주 발현된다.
- [T5] 메커니즘 정정: caramel의 현재 seed 인자는 PCG의 initstate(m_state 시작 오프셋)에 대응하고 sequence(m_inc)는 stream 기본값 1로 고정되어 있다 -- 즉 모든 열이 '동일 sequence, 다른 시작 오프셋'이다. 블로그 기법은 반대로 Hash로 sequence(m_inc)를 픽셀마다 바꾼다. 따라서 SetSequence는 새로 구현할 필요 없이 기존 생성자 본문(이미 pcg32_srandom_r 패턴)을 set_sequence()로 추출해 재사용하면 된다. 사전 조사의 'SetSequence(Hash(...))' 표기는 방향은 맞지만 이 재사용 가능성을 드러내지 않았다.
- [T7] prior 패스는 path.cpp의 mis_sampling_path를 'depth-first recursion'이라 했으나, 실제로는 재귀가 아니라 bounded iterative loop다 (path.cpp:52 `for(Index depth=1;depth<=m_max_depth;depth++)`). 'sample 하나를 끝까지 추적하는 depth-first' 성격은 맞지만 'recursion'은 부정확. 실제 재귀는 BVH build/flatten(bvh_base.cpp:144-145, 170-172)에 있다.
- [T7] prior의 '인덱스드 AoS 메시 레이아웃' 표현은 정밀화 필요: 속성(position/normal/texcoord)은 이미 별도 std::vector로 분리 저장돼 있어 attribute-level로는 SoA다(shape.h:158-160). SIMD에서 문제되는 것은 (a) 각 Vector3f가 xyz-packed 스칼라라 lane 병렬 x[]/y[]/z[]가 없다는 점과 (b) 삼각형 테스트마다 m_face_indices(Vector3i)를 통한 간접 gather(triangle_mesh.cpp:238-241)라는 점이다.
- [T7] 'perfNotes'는 작업 트리에 존재하는 파일이 아니다(find/grep 결과 없음). no-SIMD/no-GPU 주장 자체는 코드+빌드로 직접 검증됨: 단일 Ray 처리(ray.h:30-40), 단일 레이 BVH traversal(bvh_base.cpp:189-241), CMakeLists.txt에 -march/AVX/CUDA/ISPC/OpenMP/fast-math 플래그 전무.
- [T7] envmap 'get_data_for_sampling 두 번 빌드'는 확인됨. 정확한 위치는 imageEnvLight.cpp:48(build_sampling_distrib, 생성자 경로)과 imageEnvLight.cpp:101(power()). 둘 다 인자 true로 동일 데이터를 재생성하며 power()는 scene.cpp:113 build_light_pdf에서 호출된다.
- [A1] 명시적 카메라 위치는 prior가 지목한 m_cam_to_world 가 아니라 Vector3f m_pos (include/camera.h:65) 이다. m_cam_to_world (camera.h:72) 는 4x4 행렬이고, m_pos 는 src/cameras/camera.cpp:51 에서 이미 블로그의 step-1 공식 그대로 `m_pos = Block<0,0,3,1>(m_cam_to_world * Vector4f{0,0,0,1})` 로 (0,0,0)을 world로 변환해 저장해 둔다. 즉 '카메라 월드 위치 추출'은 신규 코드 없이 이미 존재한다.
- [A1] '로드시 world-space 로 bake' 는 obj/ply/trianglemesh 로더에만 해당한다(src/shapes/inline_triangle_mesh.cpp:43, src/shapes/plymesh.cpp:67-71, src/shapes/objmesh.cpp:82-87). 반례가 둘 있다: (1) Instance(src/shapes/instance.cpp:98-120)는 의도적으로 bake 하지 않고 per-ray 로 world<->local 변환한다. (2) 단일 Triangle 은 파서에서 to_world 없이 raw p0/p1/p2 로 생성된다(src/scene_parser.cpp:248-263). 따라서 recenter 대상은 3개 mesh 로더뿐 아니라 Instance의 to_world 와 Triangle 정점도 포함해야 한다.
- [A1] prior의 'grazing-angle half 는 못 고친다'는 옳으며 코드로 확증된다: src/rayintersectinfo.cpp:37 에 `// TODO : add offset using geometry normal` 주석이 있고, 실제 offset 은 geometry normal 이 아니라 ray 방향으로 준다(rayintersectinfo.cpp:38 의 `p + world_d*EPSILON`, scene.cpp:84 의 `pos1 + dir*EPSILON`). recenter 는 좌표 '크기(magnitude)' 실패 모드만 없앨 뿐 '각도(angle)' 실패 모드는 그대로다.
- [A1] plymesh 정점 bake 라인은 :67 단일 라인이 아니라 transform_point 호출이 67-71 에 걸쳐 있다(미세 정정).
- [A1] prior 가 'half the EPSILON risk' 로 반반이라 표현했지만, 정량적으로 EPSILON=1e-3 가 위험해지는 임계는 |좌표|~1.6e4 (offset≈1 ULP)이고 1e6 에서 offset≈1/60 ULP 로 완전 소멸한다. caramel/CLO 씬 규모(O(1~10))에서는 8000 ULP 이상 여유라 현재 실질 위험은 0에 가깝다 — 즉 magnitude half 자체가 '현재는' 잠재적이다.
- [A2] 프레이밍 정정: OBJ float 파싱은 caramel 자체 코드가 아니라 vendored ext/tiny_obj_loader.h가 담당하며, strtod가 아닌 자체 고속 파서 tryParseDouble(ext/tiny_obj_loader.h:866, parseReal 경유 998)를 쓴다. 즉 블로그가 지목한 pbrt의 strtod 핫스팟은 caramel에 그대로는 존재하지 않는다(블로그 part 3에서 도달하는 '커스텀 파서' 최적화를 의존성이 이미 내장).
- [A2] 출처 정정: 블로그 part 1은 weld/hashmap이나 strtod 교체를 실제로 분석하지 않는다(둘 다 part 3로 미룸). part 1이 실측한 것은 PLY 바이너리 변환 이득(34m58s→27m35s, 1.3x; PLY 130MB/s vs pbrt text 16.5MB/s, 약 8x)과 perf 프로파일링 방법론이다. unordered_map 아이디어는 글 도입부의 Pixar 해시테이블 일화에서 착안된 것이지 part 1의 기술 본문이 아니다.
- [A2] 환경 전제 정정: 과제는 ext/ 서브모듈(peanut, caramel-scenes 등)이 비어있다고 했으나, 이 워킹트리에는 실제로 체크아웃돼 있다(caramel-scenes에 binary_little_endian PLY 441개 = lego 씬, ajax.obj 50MB, ext/peanut 헤더 24개). 이 자산들로 직접 검증함.
- [A2] prior-pass 코드 클레임은 모두 정확: std::map<tuple<int,int,int>,Int>는 objmesh.cpp:71 그대로, single-shape 거부는 line 58 체크/line 59 CRM_ERROR, happly PLY 경로는 plymesh.cpp:44. 라인 넘버 오류 없음.
- [A3] prior 주장 (2) 정정: per-mesh BVH는 Scene::build_accel이 아니라 TriangleMesh::finalize(triangle_mesh.cpp:70-71)에서 빌드되며, 이는 mesh 생성자를 통해 SceneParser::parse_shapes의 직렬 루프(scene_parser.cpp:150-158)에서 실행된다. Scene::build_accel(scene.cpp:100-103)은 전체 shape에 대한 단일 top-level scene BVH(BVHScene::build, bvh_scene.cpp:41-43) 하나만 빌드한다. 따라서 '메시 간 parallel_for'의 대상은 build_accel이 아니라 parse_shapes 루프이고, build_accel 자체는 트리 하나라 '메시별 병렬'이 성립하지 않는다.
- [A3] prior 주장 (3) 심화/정정: caramel은 로드 시 to_world를 vertex에 굽는다(objmesh.cpp:82-83, plymesh.cpp:67-74). 그래서 블로그식 raw-buffer content-hash는 '동일 파일 + 동일 transform'만 dedup한다. Moana의 진짜 이득(하나의 지오메트리를 서로 다른 transform으로 공유)을 얻으려면 로더가 transform을 굽지 않고 local-space로 유지한 뒤 기존 Instance 경로로 라우팅하는 auto-instancing이 필요하며, 이는 단순 '버퍼 해시'보다 훨씬 큰 변경이다.
- [A3] prior 주장 (1) 정밀화: arena는 반드시 per-thread여야 병렬 빌드에 도움이 된다. 단일 공유 arena는 자체 lock이 필요해 malloc-mutex 경합을 그대로 재현한다(블로그의 per-thread 1MB slab 결론과 일치). 또한 churn은 135-136의 두 vector에 국한되지 않고, 노드마다의 make_unique<BVHNode>(142-143)와 flatten_recursive(185) 후 통째로 폐기되는 pointer-tree 전체가 대상이다.
- [A3] prior 누락: parse_shapes 병렬화의 hard prerequisite로 Logger 스레드 안전성이 있다. Logger::print_*(logger.h:38,44,50)는 std::cout 체인 + std::localtime(57)을 mutex 없이 쓰므로 병렬 로드 시 로그가 섞이고 localtime은 UB다.
- [A4] include/distribution.h:73 지목은 부정확: Distrib1D::sample 함수는 73-76행이지만, 불변식이 load-bearing 하게 걸리는 실제 지점은 std::ranges::upper_bound 호출인 74행과 index를 반환하는 75행(return iter - m_cdf.begin();)이다.
- [A4] prior는 envmap을 'Distrib1D::sample ... feeding envmap'으로 뭉뚱그렸으나, envmap 경로는 Distrib2D::sample(distribution.h:99-107)를 거치며 별도의 out-of-bounds 지점이 존재한다: 1.0 샘플이 들어오면 m_width_distrib.sample가 행 개수 w를 반환하고 다음 줄 m_height_distrib_list[w](101행)에서 OOB가 난다. Distrib1D 단일 사이트가 아니라 최소 3개(scene.cpp:93 / imageEnvLight→Distrib2D:101 / triangle_mesh.cpp:211)의 독립 크래시 지점이다.
- [A4] prior의 'assert' 표현은 정제 필요: 순수 assert()는 NDEBUG(=Release/렌더 설정)에서 컴파일 아웃되므로 실제 렌더 구성에서 무력하다. 동등한 안전장치는 컴파일타임 static_assert + Distrib1D::sample의 clamp 조합이다.
- [A5] prior pass는 triangle.cpp:62-66의 sqrt-warp가 'AreaLight uniform-area fallback'을 먹인다고 했으나 부정확하다. Triangle의 세 생성자 모두 Shape(bsdf, nullptr)로 초기화되어(triangle.cpp:35-46) standalone Triangle은 절대 이미터가 되지 않는다. 게다가 Triangle::is_solid_angle_sampling_possible()==true(triangle.cpp:117-119)라 인스턴스화된 Triangle 라이트조차 Peters-2021 solid-angle 경로를 탄다. AreaLight fallback(area.cpp:111)에서 실제로 도달하는 sqrt-warp는 TriangleMesh::get_triangle_sample_point의 triangle_mesh.cpp:219-220 뿐이다.
- [A5] warp 산술식의 정확한 줄 번호: triangle_mesh.cpp는 219-220(샘플 draw는 216-217), triangle.cpp는 65-66(draw는 62-63). prior의 '62-66'/'216-220'은 블록 범위이며 실제 warp 연산 줄은 그보다 좁다. 216-220 범위는 정확.
- [A5] prior의 '한 번의 sample_1d()면 인터페이스 적합' 주장은 맞지만 불완전하다. sample_1d()는 24비트 정밀도만 반환하므로(uniformstd.cpp:59: next_uint32()>>8 * 2^-24) u*2^32로 정수 재구성 시 하위 8비트가 0이 되어 base-4 자리가 16개가 아닌 12개(4^12 sub-triangle)로 제한된다. 깔끔한 구현은 이미 존재하는 private next_uint32()(uniformstd.cpp:43)를 Sampler 인터페이스에 노출해야 한다.
- [A5] prior가 payoff 대상으로 든 'instanced emitters'는 독립 경로가 아니다. instance.cpp:135의 warp는 오직 area.cpp:111 fallback을 통해서만 도달하며, 그 fallback은 템플릿 mesh의 is_solid_angle_sampling_possible()에 게이팅된다(instance.cpp:151) — 직접 mesh 라이트와 동일한 조건이다.

# Top Picks

## T2. 삼각형 교차 커널·외적에 Kahan difference-of-products (FMA)
**노력**: 낮음~중간 · **효과**: 낮음~중간 (정확도/견고성) · **출처**: [Accurate Differences of Products with Kahan's Algorithm](https://pharr.org/matt/blog/2019/11/03/difference-of-floats.html)

### 현재 상태
기본 삼각형 교차 커널은 watertight 방식(`watertight_intersection`, `src/shapes/shape.cpp:79-164`)이고, `moller_trumbore`(`shape.cpp:46-76`)는 `USE_MOLLER_TRUMBORE` 매크로가 정의될 때만 쓰인다(`triangle.cpp:89`, `triangle_mesh.cpp:243`). watertight 커널의 핵심은 세 개의 2x2 행렬식(스케일된 barycentric)이다:

```cpp
// src/shapes/shape.cpp:113-116
// Calculate scaled barycentric coordinates
Float U = cx*by - cy*bx;
Float V = ax*cy - ay*cx;
Float W = bx*ay - by*ax;
```

이 세 값 중 하나라도 정확히 0이면 Double로 재계산한다(`shape.cpp:119-129`). 이는 JCGT watertight 논문(주석 `shape.cpp:78`)이 요구하는, 광선이 공유 에지를 정확히 통과할 때의 부호 tie-break용 경로다:

```cpp
// src/shapes/shape.cpp:118-129
if(U==Float0 || V==Float0 || W==Float0){
    const Double cxd = static_cast<Double>(cx); /* ...6개 double 캐스팅... */
    U = static_cast<Float>(cxd*byd - cyd*bxd);
    V = static_cast<Float>(axd*cyd - ayd*cxd);
    W = static_cast<Float>(bxd*ayd - byd*axd);
}
```

이어서 `det = U + V + W`(`shape.cpp:138`), `T = U*az + V*bz + W*cz`(`shape.cpp:147`)를 계산한다 — 둘 다 3-항 합이다.

외적 `Vector3f::cross`는 `ext/peanut/include/Peanut/impl/matrix.h:394-399`에서 성분별 naive `a*b-c*d`로 구현돼 있다(이 워킹트리에서 실제로 읽힘):

```cpp
static Matrix cross(const Matrix &m1, const Matrix &m2) requires (...) {
    return {m1[1]*m2[2] - m1[2]*m2[1],
            m1[2]*m2[0] - m1[0]*m2[2],
            m1[0]*m2[1] - m1[1]*m2[0]};
}
```

`cross`는 삼각형 법선/면적(`triangle.cpp:58,72,111`, `triangle_mesh.cpp:207,229,280` 등), 카메라/좌표계(`camera.cpp:60-61`, `coordinate.h:49,53`), moller의 `DE2/TE1`(`shape.cpp:53,58`)에서 광범위하게 쓰인다. 이미 `std::fma`는 `include/polygon_sampling.h`(`mix_fma` 94-97, Householder 158-166, slerp 264-274)에서 활발히 사용 중이고, peanut `common.h:57`도 `if constexpr (std::is_floating_point_v<T>)` 관용구를 쓴다. CMake에는 `-ffast-math`/`-ffp-contract`/`-march`/`-O2` 등 FP 플래그가 전혀 없다(`CMakeLists.txt`).

### 문제/기회
`a*b-c*d`는 두 곱의 크기가 크고 부호가 같아 값이 거의 상쇄될 때 catastrophic cancellation을 겪는다. 블로그 예시에서 float 결과 −128, 정답 −75.16으로 유효숫자가 전부 날아간다. watertight의 U/V/W와 cross 성분이 정확히 이 형태다. 특히 좌표 크기가 클수록(테스트에도 `far from origin` 1000+, `needle` 삼각형 케이스 존재) 오차가 커지고, 잘못된 부호는 에지에서 hit/miss를 뒤집어 crack이나 shadow acne를 유발할 수 있다. Double fallback(`shape.cpp:119-129`)이 존재한다는 사실 자체가 float 정밀도 부족을 인정하는 코드다. 단, 이 기법은 barycentric/법선 정확도를 높일 뿐 `EPSILON = 1e-3`(`common.h:55`, "TODO: lower epsilon / adaptive")를 직접 낮추지는 않는다 — EPSILON은 그림자/재귀 광선 오프셋용(`scene.cpp:80-88`, `rayintersectinfo.cpp:38`, `area.cpp:96`)이라 별개 이슈다.

### 블로그 기법
FMA(fused multiply-add)는 `a*b`를 반올림 없이 유지하고 마지막에 한 번만 반올림한다. 이를 이용해 `c*d`의 반올림 오차를 정확히 복원한다:

```cpp
inline float DifferenceOfProducts(float a, float b, float c, float d) {
    float cd  = c * d;
    float err = std::fma(-c, d, cd);   // c*d의 반올림 오차 (부호 반대)
    float dop = std::fma(a, b, -cd);   // a*b - cd (상쇄 없이)
    return dop + err;                  // 오차 보정
}
```

핵심은 `cd = c*d`를 **한 번만** 계산해 두 fma에서 동일 값으로 재사용하는 것이다. 정확도는 **≤1.5 ulp**(naive fma 대비, 이론 하한 0.5 ulp에 근접), 비용은 naive 대비 **1.09배**(Double 승격은 2.98배). cross 예시 (33962, 41563, 7706)×(−24871, −30438, −5643)에서 naive float `(1552,−1248,−128)` vs DOP `(1556.03,−1257.52,−75.17)` ≈ double 일치. 주의: `a*b`와 `c*d`의 부호가 반대면 상쇄가 없어 `err`가 무의미(그래도 정확). PBRT 차기판에 80곳 이상 적용됐다.

### caramel 적용 설계
**1) 공용 헬퍼 추가** — `include/common.h`에:

```cpp
inline Float difference_of_products(Float a, Float b, Float c, Float d) {
    using std::fma;
    const Float cd  = c * d;
    const Float err = fma(-c, d, cd);
    const Float dop = fma(a, b, -cd);
    return dop + err;
}
```

**2) watertight U/V/W 교체** (`shape.cpp:114-116`):

```cpp
Float U = difference_of_products(cx, by, cy, bx);
Float V = difference_of_products(ax, cy, ay, cx);
Float W = difference_of_products(bx, ay, by, ax);
```

DOP는 결정적(deterministic)이므로 공유 에지에서 동일 입력→동일 출력이 유지돼 watertightness가 보존된다. Double fallback(`119-129`)은 **일단 그대로 두는 것을 권장**한다(정확한 0 tie-break 보증). 다만 후속 실험으로 DOP가 float에서 부호를 회복해 fallback 진입 빈도를 줄이는지, 나아가 fallback을 DOP로 대체 가능한지 검증하는 옵션이 있다.

**3) cross에 DOP 적용** — 두 가지 안:
- (a) **caramel 측 헬퍼** `cross_dop(a,b)`를 별도 함수로 두고 법선/면적 계산부(`triangle*.cpp`)에서 호출. 서브모듈 미변경, 안전.
- (b) **peanut `cross` 직접 개선**(matrix.h:394-399) — `Vector3i` 등 정수 벡터가 `std::fma`로 double 경유되지 않도록 반드시 가드:

```cpp
if constexpr (std::is_floating_point_v<T>) {
    auto dop = [](T a,T b,T c,T d){ using std::fma; T cd=c*d; return fma(a,b,-cd)+fma(-c,d,cd); };
    return {dop(m1[1],m2[2],m1[2],m2[1]), dop(m1[2],m2[0],m1[0],m2[2]), dop(m1[0],m2[1],m1[1],m2[0])};
} else { /* 기존 naive */ }
```

(b)는 모든 cross 호출부에 일괄 적용돼 레버리지가 크지만 서브모듈 커밋이 필요하다. **적용 순서**: 헬퍼 추가 → watertight U/V/W → cross(안 (a) 또는 (b)) → (선택) moller의 `DE2/TE1`. `det`, `T`는 3-항 합이라 이 기법 대상이 아니다(별도 compensated-sum 기법 영역).

### 리스크·주의·트레이드오프
- **전제조건**: `std::fma`가 하드웨어 FMA로 컴파일돼야 함(Apple Silicon/NEON, x86 FMA3 모두 단일 명령 — 현 darwin 환경 OK). `-ffast-math`를 **켜면 안 됨**(재결합이 오차 보정을 파괴). 현재 CMake에 없으므로 안전하나, 향후 최적화 플래그 추가 시 문서화 필요.
- **정수 벡터**: peanut cross를 건드릴 경우 `is_floating_point` 가드 필수(위 참조).
- **효과 한계**: 일반적인 [-10,10] 규모 씬에서는 개선이 거의 안 보인다. 이득은 큰 월드 좌표/원거리/글랜싱 에지에서만 유의. 성능 개선은 아님(오히려 미미한 1.09배 비용).
- **EPSILON 미해결**: 위에서 밝혔듯 1e-3 EPSILON을 직접 고치지 않는다. 다만 barycentric 정확도 향상은 EPSILON 축소(TODO)의 사전작업이 될 수 있다.
- **서브모듈**: (b)안은 peanut 저장소 변경이라 별도 커밋/서브모듈 포인터 갱신 필요.

### 테스트 전략
- **단위 회귀**: `test/unit_tests.cpp`의 `Triangle::ray_intersect` 케이스는 씬 파일 없이 자체 완결적이라 최적의 harness다. 여기에 (i) 대좌표(1e4~1e6) 삼각형, (ii) 글랜싱/에지 관통 케이스를 추가하고, DOP 결과를 Double 참조값과 `is_approx`로 비교한다.
- **DOP 단위 테스트**: 블로그의 병리적 입력(33962/41563/… 계열)으로 `difference_of_products`가 double과 ≤1.5 ulp 일치하는지, naive는 크게 벗어나는지 직접 검증.
- **이미지 A/B**: `test/utils.h`의 `mse`/`rmse`/`diff`와 `simple_render_test`/`complex_render_test`(gt.exr 대비 `avg` 비율 허용오차)로 회귀 확인. 단 render 테스트는 `caramel-scenes`(TEST_SCENE_PATH) 서브모듈이 필요하므로 CI에서 초기화 전제.
- chi2 분포 테스트(`chi2_*`)는 교차 정밀도와 무관하므로 영향 없음(안전망).

### 노력·효과 재평가
**노력 = 낮음~중간**: 헬퍼 1개 + 호출부 3~9곳. watertight/헬퍼만이면 낮음, peanut cross까지면 서브모듈 커밋 탓에 중간. **효과 = 낮음~중간(견고성)**: 이전 패스의 "Top Pick"은 raw 임팩트 기준으로는 과대평가로 판단한다. 이유: (1) 기본 커널이 이미 Double fallback으로 최악 케이스를 방어 중, (2) 전형적 씬 규모에서 시각적 변화 거의 없음, (3) 성능 이득 없음. 그럼에도 값어치는 분명하다 — 대좌표/원거리 견고성, 결정적 watertightness 유지, 1.09배의 저렴한 보험, 그리고 향후 EPSILON 적응화의 토대. 저비용·저위험 correctness 개선으로 분류하는 것이 정확하다.

## T3. CHECK_RARE 통계 분기 카운터 — 실제 잠복 버그에 연결
**노력**: Medium · **효과**: Medium · **출처**: [CHECK_RARE and making sense of unusual occurrences](https://pharr.org/matt/blog/2018/05/31/check-rare.html)

### 현재 상태
caramel 의 유전체 굴절은 `Dielectric::sample_recursive_dir`(src/bsdfs/dielectric.cpp:34-64)가 Fresnel 비율로 reflect/refract 를 고르고, refract 분기에서 자유 함수 `refract()`(src/bsdfs/bsdf.cpp:42-50, `include/bsdf.h:54` 에 선언)를 호출한다. 그 함수는 sin_t 를 **독립적으로 다시 계산**하고 TIR 가드 없이 cos_t 를 구한다:

```cpp
// src/bsdfs/bsdf.cpp:42-50
Vector3f refract(const Vector3f &local_incoming_dir, const Vector3f &n, Float in_ior, Float ex_ior){
    const Float eta_ratio = ex_ior / in_ior;
    const Float sin_i = sqrt(1 - (local_incoming_dir[2] * local_incoming_dir[2])); // line 45
    const Float sin_t = snell_get_sin_t(sin_i, ex_ior, in_ior);                    // line 46
    const Float cos_t = sqrt(1 - (sin_t * sin_t));                                 // line 47  <- 가드 없음
    ...
}
```

반면 `fresnel_dielectric()`(bsdf.cpp:54-83)는 두 겹의 방어를 가진다: `cos_i`가 [0,1] 밖이면 clamp 하며 `CRM_WARNING`을 남기고(line 57-64), `sin_t >= Float1`이면 정확히 `Float1`을 반환한다(line 70). 그리고 `sample_1d()`는 `[0,1)` 범위다(src/samplers/uniformstd.cpp:56-60). 참고로 caramel 에는 CHECK_RARE/통계 시스템이 전혀 없다(코드베이스의 `thread_local`은 test/catch_amalgamated 뿐). `Float = float` 단정밀도(common.h:32)라 반올림 경계가 double 보다 넓다.

### 문제/기회
정통 TIR 은 사실 안전하다: TIR 이면 `reflect_ratio == Float1`, `sample_1d() <= 1.0`은 항상 참이라 refract 분기에 도달하지 못한다. NaN 은 오직 두 경로로만 샌다. (1) fresnel 과 refract 가 sin_t 를 **서로 다른 두 식**으로 계산하므로 TIR 경계에서 FMA-contraction/명령선택 차이로 한쪽만 `sin_t >= 1`이 되는 round-off 발산 — 블로그가 문서화한 바로 그 버그. (2) `local_incoming_dir`가 완벽히 정규화되지 않아 `dir[2]^2 > 1`이면 fresnel 은 cos_i 를 clamp 하지만 refract 는 안 해서 `sin_i = sqrt(negative) = NaN`. 둘 다 `EPSILON = 1e-3`(common.h:55) 수준의 좌표계 오차에서 충분히 그럴듯하다.

치명적인 점: 출력 경로에 NaN/firefly 방어가 **하나도 없다**(`isnan|isinf|clamp` grep 결과 0건). MCIntegrator.cpp:60 은 `rgb = rgb + get_pixel_value(...)`로 spp 를 누적하므로 샘플 하나만 NaN 이어도 그 픽셀 전체가 모든 spp 에 걸쳐 NaN 으로 오염된다. 굴절 유리 씬에서 드물게 검은/NaN 픽셀이 나올 수 있는데 원인 추적 수단이 없다. 또한 light 샘플링에는 조용히 zero 기여를 반환하는 rare 분기가 여럿 있어(area.cpp:76-78 isnan(dir), 84-86 !hit, 96-98 퇴화 근접) estimator 를 눈에 안 보이게 편향시킨다.

### 블로그 기법
Pharr 의 CHECK_RARE 는 "일어나면 안 되지만 부동소수 때문에 드물게 진짜로 일어나는" 사건을 hard assert 대신 **빈도**로 판정한다.

```cpp
#define CHECK_RARE(freq, condition) do {                      \
    static_assert(std::is_floating_point<decltype(freq)>::value);   \
    static thread_local int64_t numTrue, total;               \
    static StatRegisterer _([](RareSums &sums){ /* 스레드별 합산 */ \
        sums[title].first += numTrue; sums[title].second += total; \
        numTrue = total = 0; });                              \
    ++total; if (condition) ++numTrue;                        \
} while(0)
```

핵심 수치: 스레드별 `thread_local` 카운터(atomic 은 cache-coherency 비용 때문에 금지 — "catastrophic for performance"), 종료 시 2-표준편차 임계로 판정한다.

```
freqEstimate     = double(numTrue) / double(total);
varianceEstimate = 1/double(total-1) * freqEstimate * (1 - freqEstimate);
if (freqEstimate - 2*varianceEstimate >= maxFrequency) { /* report */ }
```

블로그의 대표 사례 두 개가 모두 caramel 과 직결된다: (a) bisection 근찾기에서 `if (t < a || t > b)`를 `<= / >=`로 고친 무한루프, (b) **"Fresnel 식과 Refract() 가 TIR 판정에 조금씩 다른 수학을 써서 반올림 때문에 드물게 불일치"** → `CHECK_RARE(1e-6, tir == true)`. caramel 의 상황과 정확히 동형이다.

### caramel 적용 설계
pbrt 의 `StatRegisterer` barrier 는 영속 스레드풀 전제인데 caramel `ThreadPool`(parallel_for.h)은 호출마다 `std::thread`를 생성·join 하는 **비영속** 풀이라 그대로 못 쓴다. 대신 self-registering 카운터를 둔다.

`include/check_rare.h` 신설:

```cpp
constexpr bool ENABLE_CHECK_RARE = true;   // TRY_SOLID_ANGLE_SAMPLING 스타일 컴파일 스위치
struct RareCounter {
    int64_t num_true = 0, total = 0; const char *title;
    RareCounter(const char *t);   // 전역 mutex 로 live-list 에 자기 등록
    ~RareCounter();               // join 시: 전역 accumulator(map<string,pair>) 로 병합 후 live-list 에서 제거
};
void report_rare_checks(double default_max_freq = 1e-6);  // accumulator + 아직 살아있는(main thread) 카운터 합산 → 2σ 판정 → CRM_WARNING

#define CHECK_RARE(freq, cond) do { if constexpr (Caramel::ENABLE_CHECK_RARE) { \
    static thread_local Caramel::RareCounter _rc(__FILE__ ":" CRM_STR(__LINE__) " " #cond); \
    ++_rc.total; if (cond) ++_rc.num_true; (void)double(freq); } } while(0)
```

계측 지점:
- refract() bsdf.cpp:46 직후 `CHECK_RARE(1e-6, sin_t >= Float1);`, 그리고 라인 45 직전 `CHECK_RARE(1e-6, !(local_incoming_dir[2]*local_incoming_dir[2] <= Float1));` — 두 잠복 경로 정조준.
- area.cpp:76 / 84 / 96 의 조기 return 조건에 각각 `CHECK_RARE` — 편향의 크기 정량화.
- imageEnvLight.cpp:165 극점 가드에 `CHECK_RARE(1e-4, abs(uv[1]-Float0)<1e-6 || abs(uv[1]-Float1)<1e-6);` — 가드 발화율 검증.

수집 훅: `render()`가 parallel_for 를 딱 한 번 호출하므로(MCIntegrator.cpp:53) join 직후(라인 68 이후, 기존 타이밍 로그 옆)에 `report_rare_checks();` 한 줄. worker thread_local 은 join 시 소멸자로 이미 accumulator 에 flush 되고, main thread 카운터만 live-list 에서 합산된다.

### 리스크·주의·트레이드오프
- **이것은 버그를 고치지 않는다** — 순수 관측 도구다. 실제 수정은 refract() 안에 `if (sin_t >= Float1) return reflect(...)`(또는 zero) 가드를 넣는 별개 작업이며, CHECK_RARE 는 그 수정이 정말 필요한지/빈도가 얼마인지를 먼저 증명한다.
- caramel 비영속 풀 때문에 thread_local 소멸자→전역 accumulator 병합 순서(특히 main thread 카운터가 report 시점에 아직 살아있음)를 정확히 다뤄야 한다. 등록/병합은 스레드당 1회라 per-sample 비용은 `++total; if(cond)++num_true;` 뿐 — atomic 없음.
- 2σ 판정은 표본이 커야 의미 있다(블로그: 작은 freq·큰 표본에서 variance 가 빠르게 작아짐). 저 spp 프리뷰 렌더에서는 발화가 없을 수 있다 — 대규모/장시간 렌더에서만 신뢰.
- 단정밀도(`float`)라 경계 사건 자체가 double 빌드보다 흔할 수 있음(도구엔 유리).

### 테스트 전략
기존 Catch2 + `test/` 인프라를 재사용한다. (1) 신규 `TEST_CASE`에서 `IOR::GLASS/VACUUM` 유전체를 만들고 임계각(약 41.8° for 1.5)을 스윕하는 입사각과 **의도적으로 비정규화한** `local_incoming_dir`로 `sample_recursive_dir`를 대량 호출, 반환 방향에 `std::isnan`이 하나도 없음을 assert — chi2_bsdf_test.cpp:29 의 `UniformStdSampler(42)` 구동 패턴을 그대로 차용. (2) CHECK_RARE 자체 단위 테스트: 알려진 확률(예 1e-3)로 참이 되는 합성 조건을 N=수백만 번 돌려 `report_rare_checks`가 임계 초과를 정확히 보고하고, 실제 rare(1e-8) 조건은 오탐하지 않는지 검증. (3) 회귀 방어로 complex_render_test.cpp 유리 씬 실행 후 report 로그에 refract TIR 카운터가 임계 초과로 뜨는지 확인.

### 노력·효과 재평가
**노력 Medium**(이전 저평가 가능성 교정). 매크로+2σ 판정은 작지만, 비영속 풀에서의 thread_local 하베스팅(소멸자 병합 + main thread 처리 + 정적 초기화 순서)과 FMA/반올림 근거 검증 때문에 pbrt 코드를 그대로 못 베끼고 설계가 필요 — 테스트 포함 0.5~1일. **효과 Medium**. 자체로는 무엇도 고치지 않지만, caramel 은 NaN/firefly 가드가 전무하고 NaN 한 샘플이 픽셀 전체를 오염시키므로(MCIntegrator.cpp:60) refract 경계·비정규화 경로를 수면 위로 올리는 실질 가치가 있고, light 샘플링의 조용한 편향 조기-return 들에 재사용 가능한 진단 인프라가 된다. 교육적 가치는 높음.

## T4. ImageEnvLight 등장방형 importance sampler 검증 (// ??? 해소)
**노력**: 낮음~중간 · **효과**: 낮음~중간 · **출처**: [Visualizing Warping Strategies for Sampling Environment Map Lights](https://pharr.org/matt/blog/2019/06/05/visualizing-env-light-warpings.html)

### 현재 상태
caramel은 등장방형(equirectangular) HDR 환경맵을 pbrt-v3 `InfiniteAreaLight`와 동일한 방식으로 importance sampling 한다. 빌드 시 각 픽셀 휘도에 `sin(theta)`를 곱해 2D 이산 분포(`Distrib2D`)를 만들고(`src/image.cpp:158`의 `get_data_for_sampling`, 실제 곱셈은 `:166`), `sample_direct_contribution`이 이 분포에서 픽셀을 뽑는다.

```cpp
// src/image.cpp:166-167  (get_data_for_sampling, sin_weight=true 로 호출됨)
const Float multiply = sin_weight ? sin(static_cast<Float>(h) / static_cast<Float>(m_height) * PI) : Float1;
temp.emplace_back(luminance(get_pixel_value(w, h)) * multiply);
```

PMF -> 입체각 밀도 변환은 `pdf_solidangle`에 있고, 그 위 주석(`imageEnvLight.cpp:151-163`)에 3단계 유도가 적혀 있으나 끝에 `// ???` 자기의심이 남아 있다.

```cpp
// src/lights/imageEnvLight.cpp:165-174
if (abs(uv[1] - Float1) < 1e-6 || abs(uv[1] - Float0) < 1e-6) { return 0; }   // pole guard
Vector2i pixel_idx(static_cast<int>(uv[0] * m_width), static_cast<int>(uv[1] * m_height));
if (pixel_idx[0] >= m_width) { pixel_idx[0] -= m_width; }
return m_width_height * m_imageDistrib.pdf/*technically it's pmf*/(pixel_idx[0], pixel_idx[1]) / (2 * PI * PI * sin(uv[1] * PI)); // ???
```

이 값은 `src/integrators/path.cpp`의 balance-heuristic MIS 양쪽(광원샘플링 `:106`, BSDF-히트 재평가 `:57`)에서 쓰인다. `luminance()`는 순수 산술평균이다(`common.h:149`: `(rgb[0]+rgb[1]+rgb[2])/3`).

### 문제/기회
**유도는 실제로 옳다.** `p_uv = P(i,j)*W*H` (픽셀->[0,1]^2, du·dv=1/W·1/H), `(u,v)->(theta,phi)`의 Jacobian `2*PI^2` (phi=2πu, theta=πv), `(theta,phi)->omega`의 `sin(theta)` -> `p_omega = P(i,j)*W*H/(2*PI^2*sin(theta))`. 라인 174와 정확히 일치하고 pbrt-v3 `InfiniteAreaLight::Pdf_Li`(동일 공식, 동일하게 연속 theta의 sin 사용)와 부호·상수까지 같다. 빌드시 `sin(theta)`(픽셀 입체각 보정)와 평가시 `/sin(theta)`(입체각 밀도 환산)는 서로 다른 이유로 존재하며 비(ratio)에서만 상쇄되어 `pdf ∝ luminance`가 된다. 즉 **`// ???`는 근거 없는 자기의심**이며, 남은 실질 과제는 (a) 이를 문서/테스트로 못박는 것, 그리고 조사 중 드러난 더 구체적인 두 결함이다:

1. **픽셀-중심 전용 샘플링(연속 샘플링 부재).** `Distrib2D::sample`은 정수 인덱스만 반환하고(`distribution.h:99-107`), `sample_direct_contribution`은 중심만 쓴다.
```cpp
// src/lights/imageEnvLight.cpp:131-132
const auto sampled_uv = m_imageDistrib.sample(sampler.sample_1d(), sampler.sample_1d());
const auto pos_to_light_local = normalized_uv_to_vec(Vector2f{(sampled_uv[0] + Float0_5) / m_width, (sampled_uv[1] + Float0_5) / m_height});
```
난수의 소수부를 버리고 픽셀 중심만 쏘므로 환경광은 사실상 W*H개 이산 방향의 점광원 집합이다. 그런데 `pdf_solidangle`은 픽셀 내부에 균일한 *연속* 밀도를 주장한다. 결과적으로 광원샘플링 추정량의 기대값이 midpoint quadrature(`Σ f(center)·A`)가 되어, 픽셀 내에서 변하는 항(cos·visibility·BRDF; radiance는 `get_pixel_value(int,int)` truncation으로 픽셀당 상수라 무관)에 대해 샘플을 늘려도 사라지지 않는 O(픽셀 입체각) **편향**이 남고, 부드러운 그라디언트/환경 반사에 **밴딩**을 만든다. pbrt는 `SampleContinuous`로 픽셀 내부를 jitter 해 이를 피한다.

2. **최상단 스캔라인 가중 0.** 빌드 가중이 `sin(h/H*PI)`로 픽셀 top edge를 쓰므로 `h=0` 행은 `sin(0)=0` -> 선택확률 0. pdf도 같은 행에서 0을 돌려주니 편향은 아니고 자기일관적이지만, 최상단(대개 천정 하늘) 행이 light sampling에서 통째로 빠지는 효율 손실이다. pbrt의 `(v+0.5)/H` 규약이면 해소된다.

### 블로그 기법
글은 유도가 아니라 두 warping의 *시각화 비교*다. (1) caramel이 쓰는 방식 = pbrt `Distribution2D`의 2-step: marginal PDF로 스캔라인, conditional PDF로 픽셀을 뽑음. (2) Clarberg et al. "Wavelet Importance Sampling"식 계층 warp: 환경맵 MIP-map 위에서 `f(x)=a(x-b)` 선형 warp를 두 차원 번갈아 적용해 [0,1)^2를 목표 분포로 점진 변형. 결론은 "세 개 환경맵 렌더 테스트에서 두 방식 오차가 비슷해 명확한 승자 없음 — 타깃 아키텍처에서 효율적인 쪽을 써라"이다. **즉 이 글의 caramel 관점 가치는 '현재 방식이 표준·정당함을 확증'하는 것**이지 `// ???`를 푸는 수식 제공이 아니다(수식은 PBR book). octahedral/equal-area/pole 특이점 논의는 이 글에 없다.

### caramel 적용 설계
우선순위대로:

- **P0 — 주석 정리 + 유도 확정.** `// ???` 제거, `:151-163` 주석에 "pbrt-v3 InfiniteAreaLight::Pdf_Li와 동치, `Jacobian=2*PI^2`, `dOmega=sin(theta)dθdφ`" 명시. 순수 문서 작업.

- **P1 — 연속 샘플링(핵심 개선).** `Distrib1D`/`Distrib2D`에 continuous 변형 추가, pdf 공식은 그대로 유지(이미 piecewise-constant 연속 밀도라 자동 정합).
```cpp
// distribution.h — Distrib1D
std::pair<Index, Float> sample_continuous(Float x) const {   // (index, offset in [0,1))
    const auto it = std::ranges::upper_bound(m_cdf, x);
    const Index i = it - m_cdf.begin();
    const Float lo = (i == 0) ? Float0 : m_cdf[i - 1];
    const Float du = m_pdf[i] > Float0 ? (x - lo) / m_pdf[i] : Float0;  // m_pdf already normalized
    return {i, du};
}
// distribution.h — Distrib2D
std::pair<Vector2f, Float> sample_continuous(Float x, Float y) const {
    auto [w, du] = m_width_distrib.sample_continuous(x);
    auto [h, dv] = m_height_distrib_list[w].sample_continuous(y);
    return {Vector2f{w + du, h + dv}, m_width_distrib.pdf(w) * m_height_distrib_list[w].pdf(h)};
}
```
```cpp
// imageEnvLight.cpp — sample_direct_contribution, 기존 131-132 교체
const auto [uv_px, pmf] = m_imageDistrib.sample_continuous(sampler.sample_1d(), sampler.sample_1d());
const auto pos_to_light_local = normalized_uv_to_vec(Vector2f{uv_px[0] / m_width, uv_px[1] / m_height});
```
`pdf_solidangle`은 여전히 `floor`로 픽셀을 찾으므로 무변경. 이로써 sampler가 pdf가 주장하는 밀도를 실제로 실현 -> 편향/밴딩 동시 제거.

- **P2 — top-edge 버그.** `image.cpp:166`을 `sin((static_cast<Float>(h) + Float0_5) / m_height * PI)`로. 1줄.

- **P3(선택) — luminance 정책.** Rec.709는 지각 튜닝일 뿐이라 보류 권장(아래 트레이드오프).

### 리스크·주의·트레이드오프
- P1은 sampler 밀도와 pdf를 *일치*시키는 방향이라 안전하지만, `Distrib1D::sample`의 반환 규약(현재 `upper_bound` 인덱스)과 continuous 변형의 offset 계산이 정확히 정합해야 한다(경계 픽셀에서 `m_pdf[i]==0`이면 `du=0` 폴백). MIS는 balance-heuristic이라 pdf가 바뀌지 않는 한 자동 정합.
- P2/P1 모두 `power()`·`radiance()`는 건드리지 않는다(스케일 무변경).
- P3 주의: env-**light** 타깃은 지각 luminance가 아니라 에너지이므로 `(r+g+b)/3`이 물리적으로 더 타당. Rec.709 전환은 분산 개선 보장이 없고 어두운-청색 강조 환경에서 오히려 나빠질 수 있음.
- **해결 못 하는 것**: 극점 근처 midpoint `sin(theta)` 근사 자체의 O(1/H^2) 오차(pbrt도 동일하게 감수), nearest-neighbor radiance로 인한 픽셀 스케일 계단. octahedral equal-area(상수 Jacobian, 극점 특이점 제거)는 큰 재작성이고 이 블로그와 무관하므로 별도 이슈로 분리.

### 테스트 전략
서브모듈이 실제로 채워져 있어(`ext/hypothesis/hypothesis.h` 존재) chi-square 테스트를 바로 빌드 가능. `test/chi2_bsdf_test.cpp`/`chi2_polygon_test.cpp` 패턴을 그대로 재사용한다.
- 작은 비균질 환경맵(예: 64x128, 밝은 점 몇 개 + 그라디언트)을 `Image(w,h)`+`set_pixel_value`로 만들어 `write_exr`로 임시 파일에 저장 후 `ImageEnvLight`로 로드(생성자가 경로만 받으므로 임시 EXR 경유가 최소 침습).
- 방향을 `(cosTheta, phi)` 격자(맵보다 훨씬 성긴 16x32)로 binning: 관측빈도는 `m_imageDistrib.sample_continuous`->`normalized_uv_to_vec`로 생성, 기대빈도는 `hypothesis::adaptiveSimpson2D`로 `pdf_solidangle`를 각 bin에서 적분×N. `hypothesis::chi2_test(res, obs, exp, N, 5, 0.01, testCount)`로 판정.
- 이 테스트가 잡는 것: Jacobian 상수(2π^2 오타), sin(theta) 유무/위치, pdf 정규화(∫=1). P1 적용 후에도 통과해야 하며(성긴 bin에서는 이산/연속 차가 평균화), 회귀 가드로 유용.
- 보조: `pdf_solidangle`이 반환하는 밀도를 반구/전구에서 Monte Carlo로 적분해 1에 수렴하는지 sanity 유닛테스트 1개 추가.

### 노력·효과 재평가
이전 pass의 핵심 주장(sin 가중·공식·pole guard·유도 존재·산술평균 luminance)은 모두 실측과 일치했다. 재평가:
- **P0(주석+chi2 테스트)**: 노력 **낮음**(서브모듈 실재로 이전 pass 추정보다 오히려 쉬움), 효과 **낮음**(신뢰·회귀 방어). 이전 pass 평가와 동일.
- **P1(연속 샘플링)**: 노력 **중간**(분포 클래스에 함수 2개 + 호출부 3줄), 효과 **중간**. 이전 pass가 놓친, 실제 편향·밴딩을 없애는 최고 가성비 항목이며 블로그의 레퍼런스 구현(pbrt `SampleContinuous`)이 그대로 근거가 된다. 종합 노력 **낮음~중간**, 효과 **낮음~중간**으로 상향 조정.

## T5. 스레딩 독립 결정성 (per-pixel-sample 해시 시드)
**노력**: 낮음 (sampler에 `advance()`/`hash` 추가 + MCIntegrator 약 5줄; gt 테스트 재실행 필요) · **효과**: 중간 (재현성/디버깅 replay, 정확도·속도 개선은 아님) · **출처**: [Debugging Your Renderer (5/n): Rendering Deterministically](https://pharr.org/matt/blog/2021/12/24/debugging-renderers-rendering-deterministically.html)

### 현재 상태
`MCIntegrator::render`는 열(column) 단위로 병렬화하고, 각 열마다 sampler를 하나 만들어 그 열의 모든 row·spp를 순차 소비한다 (`src/integrators/MCIntegrator.cpp:53-64`):

```cpp
parallel_for(0, size.first, std::function([&](int i){          // i = column index (width)
    std::random_device rd;
    UniformStdSampler sampler(config.random_seed ? static_cast<int>(rd()) : i);  // seed = i
    for(int j=0;j<size.second;j++){                            // j = row (height)
        Vector3f rgb = vec3f_zero;
        for(Index s=0;s<real_spp;s++){
            rgb = rgb + get_pixel_value(scene, i + sampler.sample_1d(), j + sampler.sample_1d(), sampler);
        }
        rgb = rgb / real_spp;                                  // box filter (uniform 평균)
        output.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
    }
}));
```

`get_size()`가 `{m_w, m_h}`를 반환하므로(`include/camera.h:49-51`) `size.first`는 width이고 `i`는 **열 인덱스**다 — 사전 조사의 "seed=column index"는 정확하다. `parallel_for`는 `next_task.fetch_add(1)`로 열을 나눠주고 한 태스크가 그 열의 j·spp 루프를 끝까지 실행하므로(`include/parallel_for.h:54-63`), "한 열을 한 스레드가 start-to-finish"도 정확하다. sampler는 PCG32(`m_state`, `m_inc`)이며 생성자는 `UniformStdSampler(uint64_t seed, uint64_t stream = 1)`로 sequence 인자를 받는다(`include/sampler.h:43`). 생성자 본문은 표준 `pcg32_srandom_r` 패턴이고(`src/samplers/uniformstd.cpp:31-41`), LCG 상수는 pbrt의 `PCG32_MULT`와 동일한 `6364136223846793005ULL`이다(`uniformstd.cpp:48`). `advance`/`hash`/`set_sequence`는 코드베이스 어디에도 없다(grep 확인).

한 가지 정정: `seed`는 PCG의 `initstate`(상태 오프셋)에 매핑되고 `stream`은 기본 1로 **고정**된다. 즉 모든 열이 *같은 sequence, 다른 시작 오프셋*을 쓴다. 블로그 기법은 반대로 sequence를 픽셀마다 바꾼다.

### 문제/기회
결정성이 **우연히** 성립한다. 결과가 재현되는 유일한 이유는 (1) seed=`i`가 스레드와 무관하고, (2) 한 sampler가 한 열의 RNG 소비 순서를 고정하며, (3) box-filter 합산이 `s=0..spp-1` 순차이기 때문이다. 그런데 한 열의 sampler 하나가 **모든 row × 모든 spp를 공유**하므로, 픽셀 `(i,j,s)`의 RNG 스트림 위치는 그 열에서 앞서 소비된 총 draw 수에 의존한다. 따라서 스레딩을 건드리지 않아도 — BSDF lobe 하나 추가, RR 분기 조정, `sample_1d()` 호출 재배열 등 — per-sample 소비를 바꾸는 어떤 수정이든 해당 열의 이후 전 픽셀 노이즈를 이동시킨다. 사전 조사는 이를 "tile/row 재병렬화 시"로만 좁혔지만, 활발히 개편 중인 이 코드베이스(최근 BVH flattening, env-map MIS 등)에서는 소비-cascade가 더 자주 터진다. 또 seed가 열 단위라 **단일 픽셀/샘플을 격리 재현할 수 없다** — `(x=5, y=100, s=3)` 하나만 디버깅하려 해도 `j=0`부터 그 열 전체를 돌려야 한다. 부수적으로 width < `hardware_concurrency`이면 열 granularity가 코어를 저활용한다.

### 블로그 기법
Pharr는 "샘플 값이 스레드 배정과 이전 소비량에 의존"하는 것을 근본 원인으로 지목하고, 각 픽셀-샘플 시작 시 RNG를 **좌표로부터 재시드**한다:

```cpp
void StartPixelSample(Point2i p, int sampleIndex, int dimension) {
    rng.SetSequence(Hash(p, seed));                 // 픽셀 좌표+유저 seed -> PCG sequence 선택
    rng.Advance(sampleIndex * 65536ull + dimension); // 그 sequence 내 결정적 오프셋으로 점프
}
```

핵심 수치·요소: (a) 샘플당 dimension budget `65536 = 2^16` — 샘플 인덱스마다 스트림에서 이만큼 떨어뜨려, 각 샘플이 소비하는 차원 수가 서로 달라도 겹치지 않게 한다. (b) `Hash(...)`는 인자 바이트를 MurmurHash로 마샬링; `HashFloat(...) = uint32_t(Hash(...)) * 0x1p-32f`(= 2^-32)로 [0,1) 변환. (c) PCG의 `Advance(delta)`는 LCG jump-ahead(곱셈자/증분 repeated squaring)로 `O(log2 delta)`에 임의 오프셋 점프 — PCG가 "sequence 선택 + 오프셋 지정"을 모두 지원하기에 가능하다. 이로써 `(x,y,sample)` → 난수열이 **traversal 순서와 완전 독립**이 되어 어떤 병렬 분해에서도 동일하고, 단일 픽셀 replay가 가능해진다. Caveat로 명시된 것: struct padding을 해시하면 padding 바이트가 결과를 오염시킴; light-tracing의 splat 기여는 여전히 비결정적; 결정성은 "특정 머신·컴파일러" 한정.

### caramel 적용 설계 (미적용)
caramel의 생성자가 이미 SetSequence-등가 로직을 담고 있으므로 재사용한다.

1. **hash 유틸** (`include/common.h` 또는 새 `include/hash.h`) — splitmix64 finalizer 기반, 바이트 마샬링 불필요:
```cpp
inline uint64_t mix_bits(uint64_t v){
    v ^= v >> 31; v *= 0x7fb5d329728ea185ull;
    v ^= v >> 27; v *= 0x81dadef4bc2dd44dull;
    v ^= v >> 33; return v;
}
inline uint64_t hash_seed(uint32_t x, uint32_t y, uint64_t seed){
    return mix_bits(((uint64_t(x) << 32) | y) ^ mix_bits(seed));
}
```

2. **`UniformStdSampler`에 추가** (`include/sampler.h`, `src/samplers/uniformstd.cpp`):
```cpp
void set_sequence(uint64_t sequence, uint64_t seed);           // 기존 생성자 본문 추출
void advance(uint64_t delta);                                  // NEW: PCG jump-ahead
void start_pixel_sample(uint32_t x, uint32_t y, uint32_t s);   // NEW
// 멤버: uint64_t m_base_seed;  (생성자에서 저장)
```
```cpp
void UniformStdSampler::advance(uint64_t delta){
    uint64_t cur_mult = 6364136223846793005ULL, cur_plus = m_inc, acc_mult = 1u, acc_plus = 0u;
    while(delta > 0){
        if(delta & 1){ acc_mult *= cur_mult; acc_plus = acc_plus * cur_mult + cur_plus; }
        cur_plus = (cur_mult + 1) * cur_plus;  cur_mult *= cur_mult;  delta >>= 1;
    }
    m_state = acc_mult * m_state + acc_plus;
}
static constexpr uint64_t SAMPLE_STRIDE = 1ull << 16;          // 65536
void UniformStdSampler::start_pixel_sample(uint32_t x, uint32_t y, uint32_t s){
    const uint64_t seq = hash_seed(x, y, m_base_seed);
    set_sequence(seq, mix_bits(seq));      // sequence(=m_inc)를 픽셀마다 바꿈
    advance(uint64_t(s) * SAMPLE_STRIDE);  // 샘플 오프셋 (dim은 caramel에선 순차소비라 0 고정)
}
```

3. **`MCIntegrator::render` 변경** (`src/integrators/MCIntegrator.cpp:53-64`) — seed를 열이 아니라 **전역**으로 옮기고 좌표를 해시에 통과시킨다:
```cpp
const uint64_t base_seed = config.random_seed ? std::random_device{}() : config.seed; // RenderConfig에 seed 추가
parallel_for(0, size.first, std::function([&](int i){
    UniformStdSampler sampler(base_seed);
    for(int j=0;j<size.second;j++){
        Vector3f rgb = vec3f_zero;
        for(Index s=0;s<real_spp;s++){
            sampler.start_pixel_sample(i, j, s);   // <-- 추가: (pixel,sample)로 재시드
            rgb = rgb + get_pixel_value(scene, i + sampler.sample_1d(), j + sampler.sample_1d(), sampler);
        }
        rgb = rgb / real_spp;
        output.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
    }
}));
```
GUI progressive renderer는 현재 `config.spp=1; config.random_seed=true`로 프레임마다 누적한다(`gui_src/progressive_renderer.cpp:104-105`). 여기선 `start_pixel_sample(i, j, frame_index)`로 프레임 인덱스를 샘플 축에 흘려 넣으면 프레임마다 새 노이즈 + 프레임 단위 재현성을 동시에 얻는다.

### 리스크·주의·트레이드오프
- **정확도/속도 개선 아님.** 현재 이미 결정적이므로 이 작업의 가치는 오직 재현성 견고화 + 단일 픽셀 replay 디버깅이다.
- **합산 순서는 별개 문제.** per-sample 값이 결정적이어도, 한 픽셀의 spp를 여러 스레드로 쪼개 비결정적 순서로 더하면 float 비결합성 때문에 최종 픽셀은 다시 어긋난다. 현재처럼 픽셀당 spp 루프를 한 주체가 index 순서로 합산하면 유지된다(블로그도 splat 비결정성은 미해결로 명시).
- **STRIDE 한계.** path tracer의 샘플당 draw 수는 bounce당 약 5~6개 × depth로 대개 수천 미만 → 65536 여유 안에 충분. 병적으로 긴 경로가 65536을 넘으면 다음 샘플과 충돌하므로 STRIDE를 키우거나 config화.
- **전제 없음.** QMC/Sobol 같은 저불일치 샘플러가 없어 dimension 추적이 불필요 — 순수 PCG라 오히려 적용이 단순하다.
- 미세 부수효과: sequence를 해시로 바꾸면 인접 열의 상관이 (지금의 same-sequence 대비) 더 잘 깨져 노이즈가 근소 개선될 수 있으나 본질은 아니다.

### 테스트 전략
1. **결정성 유닛 테스트 신설**(`test/unit_tests.cpp`): 동일 scene을 두 번 렌더 → 픽셀 bit-exact 동일 `CHECK`. 나아가 `parallel_for`를 강제 1-스레드로 돌린 결과와 멀티스레드 결과를 비교해 **스레드 독립성**을 직접 증명(핵심 회귀 가드).
2. **단일 픽셀 replay 테스트**: `start_pixel_sample(x,y,s)` 후 뽑은 난수 시퀀스가 전체 렌더 중 같은 픽셀에서 나온 것과 일치하는지 확인 → replay 기능 자체를 검증.
3. **기존 gt 회귀 재사용**: `simple_render_test`(7) + `complex_render_test`(다수)의 avg-비율 tolerance 검사(`utils.h:44-45`, `simple_render_test.cpp:53-54`)를 그대로 재실행. 이들은 평균 휘도 비율을 보므로 노이즈 재배치에 둔감 — **gt.exr 재베이스라인 불필요**(사전 조사의 주의사항을 정정). tolerance만 통과하면 편향 없음이 확인된다.

### 노력·효과 재평가
**노력: 낮음.** PCG의 `(state, inc)` 구조와 `srandom` 패턴 생성자가 이미 있어 신규 코드는 `advance()`(약 10줄, 표준 jump-ahead), `hash_seed`(수 줄), `start_pixel_sample`(수 줄), MCIntegrator 배선(약 5줄) 뿐이다. 다만 sampler 시드를 건드리면 모든 scene 노이즈가 바뀌어 gt 기반 렌더 테스트(약 13개+)를 재실행·확인해야 하고, GUI progressive 경로도 손봐야 해 완전 무마찰은 아니다. **효과: 중간.** 정확도/성능엔 무영향이고 오늘도 결정적이지만, (1) 활발히 개편되는 integrator/BSDF 수정마다 노이즈가 조용히 이동하는 취약성을 제거하고, (2) tile/row/GPU 재병렬화에 대한 재현성을 사전 보장하며, (3) firefly 픽셀 하나를 격리 재현하는 디버깅 workflow를 연다. 사전 조사의 낮음/중간 평가에 동의하되, 실제 가치의 무게중심은 "스레딩"보다 **디버깅 replay + 소비-cascade 견고화**에 있다.

## T7. 장기: Wavefront/SoA path tracer → SIMD/GPU, 그리고 Amdahl startup 조각

**노력**: 전체 재작성 매우 높음(수개월) · Amdahl 조각은 낮음~중간 · **효과**: 장기적으로 높음, 현재 스케일에서는 낮음~중간 · **출처**: [Swallowing the Elephant (Part 10): Rendering on the GPU—Finally](https://pharr.org/matt/blog/2021/07/29/moana-rendered-on-the-gpu.html)

### 현재 상태

caramel의 렌더 루프는 철저히 스칼라 + depth-first per-sample 구조다. `MCIntegrator::render`가 이미지 폭(`size.first == m_w`, `camera.h:49-51`)에 대해 `parallel_for`를 돌려 **열(column) 단위**로 병렬화하고, 각 픽셀-샘플은 `get_pixel_value`가 반환한다:

```cpp
// src/integrators/MCIntegrator.cpp:53-63  (열 병렬, 샘플러 시드 = 열 인덱스)
parallel_for(0, size.first, std::function([&](int i){
    UniformStdSampler sampler(config.random_seed ? static_cast<int>(rd()) : i);
    for(int j=0;j<size.second;j++){
        Vector3f rgb = vec3f_zero;
        for(Index s=0;s<real_spp;s++)
            rgb = rgb + get_pixel_value(scene, i + sampler.sample_1d(), j + sampler.sample_1d(), sampler);
        rgb = rgb / real_spp;
        output.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
    }
}));
```

`PathIntegrator::mis_sampling_path`는 **재귀가 아니라 bounded loop**로 한 경로를 끝까지 추적한다(prior 패스의 "recursion" 표현은 부정확):

```cpp
// src/integrators/path.cpp:45-52 — 경로 하나를 depth-first로 끝까지
Ray ray = scene.m_cam->sample_ray(i, j, sampler);
for(Index depth=1; depth<=m_max_depth; depth++){
    const auto [is_hit, info] = scene.ray_intersect(ray);   // 단일 Ray
    ...
}
```

레이는 한 번에 하나(`ray.h:30-40`, packet 없음), BVH traversal도 단일 레이 + `int to_visit_stack[64]`(`bvh_base.cpp:194-241`)다. 메시는 속성별 분리 배열(`m_vertices`/`m_normals`/`m_tex_coords`/`m_face_indices`, `shape.h:158-161`)이지만 삼각형 테스트마다 인덱스로 정점 3개를 **gather**한다:

```cpp
// src/shapes/triangle_mesh.cpp:238-241
const Vector3i& idx = m_face_indices[i];
const Vector3f &p0 = m_vertices[idx[0]];
const Vector3f &p1 = m_vertices[idx[1]];
const Vector3f &p2 = m_vertices[idx[2]];
```

`Vector3f = Peanut::Matrix<float,3,1>`(`common.h:70`)로 xyz-packed 스칼라 → lane 병렬 `x[]/y[]/z[]` 없음. 교차 커널(`shape.cpp:46-164`)도 완전 스칼라. `CMakeLists.txt`에 `-march`/AVX/CUDA/ISPC/OpenMP/fast-math 플래그가 하나도 없다.

**Startup 쪽 사실**: (1) BVH 빌드는 단일스레드 재귀이며 노드마다 새 vector를 할당한다.

```cpp
// src/bvh_base.cpp:135-145
std::vector<Primitive> left(mid - m_primitives.begin());
std::vector<Primitive> right(m_primitives.end() - mid);
std::move(...); std::move(...); m_primitives.clear();
m_left  = std::make_unique<BVHNode>(std::move(left), traits);
m_right = std::make_unique<BVHNode>(std::move(right), traits);
m_left->create_child(...);  m_right->create_child(...);   // 재귀
```

(2) 메시별 BVH는 `TriangleMesh::finalize`에서 파싱 도중 순차 빌드된다(`triangle_mesh.cpp:70-71`). (3) OBJ weld는 정렬 `std::map`(`objmesh.cpp:71`)이고 파일당 shape 1개만 허용(`objmesh.cpp:58-60`). (4) envmap `get_data_for_sampling(true)`가 **두 번** 실행된다 — `imageEnvLight.cpp:48`(샘플링 분포)과 `:101`(`power()`, `scene.cpp:113`에서 호출). **결정적으로, startup 단계는 어디에도 계측이 없다** — `render`만 타이밍된다(`MCIntegrator.cpp:47-73`), `build_scene`(`render.cpp:36-60`)은 무계측.

### 문제/기회

Part 10의 핵심은 SoA/커널 메커니즘이 아니라 **Amdahl 법칙**이다: 렌더가 12~17배 빨라지면 고정 startup 비용(파싱·BVH·텍스처)이 wall-clock을 지배한다. caramel은 현재 render 시간이 워낙 길어 startup이 안 보이지만, 방향성은 동일하다. 그런데 startup이 **측정조차 안 되는** 상태라 "Amdahl 조각"은 지금 당장 싸게 착수 가능한 실질 슬라이스다. 전면 SIMD/wavefront는 처리량의 가장 큰 미개척 표면이지만 수개월짜리 재설계다.

### 블로그 기법

Part 10의 실측치(Moana, 256 spp, 1920×804):
- **속도향상**: main view GPU(RTX A6000) 26.7s vs CPU(32코어 3970X) 326.5s = **12.2×**; roots 카메라 32.1s vs 557.6s = **17.4×**; 2048 spp도 >12×.
- **Amdahl**: GPU 렌더 후 전체 ~90s 중 **약 2/3(~60s)가 "getting things ready to render", ~30%(~27s)만 실제 렌더**. Pharr: *"if you want to see that image sooner, optimizing startup time can be a better place to focus than optimizing rendering time."* Amdahl 상한: 전체 가속 `S_overall = 1 / ((1-p) + p/s)` — 렌더 비중 `p`를 `s`배 가속해도 startup `1-p`가 하한. `p=0.3`이면 렌더를 무한 가속해도 `S_overall ≤ 1/0.7 ≈ 1.43×`.
- **통합 코드베이스**: CPU·GPU가 *"other than the ray intersection routines, curves, and Ptex ... run the same C++ code."* GPU 렌더 내역: closest-hit 45.4% + shadow 32.0% = **레이 교차 77.4%**, material/BSDF 16.3%, sample gen 2.4%. 즉 backend별 특화가 필요한 부분은 교차뿐, BSDF/light/sampler는 공유.
- 메모리 29.0 GB. Ptex는 face 평균색만 배열로(프로덕션 아님). **이 글 자체는 SoA/wavefront 커널 유도를 담지 않는다**(시리즈 앞부분·pbrt 책 소관).

### caramel 적용 설계

세 단계로 분리해 위험/노력을 계단식으로 관리한다.

**Phase A — Amdahl 계측 + startup 저비용 수정 (지금, 낮은 노력):**
- `render.cpp:36-60` `build_scene`에 phase별 타이머 삽입(parse / mesh-BVH / scene-BVH / light-pdf). `MCIntegrator`의 `std::chrono` 패턴 재사용, `logger.h`로 출력. 이것이 이후 모든 판단의 근거.
- envmap 중복 제거: `build_sampling_distrib`가 만든 `data`(luminance×sinθ)를 재사용해 `power()`가 다시 `get_data_for_sampling(true)`를 부르지 않도록 캐시. 시그니처 예: `Float ImageEnvLight::power()`가 멤버 `m_lum_sum` 사용.
- OBJ weld `std::map` → `std::unordered_map<std::tuple<int,int,int>,Int>` + tuple 해시(`objmesh.cpp:71`).

**Phase B — startup 병렬화 (중간 노력, wavefront 전 선결):**
- 메시별 BVH를 메시 간 병렬로. `finalize`의 인라인 빌드(`triangle_mesh.cpp:70-71`)를 파싱과 분리해, 모든 `TriangleMesh`를 모은 뒤 `parallel_for(0, meshes.size(), [](int m){ meshes[m]->build_accel(); })`. (주의: `parallel_for`가 전역 `ThreadPool` 싱글턴이라 재진입 시 워커 생성 중첩 위험 — 중첩 호출 금지 규칙 필요.)
- `bvh_base.cpp:135-136`의 노드별 vector 할당을 arena/index 기반으로 교체(primitive를 한 배열에 두고 `[begin,end)` 구간만 재배치). 병렬 빌드 전에 필수(malloc-mutex 경합 제거).

**Phase C — wavefront/SoA 코어 (장기, 매우 높은 노력):** seam은 이미 존재한다 — `Integrator::pre_process`(현재 no-op, `MCIntegrator.cpp:76`)에서 큐/버퍼를 만들고, per-sample 스칼라 `get_pixel_value`를 stage 커널들로 대체.

```cpp
// SoA 레이/경로 큐 (before: RayIntersectInfo 값 반환, 폴리모픽 Shape*)
struct RayQueueSoA {
    std::vector<float> ox, oy, oz, dx, dy, dz;   // 레이
    std::vector<float> beta_r, beta_g, beta_b;   // throughput
    std::vector<uint32_t> pixel;                  // 결과 산란지
    std::vector<float> pdf_prev; std::vector<uint8_t> from_specular;
};
// backend-agnostic 코어 (BSDF/light/sampler) — 백엔드 무관
void shade_stage(const SceneSoA&, const HitQueueSoA&, RayQueueSoA& next, ...);
// backend-specific 하나 — 교차만 특화
void intersect_stage(const SceneSoA&, const RayQueueSoA&, HitQueueSoA&);  // CPU-SIMD / GPU 교체점
```

per-bounce 루프를 breadth-first로 뒤집어: `generate → [intersect → shade → (light-sample=shadow queue) → scatter]×depth`. 이때 **폴리모픽 `Shape*`/`BSDF*` 디스패치(`shape.h:52`, `integrators.h:70`)가 최대 장애물** — GPU/SIMD lane divergence를 피하려면 material을 tagged union/enum id로 바꾸고 shade 전에 material sort가 필요하다. 메시는 삼각형별 `p0/p1/p2`를 미리 펼친 SoA(x0[],y0[],... 또는 packed float4) 버퍼로 두어 gather 제거.

### 리스크·주의·트레이드오프

- **범위**: Phase C는 사실상 새 렌더러다. caramel의 값-반환 API(`std::pair<bool,RayIntersectInfo>`), 폴리모픽 dispatch, 스칼라 `Vector3f` 전제와 정면충돌. 부분 도입 시 두 경로를 동시 유지하는 부담.
- **선결조건**: SIMD 이득은 -march/AVX 활성 + 코어 벡터화 없이는 안 나온다. GPU는 서브모듈에 CUDA/OptiX/Metal 백엔드가 전무(현재 GUI만 macOS OpenGL) — 신규 의존성.
- **스케일 한정**: 현재 테스트 씬은 startup이 병목이 아니다. Amdahl 조각의 효과는 대규모 씬/저-spp/GUI 재로딩에서만 유의. Phase A는 "측정 인프라"라는 점에서 정당화되지, 즉각 속도이득은 작다.
- **결정성 결합**: 열 병렬 + 열 시드(`MCIntegrator.cpp:56`)에 재현성이 우연히 결합돼 있어, wavefront로 순서가 바뀌면 픽셀 노이즈가 달라진다 → tolerance 기반 `gt.exr` 재검증 필요(T5의 per-pixel 해시 시드가 선결되면 안전).
- **미해결**: 이 작업은 `EPSILON=1e-3`(`common.h:55`) shadow acne, 스펙트럴 부재 등 정확도 gap을 고치지 않는다. 순수 처리량 과제.

### 테스트 전략

- **Phase A/B**: startup 계측은 로그로 확인. 병렬 BVH·arena·unordered_map 후 기존 `simple_render_test`/`complex_render_test`가 **바이너리 동일 또는 tolerance 내** 이미지를 내야 함(빌드 순서만 바뀌고 결과 불변이 목표). BVH 정확성은 naive traversal 대비 교차 결과 일치로 별도 검증(현재 `NaiveMeshAccel`이 죽은 코드지만 oracle로 부활 가능).
- **Phase C**: intersect_stage를 기존 단일-레이 `ray_intersect`와 랜덤 레이 배치로 크로스체크(u,v,t 일치). shade_stage는 backend-agnostic이므로 스칼라 경로와 동일 코드여야 하고, 전체는 furnace/해석해(T1) + `chi2_bsdf_test`로 회귀 감시. QMC/해시 시드 도입 후 `gt.exr` 재기준화.
- 공통: `test/utils.cpp`의 미사용 `mse`/`rmse`를 이때 활성화해 "평균만 검사"의 사각지대를 닫고 재분포 회귀를 잡는다.

### 노력·효과 재평가

prior의 "높음/높음"을 **분해**한다. 전체 wavefront+SIMD+GPU는 **매우 높은 노력**(수개월, 새 백엔드·material sort·SoA 전면 도입)에 **장기 높은 효과**(Pharr 12~17× 근거). 그러나 caramel의 현재 병목은 render이지 startup이 아니라서 **Amdahl 조각의 오늘 효과는 낮음~중간**이다. 다만 Phase A(startup 계측 + envmap 중복/`std::map`/직렬 BVH 제거)는 **낮은 노력**이고, "무엇을 최적화할지" 판단의 전제가 되므로 착수 가치가 가장 높다. 권고 순서: **A(측정·저비용 수정) → B(startup 병렬화·arena) → C(wavefront/SoA, 별도 장기 트랙)**. C는 스펙트럴·QMC·결정성(T5) 등 다른 개선이 자리 잡은 뒤 진입하는 것이 안전하다.

# Also Consider

## A1. 카메라 공간(ish) 렌더링 — geometry를 평행이동해 카메라를 원점에
**노력**: Medium · **효과**: Low(현재 씬)~Medium(대규모·원점 이탈 씬 대비 robustness) · **출처**: [Rendering in Camera Space(ish)](https://pharr.org/matt/blog/2018/03/02/rendering-in-camera-space.html)

### 현재 상태
caramel은 mesh 정점을 **로드 시 world-space로 bake** 한다. 세 로더 모두 파서가 넘긴 `to_world` 로 정점을 곧바로 변환해 저장한다.

```cpp
// src/shapes/inline_triangle_mesh.cpp:42-44
for (const auto &p : positions)
    m_vertices.emplace_back(transform_point(p, transform));   // world 좌표로 굳힘
// plymesh.cpp:67-71, objmesh.cpp:82-87 동일 패턴
```

카메라 world 위치는 이미 명시적으로 보관된다 — 블로그의 step 1(원점을 world로 변환)이 그대로 코드에 있다.

```cpp
// src/cameras/camera.cpp:51
m_pos = Block<0,0,3,1>(m_cam_to_world * Vector4f{0.0f, 0.0f, 0.0f, 1.0f}); // == 카메라 world pos
// pinhole.cpp:47 : return {m_pos, d.normalize()};  방향은 m_cam_to_world 의 회전블록만 사용
```

정점·카메라·라이트 모두 **하나의 절대 world 좌표계**에서 산다. 자기교차 방지용 offset은 고정 상수 하나뿐이다.

```cpp
// include/common.h:55
constexpr Float EPSILON = static_cast<Float>(1e-3); // TODO : lower epsilon / adaptive
```

이 EPSILON은 전부 **좌표에 절대량으로 더하는** 방식으로 쓰인다: continuation ray `p + world_d*EPSILON` (rayintersectinfo.cpp:38), shadow ray `pos1 + dir*EPSILON` (scene.cpp:84), 가시성 허용오차 `<= EPSILON*1.1` (scene.cpp:88). `Float = float`(common.h:32).

### 문제/기회
float32 간격(ULP)은 원점에서 멀수록 커진다. 블로그 수치와 caramel의 EPSILON을 대입해 임계를 계산하면:

| 좌표 |x| | ULP(float32) | EPSILON/ULP |
|---|---|---|---|
| 1 | 1.19e-7 | ~8389 (여유 충분) |
| 1e3 | 6.1e-5 | ~16 |
| **1.6e4** | 9.8e-4 | **~1.0 (임계)** |
| 1e5 | 7.8e-3 | 0.13 |
| 1e6 | 6.25e-2 | 0.016 (완전 소멸) |

즉 hit point `p` 의 좌표가 ~1.6e4 를 넘으면 `p + dir*1e-3` 가 반올림되어 **다시 `p` 자신**이 되고, ray가 표면에서 그대로 재출발 → self-intersection(acne / 헛그림자 / light leak). 1e6에서는 offset이 1 ULP의 1/60이라 확실히 사라진다(블로그: "1000km 떨어지면 6cm보다 작은 디테일 표현 불가"). recenter는 near-camera 좌표를 O(sceneRadius)로 되돌려 이 magnitude 실패를 제거한다. **단, caramel/CLO 씬은 O(1~10)** 규모라 현재 8000 ULP 이상 여유 → 실질 위험은 잠재적이다.

### 블로그 기법
Pharr는 두 단계로 논증한다. (1) **Full camera space**: geometry에 world-to-camera를 prepend → 렌더 시간 ~20%↑, "ray-triangle intersection test 2.37배". 원인은 회전된 geometry가 축정렬 BVH box를 헐겁게 만들어서다. (2) **Camera space(ish)**: world-to-camera를 translation과 rotation으로 분해해 **translation만** geometry의 object-to-world에 prepend하고, **rotation만** 카메라에 남긴다 → "성능이 시작점으로 복귀". 알고리즘:
```
cam_pos = M_cam_to_world * (0,0,0)          // step 1
T = translate(-cam_pos)                     // step 2: 평행이동 성분만
각 geometry: to_world' = T * to_world        //         → 카메라가 원점
카메라: 회전만 유지 (translation → 0)          // step 3
```
핵심: **평행이동은 AABB의 축정렬성을 보존**하므로 BVH가 그대로다(20%/2.37배 회귀 회피). Pharr는 "대부분의 well-behaved 씬에선 화질이 눈에 띄게 좋아지진 않는다"고 명시하고, self-intersection의 **각도** 문제는 별개로 pbrt-v3의 error-bound 기반 robust shadow-ray offset이 해결한다고 짚는다.

### caramel 적용 설계 (미적용)
build_scene가 이미 camera를 shapes보다 먼저 파싱하므로(render.cpp:42→44) 오프셋을 파서에 주입할 수 있다. **translation-only + magnitude 자동 게이트**로 설계한다.

```cpp
// include/camera.h : getter 추가 (m_pos 는 protected)
Vector3f position() const { return m_pos; }

// include/scene_parser.h : 오프셋 멤버 + 헬퍼
Vector3f m_world_offset{Float0,Float0,Float0};
void set_world_offset(const Vector3f &o){ m_world_offset = o; }
Matrix44f recenter(const Matrix44f &m) const {
    return translate(-m_world_offset[0],-m_world_offset[1],-m_world_offset[2]) * m; }
Vector3f  recenter(const Vector3f &p) const { return p - m_world_offset; }
```
```cpp
// src/render.cpp build_scene : parse_camera 직후
Camera *cam = parser.parse_camera();
const Vector3f cam_pos = cam->position();          // 이미 계산된 값
if (cam_pos.length() > RECENTER_THRESHOLD) {        // 예: 1e3. 소규모 씬은 no-op → 골든 이미지 불변
    parser.set_world_offset(cam_pos);
    const auto [w,h] = cam->get_size();
    const Matrix44f c2w = translate(-cam_pos[0],-cam_pos[1],-cam_pos[2]) * cam->get_cam_to_world();
    Camera *r = cam->clone_with_transform(c2w, w, h); // matrix ctor 가 m_pos 를 0 으로 재계산
    delete cam; cam = r;
}
```
파서에서 **world에 고정된 모든 앵커**에 `recenter()` 적용: obj/ply/trianglemesh 의 `to_world`(scene_parser.cpp:234-236, 244-246, 302), Instance 의 `to_world`(:196), 단일 Triangle 의 p0/p1/p2(:250-262), point light 의 pos(:312). envmap `to_world`(:322)는 **방향 회전이라 평행이동 불변 → 손대지 않음**. Instance는 bake하지 않지만 recenter가 최외곽 world 배치행렬에 붙으므로 template(local)은 그대로 두고 배치만 이동해 정확하다. clone_with_transform·get_cam_to_world·translate 는 모두 기존 API라 카메라 내부 수정이 없다.

### 리스크·주의·트레이드오프
- **현재 이득 거의 없음**: caramel/CLO 씬은 원점 근처 소규모라 EPSILON 여유가 수천 ULP. 대규모·원점 이탈(도시/CAD/위성) 씬에 대한 보험이다.
- **각도 문제 미해결**: offset이 geometry normal이 아니라 ray 방향(rayintersectinfo.cpp:38, TODO at :37)이라 grazing angle self-intersection은 그대로. 진짜 robust 해법은 pbrt식 per-hit error-bound + normal 기반 OffsetRayOrigin이고, caramel엔 그쪽이 더 시급하다. recenter는 보완재이지 대체재가 아니다.
- **bake 정밀도**: `transform_point` 는 float 행렬 연산이라 큰 정점에서 cam_pos를 빼는 그 1회 연산은 여전히 ~1 ULP를 잃는다(단 결과가 작아져 downstream 전부 이득). 최대 이득을 원하면 bake 시 감산을 double로 한 뒤 cast(PLY는 이미 double 로드).
- **HitPos AOV 이동**: HitPosIntegrator(parse "hitpos") 출력이 cam_pos만큼 render-space로 이동. world 절대좌표 소비자는 offset을 되더해야 한다.
- 회전 미적용이므로 BVH 축정렬 유지 → Pharr의 20% 회귀 없음(설계의 핵심 이유).

### 테스트 전략
기존 인프라 재사용: `TEST_BODY` 매크로와 simple_render_test.cpp는 씬을 렌더해 gt.exr 대비 `avg` 비율을 tight tolerance(~1e-3)로 본다(utils.h:35-45); rmse()도 있다(utils.h:60).
1. **far-scene 동치성(핵심)**: test1/cbox를 카메라·모든 shape translate·point light pos에 +[1e6,1e6,1e6]를 더한 사본으로 만든다. GT는 원점에서 렌더한 원본 이미지. recenter ON이면 far 렌더가 원본을 재현해야 함(rmse ≈ 0, avg 비율 ∈ [0.998,1.002]).
2. **음성 대조군**: 같은 far-scene을 recenter OFF로 렌더 → acne/검은 얼룩/NaN으로 rmse가 크게 튀어야 함(버그 재현 증거).
3. **회귀**: test1~7을 게이트로 인해 offset=0(소규모)로 렌더 → 기존 골든과 bit 수준 동일(현 tolerance 통과). 자동 게이트가 소규모 씬을 절대 건드리지 않음을 보장.
4. **단위 테스트**(unit_tests.cpp): `float p=1e6f; assert((p+1e-3f)==p);`로 offset 소멸을 직접 증명하고, recenter 후 `p'~O(10)`에서 `(p'+dir*EPSILON)!=p'` 및 recentered 카메라 `position()==0` 확인.

### 노력·효과 재평가
- **노력: Medium**(prior의 은근한 저평가 상향). one-liner가 아니라 render.cpp(오케스트레이션+카메라 recenter), scene_parser.cpp의 4개 world 앵커, camera.h getter, 게이트 상수까지 cross-cutting이다. 까다로운 지점은 Triangle 직접-정점 경로, double bake 뉘앙스, 골든 이미지 비교란. 테스트 포함 ~1일.
- **효과: Low(현재)~Medium(대비)**. magnitude half는 지우지만 caramel 씬 규모에선 지금 당장 물지 않는다. 코드로 확인된 더 큰 robustness 공백은 **각도 half**(rayintersectinfo.cpp:37 TODO)이며 이건 recenter로 안 고쳐진다. 따라서 우선순위는 normal 기반 offset/pbrt error-bound < 뒤. recenter는 저비용 보험 + 대규모 씬 확장 시 전제로 두는 게 합리적이다.

## A2. 바이너리 지오메트리(PLY) 선호 + OBJ weld를 unordered_map + 측정 후 최적화
**노력**: 낮음(weld swap)~중간(PLY 파이프라인) · **효과**: 낮음~중간 (load-time 한정, render 무관) · **출처**: [Swallowing the elephant (part 1)](https://pharr.org/matt/blog/2018/07/08/moana-island-pbrt-1.html)

### 현재 상태
OBJ 로딩(`src/shapes/objmesh.cpp`)은 vendored tinyobjloader로 파일을 읽고, shape가 2개 이상이면 즉시 거부한다.

```cpp
// objmesh.cpp:58-60
if (shapes.size() != 1) {
    CRM_ERROR("We do not support obj file with several shapes");
}
```

정점 weld는 **정렬된** `std::map`을 쓴다(선언 line 71, 루프 72-97). key는 `(vertex_index, normal_index, texcoord_index)` 3-int tuple이고, corner마다 `try_emplace`로 최초 등장 시에만 `m_vertices`에 push_back하며 인덱스를 부여한다.

```cpp
// objmesh.cpp:71-97 (발췌)
std::map<std::tuple<int, int, int>, Int> welded;
for (size_t i = 0; i + 2 < indices.size(); i += 3) {
    Int tri[3];
    for (int k = 0; k < 3; ++k) {
        const auto &idx = indices[i + k];
        const auto key = std::make_tuple(idx.vertex_index,
                                         is_vn_exists ? idx.normal_index : 0,
                                         is_tx_exists ? idx.texcoord_index : 0);
        auto [it, inserted] = welded.try_emplace(key, static_cast<Int>(m_vertices.size()));
        if (inserted) { /* push_back position/normal/uv */ }
        tri[k] = it->second;
    }
    m_face_indices.emplace_back(tri[0], tri[1], tri[2]);
}
```

PLY 로딩(`src/shapes/plymesh.cpp:44`)은 happly로 읽는데, `getVertexPositions()`가 `double`을 반환해 `Float`(=`float`, `common.h:32`)로 좁힌다. **position + normal(nx/ny/nz)만** 읽고 **texcoord는 읽지 않으며**(line 47-90), face는 fan-triangulation한다(line 93-104). PLY는 이미 indexed이므로 weld가 없다.

정정할 핵심 사실: OBJ의 float 스캔은 caramel 코드가 아니라 `ext/tiny_obj_loader.h`의 자체 파서 `tryParseDouble`(line 866, `parseReal` 경유 998)가 담당한다 — **strtod가 아니다**. 즉 블로그가 지목한 pbrt의 strtod 핫스팟은 caramel엔 그대로 존재하지 않는다.

규모 근거(이 트리의 caramel-scenes로 실측): `ajax/ajax.obj` = 50MB, `v` 272,285 / `vn` 407,050 / `f` **544,566** → weld 루프가 도는 corner는 3×544,566 ≈ **163만 회**. `stormtrooper/helmets.obj` = 25MB, `f` 340,268 → 약 102만 corner. 반면 `lego` 씬은 **binary_little_endian PLY 441개**로 구성 — 즉 "바이너리 PLY at scale"은 이미 실현돼 있고, 큰 텍스트 OBJ도 이미 존재한다.

### 문제/기회
`std::map`은 노드-per-entry 트리라 삽입·조회가 O(log M) + 포인터 추적 + 개별 할당이다. ajax는 163만 회 × log2(≈54만) ≈ 3천만 비교(캐시 비친화적). `unordered_map` + `reserve()`면 amortized O(1)이 된다. 루프가 corner 순서대로 최초 등장 시 인덱스를 부여하므로 **결과 mesh는 컨테이너 종류와 무관하게 bit-identical**(정점 순서·인덱스 동일) — 진짜 drop-in이다.

단, 정직하게: 50MB ajax에서는 tinyobjloader 파싱(50MB 텍스트 읽기 + 수백만 토큰 `tryParseDouble` + `attrib` 배열 구성)이 weld보다 지배적일 개연성이 크다. weld는 여러 후보 중 하나일 뿐이며, **어느 쪽이 핫스팟인지는 프로파일 없이 단정 불가** — 이것이 바로 블로그의 교훈이다.

검증된 인접 낭비(측정-먼저 교훈의 실제 예): `ImageEnvLight`가 `get_data_for_sampling(true)`를 생성자(`imageEnvLight.cpp:48`, `build_sampling_distrib`)와 `power()`(`imageEnvLight.cpp:101`)에서 **두 번** 전체 W×H luminance를 재계산한다. 캐싱만으로 제거되는 실질 낭비.

### 블로그 기법
part 1 실측: 70GB 씬을 GCE(120GB/32스레드)에서 파싱하는 데 34m58s, 그중 절반 이상이 `yyparse`/`yylex`, `yylex`의 절반 이상이 `strtod`. 지오메트리를 바이너리 PLY로 변환하니 디스크 70GB→22GB, 파싱 27m35s(1.3x), PLY 파싱만 40s. 처리량은 **PLY 130MB/s vs pbrt 텍스트 16.5MB/s(약 8x)**. 프로파일링은 `perf --call-graph dwarf -F 100`. strtod 교체와 weld 해시맵 논의는 **part 3로 미뤄졌고**, part 1 본문의 실측 이득은 "텍스트→바이너리 PLY 변환"이다. 핵심 메타교훈: 핫스팟을 단정하기 전에 프로파일러로 측정하라.

### caramel 적용 설계 (미적용)
**(B) weld 컨테이너 교체 — objmesh.cpp만, 약 10줄, 저위험.** `std::tuple<int,int,int>`용 hash가 std에 없으므로 functor를 제공한다.

```cpp
struct TripleHash {
    size_t operator()(const std::tuple<int,int,int>& t) const noexcept {
        auto mix = [](size_t h, int v){ return h ^ (std::hash<int>{}(v)
                    + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2)); };
        size_t h = 0;
        h = mix(h, std::get<0>(t)); h = mix(h, std::get<1>(t)); h = mix(h, std::get<2>(t));
        return h;
    }
};
// std::map<std::tuple<int,int,int>, Int> welded;
std::unordered_map<std::tuple<int,int,int>, Int, TripleHash> welded;
welded.reserve(indices.size());   // rehash 방지
```

`try_emplace`/`it->second` 인터페이스가 동일해 루프 본문(72-97)은 무변경.

**(A) OBJ→binary-PLY 오프라인 변환기.** vendored happly가 이미 binary 쓰기를 지원한다(`DataFormat::Binary`, `happly.h:68`; `write(path, format)` 1361/1381). tinyobjloader로 읽고 weld한 뒤 happly로 저장하는 소형 CLI:
```cpp
void convert_obj_to_binary_ply(const fs::path& in_obj, const fs::path& out_ply);
```
이후 씬 JSON의 `path`를 `.obj`→`.ply`로 교체(생성 지점 `scene_parser.cpp:229`→`:239`). 전제: UV가 필요하면 먼저 `PLYMesh`가 texcoord를 읽도록 확장해야 함.

**(C) 프리(free) 정리.** `build_sampling_distrib`가 만든 `data`를 멤버로 캐시하거나 `power()`에 전달해 두 번째 빌드(line 101) 제거.

순서: 먼저 측정(perf 또는 macOS Instruments) → weld가 뜨면 (B), 파싱이 뜨면 (A), (C)는 독립적 즉시 이득.

### 리스크·주의·트레이드오프
- hash 품질이 나쁘면 map보다 느려질 수 있음 — 위 boost식 mix로 충분.
- PLY 리더 UV 공백: OBJ→PLY 변환은 현재 리더에선 UV를 소실(ajax는 `vt`=1, helmets는 0이라 안전하지만 일반 OBJ는 아님). happly double→float 좁힘·fan-triangulation·weld 부재도 유의.
- weld 교체는 **OBJ 경로만** 개선, indexed PLY 441개엔 무효. 전부 load-time, render 성능·결과 영향 0.
- "바이너리 선호"는 자산 파이프라인 변경 — 변환기는 1회성이나 씬 재작성 + `.obj` 원본 보존 필요.
- 최대 함정: weld를 핫스팟으로 가정하지 말 것(=블로그 요지). ajax에선 파싱이 지배적일 수 있음.

### 테스트 전략
현재 OBJ/PLY 파일-로드 유닛 테스트가 없다(`unit_tests.cpp:1829-`는 `InlineTriangleMesh`만). 이 패턴을 미러해 소형 `.obj`를 로드하고 `get_triangle_num()`·정점 수(dedup 결과)를 assert하는 `TEST_CASE` 추가. weld 교체는 bit-identical이 목표이므로 (a) 골든 정점/면 카운트 테스트 + (b) 기존 `simple_render_test`/`complex_render_test`의 avg 비교가 그대로 통과 = 회귀 그물. 변환기는 round-trip 테스트(OBJ 로드→binary PLY 저장→PLY 로드→정점 위치 tol 내 일치 + 면 수 동일). 측정은 생성자를 timing/`os_signpost`로 감싸(macOS Instruments 스킬 활용) weld vs parse 비중을 전/후로 보고.

### 노력·효과 재평가
- weld unordered_map: 노력 **낮음**/리스크 **낮음**(drop-in, 결과 동일). 효과 **낮음~중간**, load-time·OBJ 한정, 게다가 실제 핫스팟이 아닐 수 있어 **프로파일 게이팅** 필수.
- PLY 변환기: 노력 **중간**, 효과 **중간**(50MB→수 MB, weld·토큰화 자체를 건너뜀)이나 UV 소실·자산 재작성이 블로커.
- 실질 최고 ROI는 (C) 측정-먼저 + `get_data_for_sampling` 이중 빌드 제거(공짜). prior pass의 "몇 줄 win"은 weld에 대해선 정확하나 이득의 확실성을 과장 — 측정 전에는 미지수다.

## A3. 씬 전처리 가속: BVH arena 할당자 + 메시별 병렬 빌드 + content-hash dedup
**노력**: Medium · **효과**: Low~Medium (load-time only) · **출처**: [Swallowing the Elephant (Part 9): We Got Instances](https://pharr.org/matt/blog/2021/07/27/moana-gpu-instances.html)

### 현재 상태

BVH 빌드는 노드마다 fresh `std::vector`와 heap 노드를 만든다. `BVHNode::create_child`는 in-place `std::partition`(bvh_base.cpp:124-129) 뒤에 좌/우를 새 vector로 복제한다:

```cpp
// src/bvh_base.cpp:135-143
std::vector<Primitive> left(mid - m_primitives.begin());
std::vector<Primitive> right(m_primitives.end() - mid);
std::move(m_primitives.begin(), mid, left.begin());
std::move(mid, m_primitives.end(), right.begin());
m_primitives.clear();
m_left  = std::make_unique<BVHNode>(std::move(left), traits);
m_right = std::make_unique<BVHNode>(std::move(right), traits);
```

이 pointer-tree는 순수 build-time scaffolding이다. `BVHTree` 생성자(bvh_base.cpp:182-185)에서 `root`는 stack local이고, `flatten_recursive`가 `m_nodes`(`LinearBVHNode`)와 `m_ordered_primitives`로 옮긴 직후 트리 전체가 파괴된다 — runtime엔 flat array만 남는다. per-mesh 빌드는 `subspace_count=32, max_primitive_num=1`(triangle_mesh.cpp:70)이고 `create_child`가 `size()<=2`에서 early-return(bvh_base.cpp:62)하므로, T개 삼각형 메시는 약 2T개 노드 → 약 2T번의 `make_unique` + 내부노드마다 vector 2개를 만들고 전부 즉시 해제한다.

메시별 BVH는 `TriangleMesh::finalize`에서 만들어진다:

```cpp
// src/shapes/triangle_mesh.cpp:70-71
m_accel = std::make_unique<BVHMesh>(*this, Float1, Float1, 32, 1);
m_accel->build();
```

`finalize`는 `OBJMesh`/`PLYMesh` 생성자(objmesh.cpp:99, plymesh.cpp:106)에서 호출되고, 그 생성자는 `SceneParser::parse_shapes`의 직렬 루프(scene_parser.cpp:150-158)에서 순차 실행된다. 즉 파일 파싱 + welding + area/pdf 계산(triangle_mesh.cpp:53-68) + per-mesh BVH가 메시마다 한 스레드에서 직렬로 돈다. 반면 `Scene::build_accel`(scene.cpp:100-103)은 전체 shape에 대한 top-level 트리 **하나**만 만든다(`BVHScene::build`, bvh_scene.cpp:41-43).

dedup은 없다. `parse_shape`(scene_parser.cpp:228-247)는 obj/ply 항목마다 새 mesh를 만들고 path/content 캐시가 없다. 게다가 로더는 to_world를 vertex에 굽는다(objmesh.cpp:82-83, plymesh.cpp:67-74). `parallel_for`는 이름과 달리 persistent pool이 아니라 호출마다 `std::thread`를 새로 spawn/join하며(parallel_for.h:68-89), `octree.cpp:80-84`는 중첩 oversubscribe를 피하려 `depth==0`에서만 병렬화한다.

### 문제/기회

전부 **load-time 전용**이며 per-sample 시간·화질과 무관하다. GUI(application.cpp:243)는 씬 전환마다 `build_scene`→직렬 `parse_shapes`+`build_accel`을 재실행하므로 대형 씬(complex_render_test의 dragon/house/attic/stormtrooper)에서 전환 지연이 그대로 노출된다. (1) allocation churn: 대형 메시일수록 2T번의 malloc 왕복이 build 시간을 좀먹는다. (2) 직렬성: N개 메시의 파싱·빌드가 순차라 멀티코어가 idle. (3) 중복: 같은 파일 K번 로드 = K배 파싱·빌드·메모리.

### 블로그 기법

Pharr의 Moana/pbrt-v4 GPU 시리즈 Part 9(312 instance definitions)에서 이전 가능한 세 기법:

1. **BufferCache (content-hash dedup)**: vertex/index buffer를 hash table로 중복 제거해 각 buffer를 한 번만 저장, Moana에서 **4.9 GB** 절감. 동시성 최적화 3종: (a) hash를 lock 밖에서 계산, (b) reader-writer lock, (c) hash table을 **64 shard**로 쪼개 각 shard가 own mutex.
2. **Per-thread slab**: 각 스레드가 main allocator에서 **1 MB chunk**를 주기적으로 받고 그 안에선 mutex 없이 자체 할당 → malloc mutex 경합 제거.
3. **Outermost loop 병렬화**: 독립적인 instance 빌드 루프를 병렬로.

포스트가 보고한 타임라인: 72.2s → (outermost loop) 68.5s → (per-thread slab) 64.9s → (BufferCache 동시성) 59.6s, non-instanced geometry phase 4.9s→3.0s. 핵심 통찰은 **병렬화 이전에 allocator를 per-thread로 바꾸지 않으면 malloc mutex가 병목**이라는 것 — 단일 공유 arena는 lock이 필요해 경합을 재도입한다.

### caramel 적용 설계

세 단계, 독립 배포 가능. 권장 순서 (1)→(2)→(3).

**(1) BVH build arena (bvh_base.cpp 국소).** 이미 in-place `std::partition`이 있으므로 노드를 `[begin,end)` index range로 재귀시키고, 노드/vector를 per-build monotonic buffer에서 할당한다:

```cpp
// std::pmr 기반 최소 침습안
std::pmr::monotonic_buffer_resource arena;      // BVHTree ctor 소유, build 종료 시 통째 release
// create_child(...): 135-138의 fresh vector를 std::pmr::vector<Primitive>{&arena}로,
// 142-143의 make_unique<BVHNode>를 arena에서 bump-alloc한 BVHNode*로 대체
```

**(2) 메시별 병렬 (parse_shapes 루프, scene_parser.cpp:150-158).** build_accel이 아님에 주의. 2-phase로: 먼저 JSON 항목을 평탄화(instance 확장 포함)해 개수를 확정하고 `out`을 pre-resize한 뒤 index 대입:

```cpp
std::vector<Json> flat = flatten_entries(child);   // 순서 보존
out.resize(flat.size());
parallel_for(0, (int)flat.size(), [&](int i){ out[i] = parse_shape(flat[i]); });
```

전제조건: (a) **Logger에 static mutex** 추가(logger.h). (b) **중첩 금지** — 각 메시의 `BVHMesh::build`를 동시에 병렬화하면 안 됨(parallel_for가 진짜 pool이 아니라 oversubscribe); (1)의 arena는 task-local/thread_local이어야 함. (c) `m_bsdf_map`은 `parse_bsdfs_map` 선행 후 read-only라 concurrent read 안전. (d) cwd는 build_scene 시작 시 1회 설정(render.cpp:38)되고 병렬 구간 중 불변 → 상대경로 안전.

**(3) content-hash dedup / auto-instancing.** transform-baking 때문에 raw-buffer hash는 '동일 파일+동일 transform'만 잡는다. 로더를 local-space 파싱으로 바꾸고 MeshCache를 두어 재사용분을 기존 Instance로 라우팅:

```cpp
class MeshCache {                       // key = content hash(file bytes) or canonical path+mtime
    std::unordered_map<Hash, const TriangleMesh*> m_by_content;
    const TriangleMesh* get_or_load(const std::filesystem::path&, Loader);
};
// parse_shape(obj/ply): auto* tmpl = cache.get_or_load(path);
//                       return new Instance(tmpl, to_world, bsdf, al);
```

Instance 인프라(shape.h:190-216, instance.cpp)는 이미 world-space sample/pdf/area가 정확하고 `geometry()` accessor(shape.h:206)도 있다.

### 리스크·주의·트레이드오프

LOAD-TIME ONLY — 화질/샘플당 비용에 0 영향. cbox급 현행 씬은 로드가 sub-second라 체감 이득 거의 없고, 대형/instance-heavy 씬이나 GUI 반복 전환에서만 의미 있다. 단일 공유 arena는 병렬에서 lock을 재도입하므로 per-thread 필수(블로그 핵심). `parallel_for`가 매번 thread를 생성하므로 메시 수가 적으면 생성비용이 이득을 잠식한다. Logger 비스레드안전은 hard prerequisite(안 고치면 로그 garble + `std::localtime` UB). (3)은 로더 transform 리팩터가 커서 노력 대비 현재 효과가 가장 낮다. 또 mesh accel을 Octree(octree.cpp:81 이미 nested parallel_for)로 바꾼 채 메시-병렬까지 걸면 3중 oversubscribe.

### 테스트 전략

- **회귀 guard**: 기존 `[RenderTest]`(complex_render_test.cpp의 dragon/house/attic/stormtrooper; simple_render_test의 avg-ratio 비교)가 그대로 대형 씬 벤치 겸 정합성 확인. arena/병렬/dedup 후 이미지가 임계값 내 동일해야 한다.
- **BVH 동치성 unit test**(`[UnitTest]` 스타일): 같은 메시를 old vs arena 빌드해 `m_nodes`/`m_ordered_primitives` 비트동일 + 무작위 ray 교차 결과 동일.
- **병렬 결정성**: index 대입이라 `Scene::m_meshes` 순서·내용이 병렬 전/후 동일해야 함.
- **dedup**: `build_scene` 후 동일 파일을 참조한 두 shape의 `Instance::geometry()` 포인터가 같음을 `CHECK`(shape.h:206 재사용); 옵션으로 peak RSS/mesh alloc 카운트 비교.
- **스레드 안전**: parse_shapes 병렬 구간을 ThreadSanitizer로 빌드·실행.

### 노력·효과 재평가

**노력 Medium** (prior와 동일하나 분해): arena(bvh_base 국소, small-medium), 병렬(전제조건 3개로 medium), dedup/auto-instancing(로더 transform 리팩터로 medium-high). 셋 다면 medium-high. **효과 Low~Medium** (prior "medium"에서 하향): load-time only이고 현행 씬 규모에선 무시가능, 대형/instance-heavy 씬·GUI 반복 로드에서만 medium이며 transform-baking이 dedup 상단을 제한한다. → 위험 낮고 이득 확실한 **arena+병렬을 먼저**, dedup(auto-instancing)은 대형 씬 도입 시로 유보 권장.

## A4. sampler [0, 1-2^-24] 불변식 문서화·assert
**노력**: 매우 낮음 · **효과**: 낮음 (방어적·미래 대비) · **출처**: [Sampling in Floating Point (1/3): The Unit Interval](https://pharr.org/matt/blog/2022/03/05/sampling-fp-unit-interval.html)

### 현재 상태

샘플러는 PCG32 32비트 출력의 상위 24비트를 취해 `2^-24`를 곱한다 (`src/samplers/uniformstd.cpp:56-60`):

```cpp
Float UniformStdSampler::sample_1d() {
    // Convert to [0, 1) range
    // Use upper 24 bits for float (24-bit mantissa)
    return static_cast<Float>(next_uint32() >> 8) * static_cast<Float>(0x1.0p-24);
}
```

`Float`은 `common.h:32`에서 `float`(IEEE754 binary32)이다. `next_uint32() >> 8`은 `[0, 2^24-1]`의 정수이며 float에 정확히 표현되고, `0x1.0p-24`(= `2^-24`) 곱은 지수 shift라 반올림이 없다. 따라서 출력은 `2^-24` 간격의 정확한 배수 `k * 2^-24` (k ∈ [0, 2^24-1])로, 닫힌구간 **[0, 1-2^-24]** 이다. 최댓값은 `(2^24-1)*2^-24 = 0x3F7FFFFF = 0x1.fffffep-1f ≈ 0.99999994`로 1.0 바로 아래 float이며 **절대 1.0을 반환하지 못한다**(직접 계산으로 확인).

이 값을 소비하는 이산 분포 선택기 `Distrib1D::sample` (`include/distribution.h:73-76`):

```cpp
Index sample(Float x) const{
    const auto iter = std::ranges::upper_bound(m_cdf, x);   // line 74
    return iter - m_cdf.begin();                             // line 75
}
```

생성자(61-66행)에서 `m_cdf[i] /= sum`으로 정규화하므로 마지막 원소 `m_cdf.back()`은 `sum/sum == 1.0f`로 **정확히** 1.0이다.

### 문제/기회

불변식 "sample_1d ∈ [0, 1-2^-24], 결코 1.0 아님"은 명시적 문서·검사가 전혀 없지만 load-bearing이다. `x < 1.0`이면 `upper_bound`는 항상 `1.0`인 마지막 CDF를 찾아 index ∈ [0, size-1]을 돌려주지만, **`x == 1.0`이면** 1.0보다 큰 원소가 없어 `m_cdf.end()`가 반환되고 index가 `size`(범위 밖)가 된다. 이 index는 최소 3개의 독립 크래시 지점으로 흘러든다:

- **광원 선택** `src/scene.cpp:92-93` — `m_lights[idx]` (std::vector, `scene.h:59`)와 `m_lights_pdf.pdf(idx)` (`distribution.h:79`의 `m_pdf[i]`) 둘 다 OOB.
- **envmap** `src/lights/imageEnvLight.cpp:131` → `Distrib2D::sample` (`distribution.h:99-102`) → `m_height_distrib_list[w]` (101행) OOB.
- **면광원 삼각형 선택** `src/shapes/triangle_mesh.cpp:185`, `src/shapes/instance.cpp:135` → `get_triangle_sample_point(i)` → `m_face_indices[i]` (`triangle_mesh.cpp:211`) OOB.

추가로 `sample_beckmann_distrib` (`include/warp_sample.h:133`)의 `log(1-s2)`는 s2=1.0에서 `log(0) = -inf`가 되어 NaN/degenerate 방향을 만든다.

정리하면 **현재 라이브 버그는 없다**(불변식이 수학적으로 성립). 다만 누군가 샘플러를 `/2^32`로 "단순화"하거나 1.0을 낼 수 있는 QMC 샘플러를 추가하면 즉시 UB로 전환되는 잠재 지뢰다.

### 블로그 기법

Pharr의 글이 지적하는 버그는 "32비트 정수 / 2^32" 방식이다. 1.0 아래 float 간격(ulp)은 `2^-24`인데 `2^-32`는 그 절반보다 훨씬 작아, `(2^32-1)/2^32`가 반올림되어 **1.0이 되고 비포함 상한 계약이 깨진다** — 실제로 `[2^32-128, 2^32-1]`의 128개 값이 모두 1.0로 반올림된다. 또한 round-to-nearest-even이 나눗셈 뒤에 적용되어 `[0.5,1)`에서 값이 255회·257회로 번갈아 나오는 **계통적 편향**(Goualard)이 생긴다.

caramel이 쓰는 **`>>8` 후 `2^-24` 곱** 방식은 이 두 결함을 근본적으로 피한다: `2^24 = 16,777,216`개의 정확히 표현 가능한 등간격 값을 각 확률 `2^-24`로 내며 반올림 자체가 없다. 상한은 `1-2^-24 = 0x3F7FFFFF`. 즉 블로그가 권하는 "안전한 방법"을 이미 구현하고 있다.

블로그의 심화 기법(Walker 1974 / Marc Reynolds의 64비트 구현: 지수를 기하분포로, 유효숫자 23비트를 균일 샘플)은 `[0,1)`의 실제 표현 가능 float **1,065,353,216개**를 밀도에 비례해 뽑아 0 근처 해상도를 높이는 것이다. `/2^24`(16.7M) 및 `/2^32`(83.9M, 7.87%) 모두 이 전수를 못 덮지만, 이는 정확성 버그가 아니라 0 근처 정밀도/층화 이슈로 **caramel에 필수는 아니다**.

### caramel 적용 설계 (미적용 제안)

핵심은 "불변식을 코드로 못박고, 위반 시 크래시 대신 안전 폴백"이다.

1. **문서화** — `include/sampler.h`의 `virtual Float sample_1d()`와 `uniformstd.cpp:56`에 계약 주석 추가:
```cpp
// Returns a uniform value in the CLOSED interval [0, 1 - 2^-24].
// INVARIANT: never returns 1.0. Load-bearing for Distrib1D::sample
// (upper_bound -> index would be out of range) and warp_sample's log(1-s2).
```

2. **컴파일타임 상한 고정** — `uniformstd.cpp`에 constexpr 검증:
```cpp
static_assert(0x1.0p-24f * static_cast<float>((1u << 24) - 1u) < 1.0f,
              "sample_1d upper bound must stay below 1.0");
```

3. **방어적 clamp (핵심)** — `include/distribution.h:73-76`을 belt-and-suspenders로:
```cpp
Index sample(Float x) const{
    const auto iter = std::ranges::upper_bound(m_cdf, x);
    const Index idx = static_cast<Index>(iter - m_cdf.begin());
    CRM_DEBUG_ASSERT(idx < m_cdf.size());          // 테스트에서 위반 포착
    return std::min(idx, static_cast<Index>(m_cdf.size()) - 1);  // 프로덕션 안전
}
```
이 한 줄이 3개 호출부 전부를 어떤 미래 샘플러 하에서도 OOB로부터 보호한다(이미 binary search가 있는 경로라 `min` 1회는 무시할 수준). `m_cdf`가 비어있지 않다는 전제는 기존 호출부에서 성립(광원/삼각형이 있을 때만 sample 호출).

4. **디버그 assert 매크로** — 기존 `logger.h`의 `CRM_ERROR` 스타일로 `CRM_DEBUG_ASSERT`를 두어, Release에서는 사라지되 test/Debug에서는 rogue 샘플러(1.0 방출)를 즉시 잡게 한다.

### 리스크·주의·트레이드오프

- **assert 단독은 신뢰 불가**: 루트 `CMakeLists.txt`에 `CMAKE_BUILD_TYPE`/`NDEBUG` 지정이 없다. 렌더는 Release(`-DNDEBUG`)로 돌리므로 순수 `assert()`는 정작 필요한 구성에서 컴파일 아웃된다 → 반드시 `static_assert` + `clamp`(런타임 상시) 조합으로 가야 한다.
- **clamp는 진짜 버그를 은폐할 수 있다**: 미래 샘플러가 1.0을 내면 clamp가 조용히 마지막 bin으로 편향시킨다. 그래서 clamp와 debug-assert를 **함께** 둔다(테스트는 잡고, 프로덕션은 안 죽음).
- **품질 개선 아님**: 이 작업은 0 근처 해상도/coverage(블로그 심화 기법)를 전혀 개선하지 않는다. 순수 안전성·문서 변경이다.
- **선행조건 없음**. 다만 실질 가치는 QMC(Sobol/Halton) 샘플러 도입 시점에 발현된다. 최근 커밋 흐름(BVH flatten, envmap MIS, shape 추가 등 활발한 개발)을 보면 그 시점이 올 가능성이 있다.

### 테스트 전략

기존 Catch2 하니스(`test/unit_tests.cpp`, `[UnitTest]` 태그)를 그대로 재사용하며 새 의존성 불필요.

- **샘플러 경계 테스트**: `UniformStdSampler`에서 대량(예: 5천만) `sample_1d()`를 뽑아 `s >= 0.0f && s <= 0x1.fffffep-1f && s < 1.0f` 확인. 추가로 `s * 0x1.0p24f`가 정수(반올림 없음의 증거)임을 검사.
- **Distrib1D OOB 회귀 테스트**: 작은 분포를 만들어 `sample(std::nextafterf(1.0f, 0.0f))`가 `< size`임을, 그리고 (clamp 도입 후) 의도적으로 `sample(1.0f) == size-1`임을 확인 — 미래 회귀를 못박는 가드.
- **기존 chi2 프레임워크**(`test/chi2_bsdf_test.cpp`, `chi2_polygon_test.cpp`의 `hypothesis::chi2_test`)는 warp의 분포 형상은 검증하지만 1.0 경계 케이스는 못 잡는다 → 위 타깃 단위 테스트가 올바른 도구다.

### 노력·효과 재평가

- **노력: 매우 낮음** — 주석 + static_assert + clamp 한 줄 + 단위 테스트 ~30줄. 빌드·테스트 포함 1-2시간. (prior의 "low"보다 더 낮게 본다.)
- **효과: 낮음 (방어적)** — 오늘 기준 라이브 버그가 없어 즉시 이득은 없다. 그러나 (a) clamp가 잠재 크래시를 안전 결과로 전환, (b) 문서화가 `/2^32` 단순화·QMC 추가라는 **개연성 있는 회귀**를 예방, (c) NDEBUG 함정을 회피하는 올바른 안전장치 형태 확립. prior의 "low/low"에 대체로 동의하되, 활발한 개발 궤적을 감안하면 방어적 가치는 low의 상단으로 본다.

## A5. Basu-Owen 측도보존 삼각형 샘플링 (QMC sampler 추가 후에만)
**노력**: Medium (기법 함수 자체는 Low, 선행 QMC sampler가 병목) · **효과**: Low (QMC 없으면 실질 이득 0, 있어도 fallback 경로 한정) · **출처**: [Adventures in Sampling Points on Triangles (Part 1)](https://pharr.org/matt/blog/2019/02/27/triangle-sampling-1.html)

### 현재 상태
caramel의 삼각형 내부 균등 샘플링은 동일한 고전 sqrt-warp가 두 곳에 복제되어 있다. 실제 라이트 샘플링에서 도달하는 것은 `TriangleMesh::get_triangle_sample_point` 쪽이다.

```cpp
// TriangleMesh::get_triangle_sample_point — triangle_mesh.cpp:216-220
const Float u = sampler.sample_1d();
const Float v = sampler.sample_1d();
using std::sqrt;
const Float x = Float1 - sqrt(Float1 - u);
const Float y = v * sqrt(Float1 - u);
```

`triangle.cpp:65-66`에도 문자 그대로 같은 식이 있으나(`Triangle::sample_point`), 이 경로는 라이트 샘플링에서 실질적으로 도달하지 않는다(→ corrections).

호출 흐름: `PathIntegrator::mis_sampling_path`(path.cpp:91)의 emitter sampling이 `AreaLight::sample_direct_contribution`을 부른다. 그 안에서 `is_solid_angle_sampling_possible()`가 true면 Peters-2021 solid-angle 샘플링(area.cpp:62-109, `prepare/sample_solid_angle_polygon`)을 쓰고, false일 때만 `m_shape->sample_point(sampler)` fallback(area.cpp:111)으로 위 sqrt-warp에 도달한다. Instance 경로(instance.cpp:135)도 이 fallback을 통해서만 warp에 닿는다.

Sampler는 PCG32 하나뿐이다 — `UniformStdSampler final : public Sampler`(sampler.h:41)가 유일한 구현이며 Halton/Sobol/scramble/QMC는 트리 어디에도 없다. `MCIntegrator::render`(MCIntegrator.cpp:56)에서 스캔라인 행마다 seed=행 index로 새로 만들어지고, `sample_1d()`는 24비트 정밀도 uniform 값을 돌려준다(uniformstd.cpp:59: `next_uint32() >> 8 * 2^-24`).

### 문제/기회
sqrt-warp는 정사각형 [0,1)²를 삼각형으로 접으면서 필연적으로 pinch/fold를 만든다(블로그 핵심 주장). i.i.d. uniform 입력에서는 이 왜곡이 무해하다 — 결과 점은 여전히 면적당 균등이고 분산도 동일하다. 문제는 입력이 저불일치(low-discrepancy) 점열일 때다. 정성껏 균등하게 깔아둔 2D 점들이 warp를 통과하면 층화(stratification)가 깨져 clustering/gap이 생기고 QMC의 오차 감소 성질을 잃는다. 즉 caramel처럼 uniform PCG32만 쓰는 한 이 기법의 이득은 사실상 0이다 — **QMC sampler가 하드 선행조건**이다.

또한 주 라이트 경로는 이미 Peters-2021 solid-angle 샘플링이라, warp가 실제로 쓰이는 곳은 fallback 한정이다: 비평면·비볼록·경계정점 8개 초과(`MAX_POLYGON_VERTEX_COUNT=8`, triangle_mesh.cpp:150로 게이팅) mesh 이미터. 흔한 평면 사각/삼각 이미터는 전부 solid-angle 경로로 빠진다.

### 블로그 기법
Basu & Owen 측도보존 삼각형 warp는 2D가 아니라 **단일 1D 값** u∈[0,1)를 쓴다. u를 32비트 고정소수로 보고 상위부터 16개의 base-4 자리를 뽑아, 각 자리 d∈{0,1,2,3}로 삼각형을 중점 4분할(midpoint subdivision)한 4개 sub-triangle 중 하나로 재귀 하강한다. 16단계 후 최종 sub-triangle의 무게중심을 반환한다.

```cpp
std::array<Float,3> LowDiscrepancySampleTriangle(Float u) {
    uint32_t uf = u * (1ull << 32);          // fixed point
    Point2f A(1,0), B(0,1), C(0,0);          // barycentrics
    for (int i = 0; i < 16; ++i) {
        int d = (uf >> (2*(15-i))) & 0x3;    // next base-4 digit
        Point2f An,Bn,Cn;
        switch (d) {
        case 0: An=(B+C)/2; Bn=(A+C)/2; Cn=(A+B)/2; break; // center
        case 1: An=A;       Bn=(A+B)/2; Cn=(A+C)/2; break; // corner A
        case 2: An=(B+A)/2; Bn=B;       Cn=(B+C)/2; break; // corner B
        case 3: An=(C+A)/2; Bn=(C+B)/2; Cn=C;       break; // corner C
        }
        A=An; B=Bn; C=Cn;
    }
    Point2f r = (A+B+C)/3;
    return { r.x, r.y, 1 - r.x - r.y };
}
```

핵심 수치: 32비트 → 16 base-4 자리; 블로그 실험(64샘플, 균등면적 삼각형 라이트)에서 저불일치 입력 + 이 기법이 sqrt-warp 대비 **약 2.17배 낮은 분산**. 단 "점열과 매핑 둘 다가 열쇠(both the points and the mapping are key)"라고 명시 — stratified/uniform 입력으로 바꾸면 각 sub-triangle에 정확히 한 점이 가더라도 분포는 훨씬 나빠진다. 픽셀 간 구조적 상관을 없애려면 Cranley-Patterson 회전(`u += delta; if(u>1) u-=1;`, delta는 blue-noise per-pixel 오프셋)을 권장. 결과 밀도는 sqrt-warp와 같은 면적당 균등이므로 **PDF(1/area)는 불변** — drop-in으로 안전하다.

### caramel 적용 설계
(구현이 아니라 설계 스케치다.)

1. warp_sample.h에 자유함수 추가:
```cpp
// Basu & Owen 2015. Area-uniform; identical 1/area PDF to the sqrt warp.
inline Vector2f sample_triangle_basu_owen(uint32_t u); // returns (x, y); z = 1 - x - y
```

2. Sampler 인터페이스(sampler.h:36)에 raw 정수 draw 노출:
```cpp
virtual uint32_t sample_uint32() = 0;
```
PCG32는 이미 `next_uint32()`(uniformstd.cpp:43, 현재 private)를 가지므로 override로 승격만 하면 된다. `sample_1d()`가 24비트라 `u*2^32`로 재구성하면 하위 8비트가 0이 되어 base-4 자리가 12개(4^12 ≈ 1.6e7 sub-triangle)로 제한된다. 정확한 층화 보존을 위해 raw 32비트 경로가 바람직하다.

3. 호출부 교체(triangle_mesh.cpp:210-235). before/after:
```cpp
// before (216-220)
const Float u = sampler.sample_1d();
const Float v = sampler.sample_1d();
const Float x = Float1 - sqrt(Float1 - u);
const Float y = v * sqrt(Float1 - u);
// after
const Vector2f b = sample_triangle_basu_owen(sampler.sample_uint32());
const Float x = b[0], y = b[1];
```
`interpolate(p0,p1,p2,x,y)`와 pdf(`1/get_triangle_area(i)`)는 그대로 둔다. `Triangle::sample_point`(triangle.cpp:61-74)와 Instance/AreaLight fallback은 이 함수를 공유하므로 일괄 반영된다.

4. 진짜 이득을 내려면 선행 작업이 대부분의 노력이다: (a) Halton/Sobol(+Owen scramble) 등 Sampler 구현 추가, (b) 차원 관리 — 현 통합기는 `sample_1d`를 flat stream으로 소비하므로(카메라 지터 2D, 라이트 선택 1D, 삼각형 선택 1D, warp, RR, BSDF …) dimension index를 추적하는 sampler API/정수 인터페이스로 확장해야 한다.

### 리스크·주의·트레이드오프
- 선행 QMC 없이는 이득 0 — uniform 입력에서 Basu-Owen과 sqrt-warp는 동일 분산. 이 항목 단독 구현은 무의미하다.
- 이득이 있어도 fallback 경로 한정 — 흔한 평면 이미터는 Peters-2021이 처리한다. 비평면·대형 다각형 mesh 이미터가 많은 씬에서만 체감된다.
- centroid 반환은 sub-triangle 크기만큼 미세 편향; 12~16단계면 무시 가능하나 float에서 midpoint 반복은 반올림 누적이 있을 수 있다.
- Instance는 삼각형 선택 pdf가 별도(`m_world_triangle_pdf`, instance.cpp:135)다. warp는 삼각형 내부만 바꾸므로 무관하지만, 삼각형 선택 자체의 층화는 개선되지 않는다.
- Cranley-Patterson/scramble 없이 QMC를 그냥 꽂으면 픽셀 간 구조적(상관) 노이즈가 생긴다.

### 테스트 전략
- chi2: test/chi2_polygon_test.cpp의 하네스를 본떠 삼각형 내부 warp용 chi2 테스트를 추가한다. `sample_triangle_basu_owen`로 다수 점을 생성해 barycentric/면적 히스토그램을 균등 기대값과 비교 — 측도보존(밀도 정확성) 회귀를 잡는다.
- 단위 테스트: unit_tests.cpp의 `get_area` 검증 패턴을 재사용해 b0,b1≥0 · b0+b1≤1 경계, 극단 u(0x00000000, 0xFFFFFFFF)에서 코너로의 수렴을 확인한다.
- 층화 이득 증명은 QMC sampler 도입 후에만 가능 — 동일 spp에서 sqrt-warp vs Basu-Owen 이미지의 MSE/분산을 레퍼런스 대비 비교해 블로그의 2.17배 재현 여부를 본다. 회귀 안전장치로는 면적 균등이라 수렴값이 동일해야 하므로 렌더 이미지 수렴값 동등성을 확인한다.

### 노력·효과 재평가
기법 함수 자체는 약 30줄, Low. 그러나 가치 실현은 QMC sampler(+차원 관리)라는 별도의 Medium~High 선행 작업에 전적으로 종속되므로 통합 노력은 Medium이다. 효과는 오늘 기준 Low(uniform-only라 실질 0)이며, QMC 도입 후에도 Peters-2021이 주 경로를 이미 커버하므로 payoff는 비평면·대형 다각형 mesh 이미터 fallback으로 제한된다. 결론: 독립 항목으로는 낮은 우선순위이고, "QMC sampler 추가" 항목의 하위 작업으로 묶는 것이 타당하다. prior pass의 Medium effort / Low impact 판정은 유지한다.

# Not Applicable

## Sampling in Floating Point (2/3): 1D Intervals

caramel의 세 importance 분포(Distrib1D, Distrib2D, 삼각형 면적)가 모두 이산(버킷 인덱스 반환, envmap 샘플을 픽셀 중심 배치) → canonical uniform을 연속 float 서브구간에 lerp하는 이 글의 주제를 안 함. Pharr 본인도 잔여 효과 무시 가능이라 평가. envmap을 픽셀 내 연속 샘플링으로 업그레이드할 때만 해당.

**출처**: <https://pharr.org/matt/blog/2022/03/14/sampling-float-intervals.html>

## Let's Stop Calling it "GGX"

NDF 공식·Smith masking 유도 없는 순수 용어/저작권 글 → Microfacet(Beckmann + G1 근사)·smooth-only Dielectric gap을 못 채움(그 수학은 Walter et al. 2007 / Heitz 2014). caramel 관련 takeaway는 미용적: 미래 분포를 "GGX" 대신 "TrowbridgeReitz"로 명명.

**출처**: <https://pharr.org/matt/blog/2022/05/06/trowbridge-reitz.html>
