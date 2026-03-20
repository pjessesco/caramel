# Practical Path Guiding (Muller et al. 2017) - 상세 해설 및 구현 계획

## 1. 논문 핵심 아이디어

### 1.1 풀고자 하는 문제

Path tracing에서 렌더링 방정식의 반사 적분(reflection integral)을 Monte Carlo로 추정할 때,
나가는 radiance $L_o(\mathbf{x}, \hat{\omega}_o)$는 다음과 같다 (Eq. 1):

$$L_o(\mathbf{x}, \hat{\omega}_o) = L_e(\mathbf{x}, \hat{\omega}_o) + \int_{\Omega} L(\mathbf{x}, \tilde{\omega}) \, f_s(\mathbf{x}, \hat{\omega}_o, \tilde{\omega}) \, \cos\theta \, d\tilde{\omega}$$

반사 적분 $L_r$을 $N$개 샘플의 Monte Carlo 추정량(Eq. 2)으로 계산한다:

$$\langle L_r \rangle = \frac{1}{N} \sum_{j=1}^{N} \frac{L(\mathbf{x}, \tilde{\omega}_j) \, f_s(\mathbf{x}, \hat{\omega}_o, \tilde{\omega}_j) \, \cos\theta_j}{p(\tilde{\omega}_j \,|\, \mathbf{x}, \hat{\omega}_o)}$$

추정량의 분산 $V[\langle L_r \rangle]$은 $1/N$에 비례하며,
sampling PDF $p(\tilde{\omega})$가 피적분함수(numerator)의 형태와 비슷할수록 줄어든다.
만약 PDF가 numerator에 정확히 비례한다면 $V = 0$이 된다.

BSDF $f_s$는 이미 importance sampling이 잘 되지만, **입사 radiance** $L(\mathbf{x}, \tilde{\omega})$는
장면(scene)에 의존하는 미지의 함수이므로 사전에 알 수 없다.

**Path guiding의 목표**: 렌더링 도중에 $L(\mathbf{x}, \tilde{\omega})$의 근사 분포 $\hat{L}$을 반복적으로 학습하여,
높은 에너지의 입사 방향을 효율적으로 sampling하는 것.

### 1.2 핵심 접근법: SD-Tree

5D light field $L(\mathbf{x}, \tilde{\omega})$ (3D 공간 $\mathbf{x}$ + 2D 방향 $\tilde{\omega}$)를
**SD-tree** (Spatio-Directional tree)로 근사한다.

```
SD-tree 구조:
  [Spatial Binary Tree]     -- 3D 공간을 이진 분할 (x, y, z 축 교대)
        |
        v
  각 leaf마다 하나의:
  [Directional Quadtree]    -- 2D 방향 도메인을 4분할 (quadtree)
```

- **Spatial Binary Tree**: 장면의 AABB를 $x, y, z$ 축 교대로 절반씩 이진 분할.
  샘플이 많이 모이는 공간은 더 세밀하게 분할된다.
- **Directional Quadtree**: 각 공간 leaf에서 전체 구(sphere)의 방향을 $[0,1]^2$로 매핑한 뒤,
  에너지가 많은 방향 영역을 더 세밀하게 분할.
  각 leaf는 해당 방향 영역의 **piecewise-constant** irradiance를 저장한다.

### 1.3 왜 이 구조인가?

| 대안 | 단점 |
|------|------|
| Regular histogram (Jensen95) | 해상도 고정, 적응적이지 않음 |
| Gaussian Mixture Model (Vorba14) | EM 수렴 불안정, 메모리 많이 사용 |
| KD-tree (Gassenbauer09) | 구조 복잡, 분기 계수(branching factor) 높음 |

**Quadtree의 장점**:
- 에너지 기반으로 적응적으로 분할 → 밝은 방향은 고해상도, 어두운 방향은 저해상도
- Top-down 분할이라 안정적 (GMM처럼 local optimum에 빠지지 않음)
- 구현이 단순하고 메모리 효율적 (노드당 ~5 bytes)

---

## 2. SD-Tree 상세 구조

### 2.1 방향 도메인의 Cylindrical 좌표 매핑

전체 구(full sphere)를 $[0,1]^2$ 사각형으로 매핑한다. **Cylindrical (equal-area) 좌표**를 사용:

**순방향 변환** (3D 방향 → 2D canonical):

$$u = \frac{1 + \cos\theta}{2} = \frac{1 + d_z}{2} \quad \in [0, 1]$$

$$v = \frac{\text{atan2}(d_y, d_x)}{2\pi} \quad \in [0, 1)$$

(음수면 $+1$)

**역변환** (2D canonical → 3D 방향):

$$\cos\theta = 2u - 1, \quad \phi = 2\pi v$$

$$\sin\theta = \sqrt{1 - \cos^2\theta}$$

$$\mathbf{d} = (\sin\theta \cos\phi, \;\sin\theta \sin\phi, \;\cos\theta)$$

**이 매핑의 핵심 성질**: 면적 보존(equal-area). $[0,1]^2$ 위의 균일 분포가
구 위의 균일 분포에 대응한다. 따라서 quadtree의 면적 비율 = 입체각(solid angle) 비율.

**Jacobian 유도**:

$$d\omega = \sin\theta \, d\theta \, d\phi = d(\cos\theta) \, d\phi = 2 \, du \cdot 2\pi \, dv = 4\pi \, du \, dv$$

따라서 $[0,1]^2$ 위의 PDF $p_{uv}(u,v)$와 solid angle PDF $p_\omega(\omega)$의 관계:

$$\boxed{p_\omega(\omega) = \frac{p_{uv}(u,v)}{4\pi}}$$

### 2.2 Directional Quadtree 노드 구조

```cpp
struct DQuadTreeNode {
    Float sum[4];           // 4개 자식(또는 사분면)의 irradiance 합
    uint16_t children[4];   // 자식 노드 인덱스 (0이면 leaf)
};
```

모든 노드는 하나의 배열(`vector<DQuadTreeNode>`)에 저장되고, `children[i]`는
해당 배열의 인덱스를 가리킨다. `children[i] == 0`이면 그 사분면은 더 이상 분할되지 않은 leaf.

#### 사분면 배치

$[0,1]^2$ 정사각형을 4등분:
```
  (0,1) -------- (1,1)
    |   2  |  3   |
    |------|------|    child index:
    |   0  |  1   |      0 = 하단-좌측, 1 = 하단-우측
  (0,0) -------- (1,0)   2 = 상단-좌측, 3 = 상단-우측
```

점 $(u,v)$가 속하는 사분면을 구하는 법:
```cpp
int childIndex(float &u, float &v) {
    int idx = 0;
    if (u >= 0.5) { idx |= 1; u = 2*u - 1; }  // 우측
    else          { u = 2*u; }
    if (v >= 0.5) { idx |= 2; v = 2*v - 1; }  // 상단
    else          { v = 2*v; }
    return idx;
}
```
좌표를 $[0,1]$로 재정규화(rescale)하면서 내려간다.

### 2.3 Quadtree 핵심 연산

#### (A) Irradiance 기록 (Record/Splat)

path vertex에서 관측한 radiance를 quadtree에 기록:

```
record(u, v, irradiance):
    node = root
    while true:
        childIdx = childIndex(u, v)   // u, v가 rescale됨
        node.sum[childIdx] += irradiance
        if node.children[childIdx] == 0:  // leaf
            break
        node = nodes[node.children[childIdx]]
```

루트에서 leaf까지 경로 상의 **모든 노드**에 기록한다 (부모도 합산).
이렇게 하면 build 시 bottom-up 합산을 생략할 수 있지만,
논문 구현에서는 기록은 leaf에만 하고 `build()`에서 합산하는 방식도 쓴다.

#### (B) PDF 계산

주어진 방향 $(u,v)$에 대한 $[0,1]^2$ 도메인 위의 PDF:

```
pdf(u, v):
    node = root
    result = 1.0
    while true:
        childIdx = childIndex(u, v)
        total = sum(node.sum[0..3])
        if total <= 0: return 0
        result *= 4 * node.sum[childIdx] / total
        if node.children[childIdx] == 0:  // leaf
            break
        node = nodes[node.children[childIdx]]
    return result
```

**factor 4 설명**: 각 사분면은 부모 영역의 $1/4$을 차지한다.
사분면의 에너지 비율이 $\frac{\text{sum}[i]}{\text{total}}$이고, 영역이 $\frac{1}{4}$이므로,
PDF 밀도는 $4$배가 된다:

$$p_\text{child}(u,v) = \frac{\text{sum}[i]}{\text{total}} \cdot \frac{1}{1/4} = 4 \cdot \frac{\text{sum}[i]}{\text{total}}$$

최종 solid angle PDF:

$$p_\omega(\omega) = \frac{p_{uv}(u,v)}{4\pi}$$

#### (C) 방향 Sampling (Hierarchical Sample Warping)

McCool and Harwood (1997)의 hierarchical sample warping 기법.
입력 난수 $(u,v) \in [0,1)^2$을 에너지 분포에 비례하여 warp한다.

```
sample(rng):
    u = rng.sample_1d()   // [0, 1) 난수
    v = rng.sample_1d()   // [0, 1) 난수
    node = root
    while true:
        // 4개 사분면의 에너지를 가중치로 사용
        top = node.sum[2] + node.sum[3]      // 상단 행
        bottom = node.sum[0] + node.sum[1]    // 하단 행
        total = top + bottom

        // v 축 (세로) 선택: bottom vs top
        if v < bottom / total:
            v = v * total / bottom            // rescale to [0,1)
            left = node.sum[0]
            right = node.sum[1]
            row_total = bottom
            // v 좌표: 하단 → 오프셋 없음
        else:
            v = (v - bottom/total) * total / top
            left = node.sum[2]
            right = node.sum[3]
            row_total = top
            // v 좌표: 상단 → 0.5 오프셋

        // u 축 (가로) 선택: left vs right
        if u < left / row_total:
            u = u * row_total / left
            childIdx = (좌측)
        else:
            u = (u - left/row_total) * row_total / right
            childIdx = (우측)
            // u 좌표: 우측 → 0.5 오프셋

        if node.children[childIdx] == 0:  // leaf
            break
        node = nodes[node.children[childIdx]]

    return (최종 u, 최종 v)
```

**핵심 원리**: 에너지가 높은 leaf일수록 입력 $[0,1)^2$에서 더 많은 영역을 차지하므로
더 자주 선택된다. 이것이 importance sampling을 구현하는 방법이다.

### 2.4 Spatial Binary Tree

```cpp
struct SBinaryTreeNode {
    uint32_t children[2];    // 자식 노드 인덱스 (0이면 leaf → DTree 보유)
    int axis;                // 분할 축 (0=x, 1=y, 2=z, 순환)
    // leaf인 경우:
    DTree *dTree;            // 이 공간 영역의 방향 분포
    int sampleCount;         // 이 영역에 기록된 path vertex 수
};
```

#### 공간 탐색 (Spatial Lookup)

점 $\mathbf{x}$가 속하는 leaf 찾기:

```
lookup(x):
    node = root
    p = scene_aabb.offset(x)       // x를 AABB 기준 [0,1]^3으로 정규화
    while !node.isLeaf():
        axis = node.axis
        if p[axis] < 0.5:
            p[axis] *= 2           // 좌측 절반 → [0,1]로 rescale
            node = nodes[node.children[0]]
        else:
            p[axis] = 2*p[axis]-1  // 우측 절반 → [0,1]로 rescale
            node = nodes[node.children[1]]
    return node.dTree
```

#### 공간 분할 기준

iteration $k$에서 leaf node를 분할하는 조건:

$$\text{sampleCount} \geq c \cdot \sqrt{2^k}$$

여기서:
- $c = 12000$ (논문의 기본값)
- $2^k$ = iteration $k$에서의 총 path 수에 비례
- $\sqrt{2^k}$ = $k$번째 iteration의 분할 threshold

**의미**: iteration이 진행될수록 threshold가 높아져서,
spatial leaf 수와 leaf당 샘플 수가 같은 비율($\sqrt{2^k}$)로 증가한다.

**$c$의 유도** (Section 5.3):
- 목표: quadtree leaf당 약 $s = 40$개 샘플
- 평균 quadtree leaf 수 $N_l \approx 300$
- 첫 iteration ($k=0$)에서 분할이 시작되도록:

$$c = \frac{s \cdot N_l}{\sqrt{2^k}}\bigg|_{k=0} = \frac{40 \times 300}{1} = 12000$$

### 2.5 Quadtree 적응적 분할 (Adaptive Refinement)

매 iteration이 끝날 때, 수집된 에너지 분포에 기반하여 quadtree 구조를 재구축:

```
refine_quadtree(old_tree, rho=0.01):
    new_tree = 빈 quadtree (root만)
    Phi = old_tree.root의 총 에너지 (total flux)

    재귀적으로 old_tree를 순회:
        for each node:
            Phi_n = 이 노드의 에너지 (flux)

            if Phi_n / Phi > rho  AND  depth < max_depth:
                subdivide(node)
                각 자식에 부모 에너지의 1/4 할당
                재귀적으로 자식들도 같은 기준 적용
            else:
                leaf로 유지 (또는 prune)
```

분할 조건: 노드의 에너지 분율 $\frac{\Phi_n}{\Phi}$이 threshold $\rho$보다 클 때.

$$\frac{\Phi_n}{\Phi} > \rho \quad \Longrightarrow \quad \text{subdivide}$$

- $\rho = 0.01$ → quadtree leaf 수 $\leq 1/\rho = 100$ (이론적). 실제로는 ~300.
- $\text{max\_depth} = 20$으로 제한하여 메모리 폭발 방지.
- **결과**: 대략 equi-energy partitioning. 에너지가 높은 구형 영역이 더 높은 해상도로 표현됨.

---

## 3. 반복적 학습 알고리즘 (Iterative Training)

### 3.1 전체 흐름

항상 **두 개의 SD-tree**를 유지:
- $\hat{L}^{k-1}$ (sampling용): 이전 iteration에서 학습한 분포. 방향 sampling에 사용.
- $\hat{L}^k$ (collecting용): 현재 iteration에서 radiance 추정치를 수집.

```
알고리즘:
  k = 1 (첫 iteration)

  while 예산(budget) 남음:
      // 1. 현재 iteration의 path 수: B_k ∝ 2^(k-1)  (매 iteration마다 2배)

      // 2. Path tracing (B_k paths)
      for each path:
          if k == 1:
              방향 sampling = BSDF만 사용
          else:
              방향 sampling = MIS(BSDF, L_hat^{k-1})

          path를 추적, 각 vertex의 radiance 추정

          // 3. Radiance splatting
          for each vertex v in path:
              spatial_leaf = binary_tree.lookup(x_v)
              (u, v) = dirToCanonical(omega_v)
              spatial_leaf.dTree.record(u, v, radiance_estimate)

      // 4. Iteration 마무리
      L_hat^k.build()            // bottom-up 합산
      L_hat^{k-1} = L_hat^k     // sampling tree 교체
      L_hat^{k+1} = empty       // 새 collecting tree (구조는 L_hat^k에서 refine)

      // 5. 공간/방향 트리 구조 갱신
      binary_tree.refine()
      각 leaf의 quadtree.refine()

      k += 1
```

### 3.2 BSDF와 Guiding 분포의 MIS

iteration $k > 1$에서, 방향 sampling은 두 전략의 혼합이다.

**혼합 PDF**:

$$p_\text{mix}(\omega) = \alpha \cdot p_\text{bsdf}(\omega) + (1 - \alpha) \cdot p_\text{guide}(\omega)$$

논문에서는 $\alpha = 0.5$ (고정).

**Sampling 절차**:
```
sample_direction(bsdf, quadtree, sampler):
    if sampler.sample_1d() < alpha:
        // BSDF에서 sampling
        (dir, brdf_val, bsdf_pdf) = bsdf.sample_recursive_dir(...)
        guide_pdf = quadtree.pdf(dirToCanonical(dir)) / (4*pi)
    else:
        // Quadtree에서 sampling
        canonical = quadtree.sample(sampler)
        dir = canonicalToDir(canonical)
        guide_pdf = quadtree.pdf(canonical) / (4*pi)
        bsdf_pdf = bsdf.pdf(incoming, dir)
        brdf_val = bsdf.get_reflection(incoming, dir)

    mix_pdf = alpha * bsdf_pdf + (1-alpha) * guide_pdf
    return (dir, brdf_val * cos_theta / mix_pdf)
```

**주의**: specular BSDF (mirror, dielectric 등 `is_discrete() == true`)는
guiding하지 않고 BSDF sampling만 사용. guiding은 continuous BSDF에만 적용.

### 3.3 Radiance Splatting 상세

path의 각 vertex에서 **입사 radiance의 MC 추정치**를 기록한다.

path: $\text{camera} \to v_1 \to v_2 \to \cdots \to v_n$

각 vertex $v_i$에서 기록하는 값:

$$\hat{L}(\mathbf{x}_i, \tilde{\omega}_i) = \text{이 vertex에서 } \tilde{\omega}_i \text{ 방향으로 들어오는 radiance의 MC 추정치}$$

후방 누적(backward accumulation)으로 계산:

```
radiance[n] = 0  (또는 light에 도달했으면 L_e)
for i = n-1 down to 1:
    radiance[i] = emission[i] + radiance[i+1] * throughput_factor[i+1]
```

각 vertex에서의 기록:
- **direction**: $\tilde{\omega}_i$ (입사 방향, world space)
- **irradiance**: $\text{luminance}(\text{radiance}[i])$

### 3.4 Unbiasedness (비편향성) 보장

**핵심**: 각 iteration은 독립적이다. iteration $k$의 이미지는 **오직** iteration $k$의 path들로만 구성.
이전 iteration의 path은 학습(guiding 분포 구축)에만 사용되고, 최종 이미지에는 포함되지 않는다.

- iteration 1: BSDF sampling → 비편향 (standard path tracing)
- iteration $k > 1$: $\text{MIS}(\text{BSDF}, \hat{L}^{k-1})$ → 비편향 (MIS는 PDF만 바꿀 뿐 기대값 불변)

따라서 **어떤 iteration의 결과든 단독으로 비편향 이미지**이다.
최종 이미지는 마지막 iteration의 결과만 사용.

### 3.5 Exponential Sample Count (지수적 샘플 증가)

매 iteration마다 path 수를 2배로 늘리는 이유:

1. **마지막 iteration이 가장 중요**: 가장 많은 샘플 + 가장 좋은 guiding 분포
2. **공간 분할과의 조화**: binary tree leaf를 분할하면 부피가 절반 → 각 leaf의 샘플도 절반.
   하지만 총 샘플이 2배이므로 leaf당 샘플 수는 유지.
3. **Worst case에서도 절반만 낭비**: 학습이 전혀 도움이 안 되더라도
   총 예산의 절반만 학습에 소모.

### 3.6 Training-Rendering Budget 균형 (Section 3.5)

주어진 총 예산 $B$에서 학습과 렌더링의 최적 비율을 자동으로 결정.

iteration $k$의 **budget unit variance**:

$$\tau_k = V_k \cdot B_k$$

여기서 $V_k$는 iteration $k$ 이미지의 평균 pixel variance, $B_k$는 사용한 path 수.

$\hat{L}^k$로 남은 예산을 전부 렌더링할 때의 **추정 최종 분산**:

$$\hat{V}_k = \frac{\tau_k}{\hat{B}_k}$$

여기서 $\hat{B}_k$는 $k$번째 iteration 시작 시점의 남은 예산:

$$\hat{B}_k = B - \sum_{i=1}^{k-1} B_i$$

**최적 종료 조건**: $\hat{V}_{k+1} > \hat{V}_k$가 되는 가장 작은 $k$에서 학습 중단.

$$\hat{k} = \arg\min_k \hat{V}_k$$

$\hat{V}_k$의 convexity (Appendix A 증명)에 의해 이 조건은 전역 최솟값을 보장한다.

학습을 중단한 후, 남은 예산 전부를 $\hat{L}^k$로 렌더링에 투입한다.

**Target variance 기반 종료** (대안): 목표 분산 $\bar{V}$에 도달하기 위한 렌더링 예산:

$$\tilde{B}_k = \frac{\tau_k}{\bar{V}}$$

총 예산이 $\tilde{B}_k > \tilde{B}_{k-1}$가 되면 학습 중단:

$$\bar{B}_k = \tilde{B}_k + \sum_{i=1}^{k-1} B_i$$

---

## 4. Caramel 렌더러 구현 계획

### 4.1 새로 추가할 파일 구조

```
include/
    sd_tree.h           -- DQuadTree, SpatialBinaryTree, SDTree 클래스 선언

src/
    sd_tree.cpp         -- SD-tree 구현
    integrators/
        guided_path.cpp -- GuidedPathIntegrator 구현
```

### 4.2 단계별 구현 계획

#### Phase 1: Directional Quadtree (DQuadTree)

```cpp
// include/sd_tree.h

struct DQuadTreeNode {
    std::array<Float, 4> m_sum = {0, 0, 0, 0};
    std::array<uint16_t, 4> m_children = {0, 0, 0, 0};

    bool is_leaf(int child_idx) const { return m_children[child_idx] == 0; }

    // p가 속하는 사분면 인덱스 반환, p를 자식 좌표로 rescale
    static int child_index(Float &u, Float &v);
};

class DQuadTree {
public:
    // 방향 sampling: [0,1)^2 난수 → [0,1)^2 canonical coords
    Vector2f sample(Sampler &sampler) const;

    // PDF 계산: canonical coords → p_uv(u,v).  solid angle PDF = 이것 / 4pi
    Float pdf(const Vector2f &canonical) const;

    // Radiance 기록: canonical coords에 irradiance 값 추가
    void record(const Vector2f &canonical, Float irradiance);

    // Bottom-up 합산 (build 후에 sampling/pdf 가능)
    void build();

    // 이전 iteration의 에너지 분포로 구조 재구축
    void refine(const DQuadTree &prev, Float threshold, int max_depth);

    // 평균 radiance = sum / (4pi * statisticalWeight)
    Float mean() const;

private:
    std::vector<DQuadTreeNode> m_nodes;  // [0] = root
    Float m_sum = 0;                      // 총 에너지
    Float m_statistical_weight = 0;       // 총 통계 가중치
};
```

**좌표 변환 유틸리티** (sd_tree.h 또는 common.h에 추가):
```cpp
// World-space direction → [0,1]^2 canonical
inline Vector2f dir_to_canonical(const Vector3f &d) {
    Float cos_theta = d[2];
    Float phi = std::atan2(d[1], d[0]);
    if (phi < 0) phi += PI_2;
    return {(Float1 + cos_theta) * Float0_5, phi * PI_2_INV};
}

// [0,1]^2 canonical → world-space direction
inline Vector3f canonical_to_dir(const Vector2f &p) {
    Float cos_theta = Float2 * p[0] - Float1;
    Float phi = PI_2 * p[1];
    Float sin_theta = std::sqrt(Float1 - cos_theta * cos_theta);
    return {sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
}
```

**좌표계에 대한 중요 사항**: 이 좌표계는 **world space** 방향이다 (BSDF의 local 좌표가 아님).
guiding 분포는 위치 의존적이지만 법선(normal) 비의존적이다.
이것이 논문의 중요한 설계 결정 - 법선에 무관한 world-space cylindrical 좌표를 사용함으로써:
1. 같은 공간 영역의 다른 법선 방향에서도 분포를 공유할 수 있음
2. 회전 변환 불필요 → 구현 단순화
3. full sphere를 커버하므로 투과(refraction)도 자연스럽게 처리

#### Phase 2: Spatial Binary Tree

```cpp
struct SBinaryTreeNode {
    bool is_leaf = true;
    int axis = 0;                    // 0=x, 1=y, 2=z
    std::array<uint32_t, 2> children = {0, 0};

    // leaf일 때:
    DQuadTree dTree_sampling;        // sampling용 (이전 iteration)
    DQuadTree dTree_building;        // 수집용 (현재 iteration)
    int sample_count = 0;
};

class SDTree {
public:
    SDTree(const AABB &scene_aabb);

    // 점 x에 해당하는 DQuadTree (sampling용) 반환
    const DQuadTree& lookup_sampling(const Vector3f &pos) const;

    // 점 x에 해당하는 DQuadTree (building용) 반환
    DQuadTree& lookup_building(const Vector3f &pos);

    // Radiance 기록
    void record(const Vector3f &pos, const Vector3f &dir, Float irradiance);

    // Iteration 종료: build + swap + refine
    void end_iteration(int iteration);

private:
    std::vector<SBinaryTreeNode> m_nodes;
    AABB m_aabb;       // 정육면체로 확장된 scene AABB
};
```

**AABB 정육면체 확장**: 축 교대 분할이 균일하게 작동하려면 AABB가 정육면체여야 한다.

```cpp
SDTree::SDTree(const AABB &scene_aabb) {
    // 가장 긴 축 기준으로 정육면체 확장
    Vector3f center = scene_aabb.get_center();
    Float max_extent = 0;
    for (int i = 0; i < 3; i++)
        max_extent = std::max(max_extent, scene_aabb.m_max[i] - scene_aabb.m_min[i]);
    Float half = max_extent * 0.5f;
    m_aabb = AABB({center[0]-half, center[1]-half, center[2]-half},
                  {center[0]+half, center[1]+half, center[2]+half});
    m_nodes.emplace_back();
}
```

#### Phase 3: GuidedPathIntegrator

```cpp
class GuidedPathIntegrator final : public Integrator {
public:
    GuidedPathIntegrator(Index rr_depth, Index max_depth, Index total_spp);
    void pre_process(const Scene &scene) override;
    Image render(const Scene &scene) override;

private:
    Image render_iteration(const Scene &scene, Index spp_iter,
                           bool is_first, bool is_final);
    Vector3f guided_path(const Scene &scene, Float i, Float j,
                         Sampler &sampler, bool use_guiding,
                         std::vector<PathVertex> &vertices) const;

    SDTree *m_sd_tree;
    Index m_rr_depth;
    Index m_max_depth;
    Index m_total_spp;
    static constexpr Float m_bsdf_sampling_fraction = 0.5f;
};
```

**`render()` 메인 루프**:

```cpp
Image GuidedPathIntegrator::render(const Scene &scene) {
    m_sd_tree = new SDTree(scene.m_aabb);

    int iteration = 0;
    int total_spp_used = 0;
    int spp_per_iter = 1;
    Image final_image;

    while (total_spp_used < m_total_spp) {
        bool is_first = (iteration == 0);
        int remaining = m_total_spp - total_spp_used;

        // Budget balancing
        bool is_final = (spp_per_iter * 2 > remaining);
        int actual_spp = is_final ? remaining : spp_per_iter;

        final_image = render_iteration(scene, actual_spp, is_first, is_final);

        if (!is_final)
            m_sd_tree->end_iteration(iteration);

        total_spp_used += actual_spp;
        spp_per_iter *= 2;
        iteration++;

        if (is_final) break;
    }
    return final_image;
}
```

`MCIntegrator`를 상속하지 않고 `Integrator`를 직접 상속하는 이유:
- `render()` 루프가 완전히 다름 (iterative training)
- spp이 iteration마다 변함
- 매 iteration마다 이미지를 새로 만듦

**`guided_path()` - 핵심 path tracing 루프**:

기존 `PathIntegrator::mis_sampling_path()`를 확장하되 핵심 차이점:
1. 방향 sampling 시 BSDF 대신 $\text{MIS}(\text{BSDF}, \text{guiding})$ 사용
2. path의 각 vertex 정보를 저장 (나중에 splatting용)
3. specular bounce에서는 guiding 미사용

```
"brdf sampling" 블록에서:
  if use_guiding AND !is_current_specular:
      dtree = sd_tree.lookup_sampling(info.p)

      if sampler.sample_1d() < alpha:
          (local_dir, sampled_brdf, bsdf_pdf) = bsdf.sample(...)
          world_dir = info.sh_coord.to_world(local_dir)
          guide_pdf = dtree.pdf(dir_to_canonical(world_dir)) / (4*pi)
      else:
          canonical = dtree.sample(sampler)
          world_dir = canonical_to_dir(canonical)
          local_dir = info.sh_coord.to_local(world_dir)
          bsdf_pdf = bsdf.pdf(local_incoming, local_dir)
          guide_pdf = dtree.pdf(canonical) / (4*pi)
          brdf_val = bsdf.get_reflection(local_incoming, local_dir, uv)
          cos_theta = abs(local_dir[2])

      mix_pdf = alpha*bsdf_pdf + (1-alpha)*guide_pdf
      throughput *= brdf_val * cos_theta / mix_pdf
  else:
      기존 bsdf sampling 유지
```

#### Phase 4: Radiance Splatting

path tracing 후, 각 vertex의 radiance를 SD-tree에 기록:

```cpp
void splat_path_vertices(SDTree &tree,
                         const std::vector<PathVertex> &vertices) {
    Vector3f radiance = vec3f_zero;
    for (int i = vertices.size() - 1; i >= 0; i--) {
        radiance = vertices[i].emission + radiance % vertices[i].throughput;
        Float irradiance = luminance(radiance);
        if (irradiance > 0 && !vertices[i].is_delta) {
            tree.record(vertices[i].position, vertices[i].direction, irradiance);
        }
    }
}
```

#### Phase 5: Thread Safety

병렬 렌더링에서 여러 스레드가 동시에 quadtree에 기록할 수 있으므로
`std::atomic<float>`를 사용 (레퍼런스 구현 방식):

```cpp
struct DQuadTreeNode {
    std::array<std::atomic<Float>, 4> m_sum;
    // ...
};
```

C++20의 `std::atomic<float>::fetch_add` 또는 CAS 루프로 구현:
```cpp
inline void atomic_add(std::atomic<Float> &target, Float value) {
    Float current = target.load(std::memory_order_relaxed);
    while (!target.compare_exchange_weak(current, current + value,
                                          std::memory_order_relaxed));
}
```

### 4.3 기존 코드 수정 사항

- `integrators.h`에 `GuidedPathIntegrator` 선언 추가
- scene_parser에서 `"guided_path"` integrator 타입 지원
- `PathIntegrator`, BSDF, Light, Scene 인터페이스는 **변경 없음**
- `common.h`에 좌표 변환 유틸리티만 추가

### 4.4 구현 순서 (권장)

```
Step 1: DQuadTree 기본 구조
  - DQuadTreeNode, 배열 기반 구조
  - record(), build(), pdf(), sample() 구현
  - 단위 테스트: 균일 기록 → 균일 PDF, 편향 기록 → 비례 PDF

Step 2: DQuadTree 적응적 분할
  - refine() 구현
  - 단위 테스트: 에너지 집중 영역이 더 세밀하게 분할되는지 확인

Step 3: 좌표 변환
  - dir_to_canonical(), canonical_to_dir()
  - 왕복 변환 테스트

Step 4: SDTree (Spatial + Directional)
  - SBinaryTreeNode, SDTree 클래스
  - lookup(), record(), refine_spatial() 구현
  - 단위 테스트: 공간 lookup 정확성

Step 5: GuidedPathIntegrator 기본
  - Integrator 상속, render() 루프 구현
  - 첫 iteration은 BSDF only (기존 PathIntegrator와 동일)
  - splatting + SD-tree 구축
  - 이 시점에서 렌더링 결과가 기존과 동일해야 함

Step 6: Guiding 적용
  - iteration 2+에서 quadtree sampling 도입
  - MIS(BSDF, guide) 구현
  - 결과 비교: 간접 조명이 어려운 장면에서 개선 확인

Step 7: Budget balancing
  - Variance 추정 + 자동 종료
  - Exponential sample doubling 완성

Step 8: 최적화 및 정리
  - Thread safety (atomic)
  - 메모리 제한 (max nodes)
  - 성능 프로파일링
```

### 4.5 주요 파라미터 정리

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| $\rho$ (directional threshold) | $0.01$ | quadtree 분할 에너지 분율 임계값 |
| $c$ (spatial threshold) | $12000$ | binary tree 분할 임계값 |
| $\text{max\_depth}$ (quadtree) | $20$ | quadtree 최대 깊이 |
| $\alpha$ (bsdf sampling fraction) | $0.5$ | BSDF vs guiding 혼합 비율 |

논문에 따르면 $\rho = 0.01$, $c = 12000$은 모든 테스트 장면에서 잘 작동하며,
유일하게 사용자가 설정해야 할 파라미터는 **SD-tree의 최대 메모리** 뿐이다.

---

## 5. 레퍼런스 구현과의 차이점 (원본 논문 범위)

레퍼런스 코드(`guided_path.cpp`)는 논문 이후 발전된 버전이다.
**원본 논문에 해당하는 부분만** 구현하기 위해, 다음은 제외:

| 레퍼런스의 추가 기능 | 원본 논문에서는 |
|---|---|
| Adam optimizer로 $\alpha$ 학습 | **고정값 $\alpha = 0.5$** |
| Box filtering (spatial/directional) | **Nearest filtering만** |
| NEE (Next Event Estimation) 옵션 | **기존 light sampling 유지** |
| `bsdfSamplingFractionLoss` | **없음** |
| Multiple sample combination strategies | **마지막 iteration만 사용** |
| Stochastic box filter | **없음** |

**원본 논문의 핵심만 구현**:
1. SD-tree (spatial binary + directional quadtree)
2. Cylindrical 좌표계
3. Iterative training with exponential doubling ($B_k = 2^{k-1} \cdot B_1$)
4. $\text{MIS}(\alpha \cdot p_\text{bsdf} + (1-\alpha) \cdot p_\text{guide})$ with $\alpha = 0.5$
5. Adaptive spatial refinement ($\text{count} \geq c \cdot \sqrt{2^k}$)
6. Adaptive directional refinement ($\Phi_n / \Phi > \rho$)
7. Budget balancing ($\hat{k} = \arg\min_k \hat{V}_k$)
8. Nearest filtering (가장 가까운 leaf에만 기록)
