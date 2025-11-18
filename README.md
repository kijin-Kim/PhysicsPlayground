# Physics Playground

2D 물리 엔진 시뮬레이션 및 시각화 프로젝트입니다. C++와 OpenGL을 사용하여 구현되었으며, 다양한 Broad Phase 충돌 감지 알고리즘을 비교할 수 있습니다.

## 주요 기능

### 물리 시뮬레이션
- **2D Rigid Body 물리 엔진**
  - 힘/충격량 기반 물리 시뮬레이션
  - 회전 및 각운동량 지원
  - 탄성, 마찰력, 감쇠 구현

### 충돌 감지 시스템
- **GJK (Gilbert-Johnson-Keerthi) 알고리즘**
  - 정확한 볼록 다각형 충돌 감지
- **EPA (Expanding Polytope Algorithm)**
  - 충돌 깊이 및 접촉점 계산
- **다양한 Broad Phase 알고리즘**
  - Naive (Brute Force)
  - Spatial Grid
  - QuadTree
  - Sweep and Prune (SAP)

### 시각화 및 디버깅
- ImGui 기반 실시간 제어 패널
- AABB 및 Broad Phase 구조 시각화
- 실시간 파라미터 조정

## 기술 스택

- **언어**: C++20
- **그래픽스**: OpenGL 3.3+
- **수학 라이브러리**: GLM
- **UI**: ImGui


## 프로젝트 구조

```
PhysicsPlayground/
├── Application/
│   ├── src/
│   │   ├── Layers/          # 애플리케이션 레이어
│   │   │   ├── MainLayer    # 메인 물리 시뮬레이션 레이어
│   │   │   └── ImGuiLayer   # UI 레이어
│   │   ├── Physics/         # 물리 엔진 코어
│   │   │   ├── BroadPhase/  # Broad Phase 알고리즘들
│   │   │   ├── Shapes/      # AABB 및 도형 관련
│   │   │   ├── Object       # 물리 오브젝트
│   │   │   ├── Rigidbody    # 강체 역학
│   │   │   └── CollisionSystem # 충돌 처리 시스템
│   │   └── Events.h         # 이벤트 정의
│   └── config/
│       └── config.ini       # 런타임 설정
└── CommonCore/              # 공통 엔진 코어 (서브모듈)
```

## 빌드 방법

```bash
git clone --recursive https://github.com/yourusername/PhysicsPlayground.git
cd PhysicsPlayground
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

## 사용법

### 기본 조작
1. **Restart**: 시뮬레이션 재시작
2. **Pause/Resume**: 일시정지/재개
3. **Step**: 한 프레임씩 진행
4. **Draw AABBs**: AABB 경계 상자 표시
5. **Draw Broad Phase**: Broad Phase 구조 시각화

### Broad Phase 알고리즘 선택
UI 패널에서 다음 알고리즘 중 선택 가능:
- **Naive**: 모든 오브젝트 쌍 검사 (O(n²))
- **Grid**: 공간 분할 그리드 (설정 가능한 셀 크기)
- **Quadtree**: 재귀적 공간 분할 (깊이 및 노드당 객체 수 조정 가능)
- **SAP**: Sweep and Prune, 정렬 기반 알고리즘

### 설정 파일 (`config/config.ini`)
```ini
[Options]
Paused=false
BroadPhaseAlgorithm=Grid
DrawDebugAABB=false
DrawDebugBroadPhase=true

[GridSettings]
GridCellSize=64.0

[QuadTreeSettings]
MaxObjectsPerCell=2
MaxDepth=6
```

## 아키텍처 특징

### 이벤트 기반 시스템
- EventBus를 통한 느슨한 결합
- UI와 물리 시스템 간 통신
- 런타임 설정 변경 지원

### 물리 파이프라인
1. **Force Integration** (힘 적분)
2. **Broad Phase** (대략적 충돌 감지)
3. **Narrow Phase** (정밀 충돌 감지 - GJK/EPA)
4. **Constraint Solving** (제약 조건 해결)
   - Velocity Constraints (속도 제약)
   - Position Correction (위치 보정)
5. **Velocity Integration** (속도 적분)

## 최적화

### 문제 인식
초기 구현에서 200개의 강체를 시뮬레이션할 때 심각한 성능 저하를 경험했습니다:
- **Naive 구현**: 모든 객체 쌍을 검사 → O(n²) 복잡도 (200개 객체 = 19,900회 검사)
- **프레임레이트**: ~15-20 FPS (목표: 60 FPS)
- **병목 지점**: 충돌 감지가 전체 물리 시뮬레이션 시간의 70% 이상 차지

### 구현된 최적화 기법

#### 1. Broad Phase 알고리즘

**문제**: 모든 객체 쌍에 대해 충돌 검사를 수행하는 것은 비효율적

**해결**: 여러 Broad Phase 알고리즘 구현 및 비교

##### Spatial Grid
```cpp
// 공간을 균일한 그리드로 분할
// 각 객체를 AABB 기준으로 해당하는 셀에 할당
for (int x = cellMin.x; x <= cellMax.x; ++x)
    for (int y = cellMin.y; y <= cellMax.y; ++y)
        grid[glm::ivec2(x, y)].push_back(i);
```
- **특징**: 구현이 단순하고 메모리 효율적
- **성능**: ~8배 향상 (60 FPS → 500+ FPS)
- **최적 사용**: 객체가 비교적 균등하게 분포된 경우

##### QuadTree
```cpp
// 재귀적으로 공간을 4개의 사분면으로 분할
// 노드당 객체 수가 임계값 초과 시 세분화
if (node.ObjectIndices.size() > maxObjectsPerNode_ && depth < maxDepth_)
{
    Subdivide(node);
    // 자식 노드로 객체 재분배
}
```
- **특징**: 동적 공간 분할, 객체 밀집도에 적응적
- **성능**: ~7배 향상 (60 FPS → 400+ FPS)
- **최적 사용**: 객체가 불균등하게 분포된 경우

##### Sweep and Prune (SAP)
```cpp
// X축 기준으로 AABB의 시작/끝점을 정렬
// 활성 리스트를 유지하며 잠재적 충돌 쌍 검출
std::sort(endpoints.begin(), endpoints.end());
```
- **특징**: 정렬 기반, 캐시 친화적
- **성능**: ~7.5배 향상 (60 FPS → 450+ FPS)
- **최적 사용**: 한 축으로 객체가 분산된 경우

#### 2. 멀티스레딩 (Multithreading)

**문제**: Narrow Phase의 GJK/EPA 계산이 CPU 집약적

**해결**: JobSystem을 통한 병렬 처리
```cpp
// Narrow Phase를 병렬로 처리
jobSystem.Prepare(potentialCollisions.size());
for (const auto& pair : potentialCollisions)
{
    jobSystem.Enqueue([&, pair]() {
        Manifold manifold = Collide(verts1, verts2);
        if (manifold.bHit)
        {
            std::lock_guard<std::mutex> lock(mutex);
            collisionTasks.emplace_back(pair.first, pair.second, manifold);
        }
    });
}
jobSystem.WaitAll();
```
- **효과**: CPU 코어 활용도 극대화
- **성능 향상**: 멀티코어 시스템에서 30-40% 추가 개선

#### 3. 고정 타임스텝 (Fixed Timestep)

**문제**: 가변 델타타임으로 인한 물리 시뮬레이션 불안정성

**해결**: 고정 타임스텝 적분 (Semi-Fixed Timestep)
```cpp
constexpr float FIXED_TIME_STEP = 1.0f / 60.0f;
constexpr float MAX_ACCUMULATED_TIME = 0.25f;

accumulatedTime += deltaTime;
accumulatedTime = std::min(accumulatedTime, MAX_ACCUMULATED_TIME);

while (accumulatedTime >= FIXED_TIME_STEP)
{
    PhysicsStep(objects, FIXED_TIME_STEP);
    accumulatedTime -= FIXED_TIME_STEP;
}
```
- **효과**: 프레임레이트와 무관하게 일관된 물리 시뮬레이션
- **안정성**: 높은 FPS에서도 물리 안정성 보장

## 성능

200개 강체 기준 성능 (대략적 수치):
- **Naive**: ~60 FPS
- **Grid**: ~500+ FPS
- **QuadTree**: ~400+ FPS
- **SAP**: ~450+ FPS

※ 실제 성능은 하드웨어 및 시나리오에 따라 다름

