#include "MainLayer.h"

#include "Colors.h"
#include "Renderer/Shapes.h"

#include "Physics/Object.h"
#include "Renderer/Renderer.h"

#include "Core/EventBus.h"
#include "GLM/gtx/hash.hpp"

#include "Physics/BroadPhase/BroadPhase.h"
#include "Physics/CollisionSystem.h"

#include "tracy/Tracy.hpp"
#include "tracy/TracyOpenGL.hpp"

namespace
{
void NormalizeToCentroid(std::vector<glm::vec2>& outPoints)
{
	glm::vec2 centroid(0.0f);
	float area = 0.0f;
	for (int i = 0; i < outPoints.size(); ++i)
	{
		int j = (i + 1) % outPoints.size();

		// 벡터 외적을 이용한 다각형의 면적 계산 누적
		const float cross = outPoints[i].x * outPoints[j].y - outPoints[j].x * outPoints[i].y;
		area += cross * 0.5f;

		// 무게중심 계산 누적
		const glm::vec2 weight = (outPoints[i] + outPoints[j]);
		centroid += weight * cross;
	}

	centroid /= (6.0f * area);
	for (glm::vec2& point : outPoints)
	{
		point -= centroid;
	}
}
} // namespace

float ComputePolygonInertia(const std::vector<glm::vec2>& vertices, float mass)
{
	float area = 0.0f;
	float inertia = 0.0f;
	float Ix = 0.0f;
	float Iy = 0.0f;

	for (size_t i = 0; i < vertices.size(); ++i)
	{
		size_t j = (i + 1) % vertices.size();
		glm::vec2 p0 = vertices[i];
		glm::vec2 p1 = vertices[j];
		float cross = p0.x * p1.y - p1.x * p0.y;
		area += cross * 0.5f;
		Ix += (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y) * cross;
		Iy += (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) * cross;
	}
	inertia = (Ix + Iy) / 12.0f * (mass / area);
	return inertia;
}

MainLayer::MainLayer()
	: bShouldPauseUpdate(false)
	, cellSize(100.0f)
	, accumulatedTime(0.0f)
{
	EventBus& eventBus = EventBus::GetInstance();

	subscribedEvents_.push_back(
		eventBus.Subscribe<PauseUpdateEvent>([this](const PauseUpdateEvent& e) { bShouldPauseUpdate = e.bPaused; }));

	subscribedEvents_.push_back(eventBus.Subscribe<StepEvent>([this](const StepEvent& e) { Step(e.DeltaTime); }));

	subscribedEvents_.push_back(eventBus.Subscribe<ChangeBroadPhaseAlgorithmEvent>(
		[this](const ChangeBroadPhaseAlgorithmEvent& e)
		{
			switch (e.NewAlgorithm)
			{
			case BroadPhase::Type::Naive:
				collisionSystem_.SetBroadPhaseAlgorithm(std::make_unique<NaiveBroadPhase>());
				break;
			case BroadPhase::Type::Grid:
				collisionSystem_.SetBroadPhaseAlgorithm(std::make_unique<GridBroadPhase>());
				break;
			case BroadPhase::Type::Quadtree:
				collisionSystem_.SetBroadPhaseAlgorithm(std::make_unique<QuadTreeBroadPhase>());
				break;
			case BroadPhase::Type::SAP:
				collisionSystem_.SetBroadPhaseAlgorithm(std::make_unique<SAPBroadPhase>());
				break;
			default:
				break;
			}
		}));
	subscribedEvents_.push_back(eventBus.Subscribe<RestartEvent>([this](const RestartEvent& e) { Restart(); }));
}

MainLayer::~MainLayer()
{
	for (const EventBus::EventHandle& handle : subscribedEvents_)
	{
		EventBus::GetInstance().Unsubscribe(handle);
	}
}

void MainLayer::Restart()
{
	objects_.clear();
	Object& o1 = objects_.emplace_back();
	o1.SetShape(std::make_unique<RectangleShape>(glm::vec2(100.0f, 100.0f)));
	o1.GetShape()->SetColor(PColor::OceanBlue);
	o1.GetRigidbody().SetMass(0.0f);
	o1.SetPosition(glm::vec2(-30.0f, -30.0f));

	// std::vector<glm::vec2> polygonVertices
	// 	= {glm::vec2(50.0f, 70.0f), glm::vec2(70.0f, 30.0f), glm::vec2(100.0f, 20.0f), glm::vec2(120.0f, 70.0f)};
	// NormalizeToCentroid(polygonVertices);
	//
	// Object& o2 = objects_.emplace_back();
	// o2.SetShape(std::make_unique<ConvexShape>(polygonVertices));
	// o2.GetShape()->SetColor(PColor::OceanBlue);
	// o2.GetRigidbody().SetMass(1.0f);
	// o2.SetPosition(glm::vec2(30.0f, 80.0f));

	constexpr glm::vec2 extents(1200, 800);
	constexpr glm::vec2 maxBounds = glm::vec2(0.0f) + 0.5f * extents;
	constexpr glm::vec2 minBounds = glm::vec2(0.0f) - 0.5f * extents;
	constexpr float wallThickness = 100.0f;
	Object& floor = objects_.emplace_back();
	floor.SetShape(std::make_unique<RectangleShape>(glm::vec2(extents.x, wallThickness)));
	floor.GetShape()->SetColor(PColor::OceanBlue);
	floor.GetRigidbody().SetMass(0.0f);
	floor.SetPosition(glm::vec2(0.0f, minBounds.y));

	Object& leftWall = objects_.emplace_back();
	leftWall.SetShape(std::make_unique<RectangleShape>(glm::vec2(wallThickness, extents.y)));
	leftWall.GetShape()->SetColor(PColor::OceanBlue);
	leftWall.GetRigidbody().SetMass(0.0f);
	leftWall.SetPosition(glm::vec2(minBounds.x, 0.0f));

	Object& rightWall = objects_.emplace_back();
	rightWall.SetShape(std::make_unique<RectangleShape>(glm::vec2(wallThickness, extents.y)));
	rightWall.GetShape()->SetColor(PColor::OceanBlue);
	rightWall.GetRigidbody().SetMass(0.0f);
	rightWall.SetPosition(glm::vec2(maxBounds.x, 0.0f));

	Object& ceiling = objects_.emplace_back();
	ceiling.SetShape(std::make_unique<RectangleShape>(glm::vec2(extents.x, wallThickness)));
	ceiling.GetShape()->SetColor(PColor::OceanBlue);
	ceiling.GetRigidbody().SetMass(0.0f);
	ceiling.SetPosition(glm::vec2(0.0f, maxBounds.y));

	for (int i = 0; i < 100; ++i)
	{
		Object& obj = objects_.emplace_back();
		obj.SetShape(std::make_unique<RectangleShape>(glm::vec2(50.0f, 50.0f)));
		obj.GetShape()->SetColor(PColor::LavenderPurple);
		obj.GetRigidbody().SetMass(100.0f);
		float x = rand() % static_cast<int>(extents.x * 0.5f) - extents.x * 0.5f * 0.5f;
		float y = rand() % static_cast<int>(extents.y * 0.5f) - extents.y * 0.5f * 0.5f;
		obj.SetPosition(glm::vec2(x, y));
	}


	for (Object& obj : objects_)
	{
		std::vector<glm::vec2> vert = obj.GetShape()->GetVertices();
		for (glm::vec2& point : vert)
		{
			point = obj.GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		float inertia = ComputePolygonInertia(vert, obj.GetRigidbody().GetMass());
		obj.GetRigidbody().SetInertia(inertia);
	}
}
void MainLayer::OnInit()
{
	Restart();
}

void MainLayer::OnUpdate(float deltaTime)
{
	ZoneScoped;
	if (!bShouldPauseUpdate)
	{
		Step(deltaTime);
	}
}

void MainLayer::OnRender(Renderer& renderer)
{
	TracyGpuZone("MainLayer::OnRender");
	renderer.BeginScene();
	for (Object& object : objects_)
	{
		object.OnRender(renderer);
	}
	collisionSystem_.DrawDebug(objects_, renderer);
	renderer.EndScene();
}

void MainLayer::Step(float deltaTime)
{
	ZoneScoped;
	constexpr float FIXED_TIME_STEP = 1.0f / 60.0f;
	constexpr float MAX_ACCUMULATED_TIME = 0.25f;

	accumulatedTime += deltaTime;
	accumulatedTime = std::min(accumulatedTime, MAX_ACCUMULATED_TIME);

	while (accumulatedTime >= FIXED_TIME_STEP)
	{
		for (Object& object : objects_)
		{
			object.OnUpdate(FIXED_TIME_STEP);
		}
		collisionSystem_.PhysicsStep(objects_, FIXED_TIME_STEP);
		accumulatedTime -= FIXED_TIME_STEP;
	}
}