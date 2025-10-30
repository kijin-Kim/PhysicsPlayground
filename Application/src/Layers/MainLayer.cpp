#include "MainLayer.h"


#include "Renderer/Shapes.h"

#include "Physics/Object.h"
#include "Renderer/Renderer.h"


#include "Core/EventBus.h"
#include "GLM/gtx/hash.hpp"

#include "Physics/CollisionSystem.h"
#include "Physics/BroadPhase/BroadPhase.h"


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

MainLayer::MainLayer()
	: bShouldPauseUpdate(false)
	, cellSize(100.0f)
{
	EventBus& eventBus = EventBus::GetInstance();

	subscribedEvents_.push_back(eventBus.Subscribe<PauseUpdateEvent>([this](const PauseUpdateEvent& e)
	{
		bShouldPauseUpdate = e.bPaused;
	}));

	subscribedEvents_.push_back(eventBus.Subscribe<StepEvent>([this](const StepEvent& e)
	{
		Step(e.DeltaTime);
	}));

	subscribedEvents_.push_back(eventBus.Subscribe<ChangeBroadPhaseAlgorithmEvent>([this](const ChangeBroadPhaseAlgorithmEvent& e)
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

}

MainLayer::~MainLayer()
{
	for (const EventBus::EventHandle& handle : subscribedEvents_)
	{
		EventBus::GetInstance().Unsubscribe(handle);
	}
}


void MainLayer::OnInit()
{
	Object& o1 = objects_.emplace_back();
	o1.SetShape(std::make_unique<RectangleShape>(glm::vec2(100.0f, 100.0f)));
	o1.GetShape()->SetColor(color1);
	o1.GetRigidbody().SetMass(0.0f);
	o1.SetPosition(glm::vec2(-30.0f, -30.0f));

	std::vector<glm::vec2> polygonVertices = {
		glm::vec2(50.0f, 70.0f),
		glm::vec2(70.0f, 30.0f),
		glm::vec2(100.0f, 20.0f),
		glm::vec2(120.0f, 70.0f)};
	NormalizeToCentroid(polygonVertices);

	Object& o2 = objects_.emplace_back();
	//o2.SetShape(std::make_unique<RectangleShape>(glm::vec2(100.0f, 100.0f)));
	o2.SetShape(std::make_unique<ConvexShape>(polygonVertices));
	o2.GetShape()->SetColor(color4);
	o2.GetRigidbody().SetMass(1.0f);
	o2.SetPosition(glm::vec2(30.0f, 80.0f));

	Object& floor = objects_.emplace_back();
	floor.SetShape(std::make_unique<RectangleShape>(glm::vec2(800.0f, 50.0f)));
	floor.GetShape()->SetColor(color1);
	floor.GetRigidbody().SetMass(0.0f);
	floor.SetPosition(glm::vec2(0.0f, -300.0f));

	Object& leftWall = objects_.emplace_back();
	leftWall.SetShape(std::make_unique<RectangleShape>(glm::vec2(50.0f, 600.0f)));
	leftWall.GetShape()->SetColor(color1);
	leftWall.GetRigidbody().SetMass(0.0f);
	leftWall.SetPosition(glm::vec2(-400.0f, 0.0f));

	Object& rightWall = objects_.emplace_back();
	rightWall.SetShape(std::make_unique<RectangleShape>(glm::vec2(50.0f, 600.0f)));
	rightWall.GetShape()->SetColor(color1);
	rightWall.GetRigidbody().SetMass(0.0f);
	rightWall.SetPosition(glm::vec2(400.0f, 0.0f));

	Object& ceiling = objects_.emplace_back();
	ceiling.SetShape(std::make_unique<RectangleShape>(glm::vec2(800.0f, 50.0f)));
	ceiling.GetShape()->SetColor(color1);
	ceiling.GetRigidbody().SetMass(0.0f);
	ceiling.SetPosition(glm::vec2(0.0f, 300.0f));

	// random circles
	for (int i = 0; i < 30; ++i)
	{
		Object& circleObj = objects_.emplace_back();
		circleObj.SetShape(std::make_unique<CircleShape>(15.0f));
		circleObj.GetShape()->SetColor(color2);
		circleObj.GetRigidbody().SetMass(1.0f);
		float x = static_cast<float>(rand() % 700 - 350);
		float y = static_cast<float>(rand() % 500 - 250);
		circleObj.SetPosition(glm::vec2(x, y));
	}
}

void MainLayer::OnUpdate(float deltaTime)
{
	if (!bShouldPauseUpdate)
	{
		Step(deltaTime);
	}
}

void MainLayer::OnRender(Renderer& renderer)
{
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
	collisionSystem_.Update(objects_, deltaTime);
}