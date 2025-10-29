#pragma once
#include <vector>

#include "Events.h"

#include "Core/Layers/Layer.h"
#include "Physics/Object.h"


class MainLayer : public ILayer
{
public:
	MainLayer(EventBus& eventBus);
	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;

private:
	void Step(float deltaTime);

	void BroadPhaseNaive(std::vector<std::pair<size_t, size_t>>& outPotentialCollisions);
	void BroadPhaseGrid(std::vector<std::pair<size_t, size_t>>& outPotentialCollisions);

private:
	std::vector<Object> objects_;


	// 이하 변수는 ini 파일에 의해 런타임에 덮어씌어짐
	bool bShouldPauseUpdate = false;
	float cellSize = 0.0f;
	BroadPhase::Type broadPhaseAlgorithm_ = BroadPhase::Type::Naive;
};