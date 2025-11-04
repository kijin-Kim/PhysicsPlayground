#pragma once
#include "Events.h"

#include "Core/Layers/Layer.h"

class ImGuiLayer : public ILayer
{
public:
	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;
	virtual void OnDestroy() override;

private:
	bool bShouldPauseUpdate_ = false;
	BroadPhase::Type currentBroadPhaseType_ = BroadPhase::Type::Naive;
	bool bDrawDebugAABBs_ = false;
	bool bDrawDebugBroadPhase_ = false;

	// Grid Broad Phase 설정
	float gridCellSize_ = 64.0f;

	// QuadTree Broad Phase 설정
	int quadTreeMaxDepth_ = 6;
	int quadTreeMaxObjectsPerNode_ = 2;
};