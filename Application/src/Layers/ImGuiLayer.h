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
	BroadPhase::Type currentBroadPhaseType_;


	// Grid Broad Phase 설정
	float currentGridCellSize_;

	// QuadTree Broad Phase 설정
	int currentQuadTreeMaxDepth_ = 6;
	int currentQuadTreeMaxObjectsPerNode_ = 2;

};