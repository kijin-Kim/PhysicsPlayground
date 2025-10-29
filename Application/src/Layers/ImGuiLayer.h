#pragma once
#include "Events.h"

#include "Core/Layers/Layer.h"


class ImGuiLayer : public ILayer
{
public:
	ImGuiLayer(EventBus& eventBus);
	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;
	virtual void OnDestroy() override;

private:
	bool bShouldPauseUpdate_ = false;
	BroadPhase::Type currentBroadPhaseType_;
	float currentGridCellSize_;
};