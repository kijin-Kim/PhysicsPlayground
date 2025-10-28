#pragma once
#include <vector>

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

private:
	std::vector<Object> objects_;
	bool bShouldPauseUpdate = true;
};