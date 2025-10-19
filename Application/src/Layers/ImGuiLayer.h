#pragma once
#include "Core/Layers/Layer.h"


class ImGuiLayer : public ILayer
{
public:
	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;
	virtual void OnDestroy() override;
};