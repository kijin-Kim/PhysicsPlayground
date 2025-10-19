#pragma once
#include <vector>

#include "Core/Layers/Layer.h"
#include "Physics/Object.h"


class MainLayer : public ILayer
{
public:
	MainLayer();
	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;

private:
	std::vector<Object> objects_;
};