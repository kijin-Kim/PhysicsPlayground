#pragma once
#include <vector>


#include "Events.h"

#include "Core/Layers/Layer.h"

#include "Physics/CollisionSystem.h"
#include "Physics/Object.h"


class IBroadPhase;

class MainLayer : public ILayer
{
public:
	MainLayer();
	virtual ~MainLayer();


	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;

private:
	void Step(float deltaTime);

private:
	std::vector<Object> objects_;
	CollisionSystem collisionSystem_;
	std::vector<EventBus::EventHandle> subscribedEvents_;

	// 이하 변수는 ini 파일에 의해 런타임에 덮어씌어짐
	bool bShouldPauseUpdate;
	float cellSize;
};