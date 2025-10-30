#pragma once
#include <vector>
#include <array>


#include "Events.h"

#include "Core/Layers/Layer.h"
#include "Physics/Object.h"

struct AABB
{
	glm::vec2 Min;
	glm::vec2 Max;
};

struct QuadTreeNode
{
	AABB Bounds;
	std::vector<size_t> ObjectIndices;
	std::array<std::unique_ptr<QuadTreeNode>, 4> Children;
	bool bIsDivided = false;
};

struct EndPoint
{
	float Value;
	size_t ObjectIndex;
	bool bIsMin; // true: min, false: max

	bool operator<(const EndPoint& other) const
	{
		if (Value == other.Value)
		{
			// min이 max보다 먼저 오도록
			return bIsMin && !other.bIsMin;
		}
		return Value < other.Value;
	}
};

class MainLayer : public ILayer
{
public:
	MainLayer(EventBus& eventBus);
	virtual void OnInit() override;
	virtual void OnUpdate(float deltaTime) override;
	virtual void OnRender(Renderer& renderer) override;

private:
	void Step(float deltaTime);

	void BroadPhaseNaive(std::vector<std::pair<size_t, size_t> >& outPotentialCollisions);
	void BroadPhaseGrid(std::vector<std::pair<size_t, size_t> >& outPotentialCollisions);
	void BroadPhaseQuadtree(QuadTreeNode& outRoot, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions);
	void BroadPhaseSAP(std::vector<std::pair<size_t, size_t> >& outPotentialCollisions);

private:
	std::vector<Object> objects_;
	QuadTreeNode rootNode_;

	// 이하 변수는 ini 파일에 의해 런타임에 덮어씌어짐
	bool bShouldPauseUpdate = false;
	float cellSize = 0.0f;
	BroadPhase::Type broadPhaseAlgorithm_ = BroadPhase::Type::Naive;

};