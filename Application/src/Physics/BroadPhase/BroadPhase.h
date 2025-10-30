#pragma once
#include <array>
#include <unordered_set>
#include <vector>

#include "Core/EventBus.h"

#include "Physics/Object.h"
#include "Physics/Shapes/AABB.h"


class EventBus;

class IBroadPhase
{
public:
	virtual ~IBroadPhase() = default;
	virtual void ComputePotentialCollisions(const std::vector<Object>& objects, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions) = 0;

	virtual void DrawDebug(Renderer& renderer)
	{
	}

	uint64_t MakePairKey(size_t indexA, size_t indexB)
	{
		return (static_cast<uint64_t>(std::min(indexA, indexB)) << 32) | static_cast<uint64_t>(std::max(indexA, indexB));
	}

protected:
};

class NaiveBroadPhase : public IBroadPhase
{
public:
	virtual void ComputePotentialCollisions(const std::vector<Object>& objects, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions) override;
};

class GridBroadPhase : public IBroadPhase
{
public:
	GridBroadPhase();
	virtual ~GridBroadPhase();
	virtual void ComputePotentialCollisions(const std::vector<Object>& objects, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions) override;
	virtual void DrawDebug(Renderer& renderer) override;

private:
	float cellSize = 100.0f;
	std::vector<EventBus::EventHandle> subscribedEvents_;
};


class QuadTreeBroadPhase : public IBroadPhase
{
public:
	struct QuadTreeNode
	{
		AABB Bounds;
		std::vector<size_t> ObjectIndices;
		std::array<std::unique_ptr<QuadTreeNode>, 4> Children;
		bool bIsDivided = false;
	};
	QuadTreeBroadPhase();
	virtual ~QuadTreeBroadPhase();
	virtual void ComputePotentialCollisions(const std::vector<Object>& objects, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions) override;
	virtual void DrawDebug(Renderer& renderer) override;

private:
	void InsertNode(QuadTreeNode& node, size_t objectIndex, const AABB& aabb, const std::vector<AABB>& allAABBs, int depth);
	void Subdivide(QuadTreeNode& node);
	void CollectPotentialCollisions(const QuadTreeNode& node, const std::vector<AABB>& aabbs, std::vector<size_t>& boundaryObjects, std::unordered_set<uint64_t>& checked, std::vector<std::pair<size_t, size_t> >& outPairs);
	void DrawNodeRecursive(Renderer& renderer, QuadTreeNode& node);

private:
	QuadTreeNode rootNode_{};
	int maxObjectsPerNode_ = 2;
	int maxDepth_ = 6;
	std::vector<EventBus::EventHandle> subscribedEvents_;
};

class SAPBroadPhase : public IBroadPhase
{
public:
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
	SAPBroadPhase();
	virtual void ComputePotentialCollisions(const std::vector<Object>& objects, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions) override;
};