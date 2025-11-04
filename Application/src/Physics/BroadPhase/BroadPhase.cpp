#include "BroadPhase.h"

#include "Colors.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "Events.h"

#include "GLM/gtx/hash.hpp"
#include "Physics/Shapes/AABB.h"

#include "tracy/Tracy.hpp"

#include <ranges>

void NaiveBroadPhase::ComputePotentialCollisions(const std::vector<Object>& objects,
												 std::vector<std::pair<size_t, size_t>>& outPotentialCollisions)
{
	ZoneScoped;
	for (size_t i = 0; i < objects.size(); ++i)
	{
		for (size_t j = i + 1; j < objects.size(); ++j)
		{
			outPotentialCollisions.emplace_back(i, j);
		}
	}
}

GridBroadPhase::GridBroadPhase()
	: cellSize(64.0f)
	, gridMin_(FLT_MAX)
	, gridMax_(-FLT_MAX)
{
	EventBus& eventBus = EventBus::GetInstance();
	subscribedEvents_.push_back(eventBus.Subscribe<GridCellSizeChangedEvent>([this](const GridCellSizeChangedEvent& e)
																			 { cellSize = e.NewCellSize; }));
}

GridBroadPhase::~GridBroadPhase()
{
	EventBus& eventBus = EventBus::GetInstance();
	for (const EventBus::EventHandle& handle : subscribedEvents_)
	{
		eventBus.Unsubscribe(handle);
	}
}

void GridBroadPhase::ComputePotentialCollisions(const std::vector<Object>& objects,
												std::vector<std::pair<size_t, size_t>>& outPotentialCollisions)
{
	ZoneScoped;
	std::unordered_map<glm::ivec2, std::vector<size_t>> grid;
	grid.reserve(objects.size());
	std::vector<AABB> aabbs(objects.size());
	for (size_t i = 0; i < objects.size(); ++i)
	{
		std::vector<glm::vec2> vert = objects[i].GetShape()->GetVertices();
		for (glm::vec2& point : vert)
		{
			point = objects[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		aabbs[i] = AABB::Compute(vert);

		const glm::ivec2 cellMin = glm::floor(aabbs[i].Min / cellSize);
		const glm::ivec2 cellMax = glm::floor(aabbs[i].Max / cellSize);
		for (int x = cellMin.x; x <= cellMax.x; ++x)
		{
			for (int y = cellMin.y; y <= cellMax.y; ++y)
			{
				grid[glm::ivec2(x, y)].push_back(i);
			}
		}
	}

	gridMin_ = glm::vec2(FLT_MAX);
	gridMax_ = glm::vec2(-FLT_MAX);
	for (const glm::ivec2& cell : grid | std::views::keys)
	{
		gridMin_ = glm::min(gridMin_, glm::vec2(cell) * cellSize);
		gridMax_ = glm::max(gridMax_, (glm::vec2(cell) + glm::vec2(1.0f)) * cellSize);
	}

	std::unordered_set<uint64_t> checkedPairs;
	checkedPairs.reserve(objects.size());
	outPotentialCollisions.reserve(objects.size());
	for (const std::pair<const glm::ivec2, std::vector<size_t>>& cell : grid)
	{
		for (size_t i = 0; i < cell.second.size(); ++i)
		{
			for (size_t j = i + 1; j < cell.second.size(); ++j)
			{
				size_t indexA = cell.second[i];
				size_t indexB = cell.second[j];
				uint64_t pairKey = MakePairKey(indexA, indexB);
				if (checkedPairs.insert(pairKey).second)
				{
					outPotentialCollisions.emplace_back(indexA, indexB);
				}
			}
		}
	}
}

void GridBroadPhase::DrawDebug(Renderer& renderer)
{
	for (float x = gridMin_.x; x <= gridMax_.x; x += cellSize)
	{
		renderer.DrawRectangle( {x, (gridMin_.y + gridMax_.y) * 0.5f}, 0.0f,
							 {0.1f, gridMax_.y - gridMin_.y}, PColor::Grey* glm::vec4(1.0f, 1.0f, 1.0f, 0.5f), true);
	}
	for (float y = gridMin_.y; y <= gridMax_.y; y += cellSize)
	{
		renderer.DrawRectangle( {(gridMin_.x + gridMax_.x) * 0.5f, y}, 0.0f,
							 {gridMax_.x - gridMin_.x, 0.1f}, PColor::Grey * glm::vec4(1.0f, 1.0f, 1.0f, 0.5f), true);
	}
}

QuadTreeBroadPhase::QuadTreeBroadPhase()
{
	EventBus& eventBus = EventBus::GetInstance();
	subscribedEvents_.push_back(eventBus.Subscribe<QuadTreeMaxObjectsPerNodeChangedEvent>(
		[this](const QuadTreeMaxObjectsPerNodeChangedEvent& e) { maxObjectsPerNode_ = e.NewMaxObjectsPerNode; }));
	subscribedEvents_.push_back(eventBus.Subscribe<QuadTreeMaxDepthChangedEvent>(
		[this](const QuadTreeMaxDepthChangedEvent& e) { maxDepth_ = e.NewMaxDepth; }));
}

QuadTreeBroadPhase::~QuadTreeBroadPhase()
{
	for (const EventBus::EventHandle& handle : subscribedEvents_)
	{
		EventBus::GetInstance().Unsubscribe(handle);
	}
}

void QuadTreeBroadPhase::ComputePotentialCollisions(const std::vector<Object>& objects,
													std::vector<std::pair<size_t, size_t>>& outPotentialCollisions)
{
	ZoneScoped;
	std::vector<AABB> aabbs(objects.size());
	for (size_t i = 0; i < objects.size(); ++i)
	{
		std::vector<glm::vec2> vert = objects[i].GetShape()->GetVertices();
		for (glm::vec2& point : vert)
		{
			point = objects[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		aabbs[i] = AABB::Compute(vert);
	}

	AABB worldAABB;
	worldAABB.Min = glm::vec2(FLT_MAX);
	worldAABB.Max = glm::vec2(-FLT_MAX);
	for (const AABB& aabb : aabbs)
	{
		worldAABB.Min = glm::min(worldAABB.Min, aabb.Min);
		worldAABB.Max = glm::max(worldAABB.Max, aabb.Max);
	}
	constexpr float PADDING = 10;
	worldAABB.Min -= glm::vec2(PADDING);
	worldAABB.Max += glm::vec2(PADDING);
	rootNode_ = QuadTreeNode();
	rootNode_.Bounds = worldAABB;

	for (size_t i = 0; i < objects.size(); ++i)
	{
		InsertNode(rootNode_, i, aabbs[i], aabbs, 0);
	}

	std::unordered_set<uint64_t> checked;
	checked.reserve(objects.size());
	std::vector<size_t> boundaryObjects;
	CollectPotentialCollisions(rootNode_, aabbs, boundaryObjects, checked, outPotentialCollisions);
}

void QuadTreeBroadPhase::DrawDebug(Renderer& renderer)
{
	DrawNodeRecursive(renderer, rootNode_);
}

void QuadTreeBroadPhase::InsertNode(QuadTreeNode& node, size_t objectIndex, const AABB& aabb,
									const std::vector<AABB>& allAABBs, int depth)
{
	if (!node.Bounds.Overlaps(aabb))
	{
		return;
	}

	if (node.bIsDivided)
	{
		for (size_t i = 0; i < 4; ++i)
		{
			if (node.Children[i]->Bounds.Contains(aabb))
			{
				InsertNode(*node.Children[i], objectIndex, aabb, allAABBs, depth + 1);
				return;
			}
		}
		node.ObjectIndices.push_back(objectIndex);
		return;
	}

	node.ObjectIndices.push_back(objectIndex);
	if (node.ObjectIndices.size() > maxObjectsPerNode_ && depth < maxDepth_)
	{
		Subdivide(node);
		std::vector<size_t> remainings;
		for (size_t index : node.ObjectIndices)
		{
			bool bInserted = false;
			for (size_t i = 0; i < 4; ++i)
			{
				if (node.Children[i]->Bounds.Contains(allAABBs[index]))
				{
					InsertNode(*node.Children[i], index, allAABBs[index], allAABBs, depth + 1);
					bInserted = true;
					break;
				}
			}

			if (!bInserted)
			{
				remainings.push_back(index);
			}
		}
		node.ObjectIndices = remainings;
	}
}

void QuadTreeBroadPhase::Subdivide(QuadTreeNode& node)
{
	const glm::vec2 c = (node.Bounds.Min + node.Bounds.Max) * 0.5f;
	auto createChildNode = [](const glm::vec2& min, const glm::vec2& max)
	{
		std::unique_ptr<QuadTreeNode> node = std::make_unique<QuadTreeNode>();
		node->Bounds.Min = min;
		node->Bounds.Max = max;
		return node;
	};
	// 0: NE, 1: NW, 2: SW, 3: SE
	node.Children[0] = createChildNode({c.x, c.y}, {node.Bounds.Max.x, node.Bounds.Max.y});
	node.Children[1] = createChildNode({node.Bounds.Min.x, c.y}, {c.x, node.Bounds.Max.y});
	node.Children[2] = createChildNode({node.Bounds.Min.x, node.Bounds.Min.y}, {c.x, c.y});
	node.Children[3] = createChildNode({c.x, node.Bounds.Min.y}, {node.Bounds.Max.x, c.y});
	node.bIsDivided = true;
}

void QuadTreeBroadPhase::CollectPotentialCollisions(const QuadTreeNode& node, const std::vector<AABB>& aabbs,
													std::vector<size_t>& boundaryObjects,
													std::unordered_set<uint64_t>& checked,
													std::vector<std::pair<size_t, size_t>>& outPairs)
{
	// 걸친 객체들과 노드 내부 객체들끼리 검사
	for (size_t indexA : node.ObjectIndices)
	{
		const AABB& aabbA = aabbs[indexA];
		for (size_t indexB : boundaryObjects)
		{
			const AABB& aabbB = aabbs[indexB];
			if (!aabbA.Overlaps(aabbB))
			{
				continue;
			}

			uint64_t pairKey = MakePairKey(indexA, indexB);
			if (checked.insert(pairKey).second)
			{
				outPairs.emplace_back(indexA, indexB);
			}
		}
	}

	// 노드 내부 객체들끼리 검사
	const auto& indices = node.ObjectIndices;
	for (size_t i = 0; i < indices.size(); ++i)
	{
		for (size_t j = i + 1; j < indices.size(); ++j)
		{
			size_t idA = indices[i];
			size_t idB = indices[j];

			uint64_t pairKey = MakePairKey(idA, idB);
			if (checked.insert(pairKey).second)
			{
				outPairs.emplace_back(idA, idB);
			}
		}
	}

	if (!node.bIsDivided)
	{
		return;
	}

	// 자식 노드로 전달할 걸친 객체 목록 생성
	boundaryObjects.insert(boundaryObjects.end(), node.ObjectIndices.begin(), node.ObjectIndices.end());

	for (const auto& child : node.Children)
	{
		if (child)
		{
			CollectPotentialCollisions(*child, aabbs, boundaryObjects, checked, outPairs);
		}
	}
}

void QuadTreeBroadPhase::DrawNodeRecursive(Renderer& renderer, QuadTreeNode& node)
{
	node.Bounds.Draw(renderer, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
	if (node.bIsDivided)
	{
		for (std::unique_ptr<QuadTreeNode>& child : node.Children)
		{
			DrawNodeRecursive(renderer, *child);
		}
	}
}

SAPBroadPhase::SAPBroadPhase() {}

void SAPBroadPhase::ComputePotentialCollisions(const std::vector<Object>& objects,
											   std::vector<std::pair<size_t, size_t>>& outPotentialCollisions)
{
	ZoneScoped;
	std::vector<AABB> aabbs(objects.size());
	for (size_t i = 0; i < objects.size(); ++i)
	{
		std::vector<glm::vec2> vert = objects[i].GetShape()->GetVertices();
		for (glm::vec2& point : vert)
		{
			point = objects[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		aabbs[i] = AABB::Compute(vert);
	}

	std::vector<EndPoint> endpoints;
	endpoints.reserve(objects.size());
	for (size_t i = 0; i < objects.size(); ++i)
	{
		endpoints.push_back(EndPoint{aabbs[i].Min.x, i, true});
		endpoints.push_back(EndPoint{aabbs[i].Max.x, i, false});
	}

	std::sort(endpoints.begin(), endpoints.end());
	std::unordered_set<uint64_t> checked;
	checked.reserve(objects.size());
	std::vector<size_t> activeList;
	activeList.reserve(objects.size());
	for (const EndPoint& ep : endpoints)
	{
		if (ep.bIsMin)
		{
			for (size_t index : activeList)
			{
				uint64_t pairKey = MakePairKey(ep.ObjectIndex, index);
				if (checked.insert(pairKey).second)
				{
					outPotentialCollisions.emplace_back(ep.ObjectIndex, index);
				}
			}
			activeList.push_back(ep.ObjectIndex);
		}
		else
		{
			auto it = std::find(activeList.begin(), activeList.end(), ep.ObjectIndex);
			if (it != activeList.end())
			{
				activeList.erase(it);
			}
		}
	}
}