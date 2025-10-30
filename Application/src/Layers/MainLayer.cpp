#include "MainLayer.h"
#include <algorithm>


#include "GLFW/glfw3.h"
#include "Renderer/Shapes.h"

#include "Physics/Object.h"
#include "Renderer/Renderer.h"


#include "glm/ext/matrix_transform.hpp"

#include <iostream>
#include <unordered_set>

#include "Core/EventBus.h"
#include "GLM/gtx/hash.hpp"

#include "glm/gtx/integer.hpp"

constexpr glm::vec4 color1 = glm::vec4(1.0f, 0.5f, 0.2f, 1.0f);
constexpr glm::vec4 color2 = glm::vec4(0.2f, 0.5f, 1.0f, 1.0f);
constexpr glm::vec4 color3 = glm::vec4(0.5f, 0.2f, 0.2f, 1.0f);
constexpr glm::vec4 color4 = glm::vec4(0.71f, 0.49f, 0.72f, 1.0f);


AABB ComputeAABB(const std::vector<glm::vec2>& points)
{
	AABB aabb;
	aabb.Min = glm::vec2(FLT_MAX);
	aabb.Max = glm::vec2(-FLT_MAX);

	for (const glm::vec2& point : points)
	{
		aabb.Min = glm::min(aabb.Min, point);
		aabb.Max = glm::max(aabb.Max, point);
	}

	return aabb;
}

void DrawAABB(Renderer& renderer, const AABB& aabb, const glm::vec4& color)
{
	glm::vec2 center = (aabb.Min + aabb.Max) * 0.5f;
	glm::vec2 size = aabb.Max - aabb.Min;
	renderer.DrawRectangle(center, 0.0f, size, color, true);
}


void NormalizeToCentroid(std::vector<glm::vec2>& outPoints)
{
	glm::vec2 centroid(0.0f);
	float area = 0.0f;
	for (int i = 0; i < outPoints.size(); ++i)
	{
		int j = (i + 1) % outPoints.size();

		// 벡터 외적을 이용한 다각형의 면적 계산 누적
		const float cross = outPoints[i].x * outPoints[j].y - outPoints[j].x * outPoints[i].y;
		area += cross * 0.5f;

		// 무게중심 계산 누적
		const glm::vec2 weight = (outPoints[i] + outPoints[j]);
		centroid += weight * cross;
	}

	centroid /= (6.0f * area);
	for (glm::vec2& point : outPoints)
	{
		point -= centroid;
	}
}

struct Edge
{
	int Start;
	int End;
	glm::vec2 Normal;
	float Distance;
};

struct Manifold
{
	bool bHit;
	glm::vec2 Normal;
	float Penetration;
};

void ResolveCollision(Object& obj1, Object& obj2, const Manifold& manifold)
{
	Rigidbody& body1 = obj1.GetRigidbody();
	Rigidbody& body2 = obj2.GetRigidbody();
	const float totalInvMass = body1.GetInvMass() + body2.GetInvMass();
	if (totalInvMass <= 0.0f)
	{
		return;
	}

	const glm::vec2 relativeVelocity = body2.GetVelocity() - body1.GetVelocity();
	const float velocityAlongNormal = glm::dot(relativeVelocity, manifold.Normal);
	if (velocityAlongNormal > 0)
	{
		return;
	}

	const float e = 1.0f; // 반발 계수
	const float j = -(1.0f + e) * velocityAlongNormal / totalInvMass;
	const glm::vec2 impulse = j * manifold.Normal;

	body1.ApplyImpulse(-impulse);
	body2.ApplyImpulse(impulse);

	// Positional correction
	const float percent = 0.8f; // 보정 비율 (0~1)
	const float slop = 0.01f;   // 허용 오차
	const float totalSeparation = std::max(manifold.Penetration - slop, 0.0f) * percent;
	glm::vec2 correction = totalSeparation / totalInvMass * manifold.Normal;

	if (body1.GetInvMass() > 0)
	{
		obj1.SetPosition(obj1.GetPosition() - correction * body1.GetInvMass());
	}

	if (body2.GetInvMass() > 0)
	{
		obj2.SetPosition(obj2.GetPosition() + correction * body2.GetInvMass());
	}

}

Edge FindClosestEdge(const std::vector<glm::vec2>& simplex)
{
	Edge closestEdge;
	closestEdge.Distance = FLT_MAX;

	for (size_t i = 0; i < simplex.size(); ++i)
	{
		size_t j = (i + 1) % simplex.size();
		glm::vec2 a = simplex[i];
		glm::vec2 b = simplex[j];
		glm::vec2 edge = b - a;
		glm::vec2 normal = glm::normalize(glm::vec2(-edge.y, edge.x));
		if (glm::dot(normal, a) < 0)
		{
			normal = -normal;
		}

		float distance = glm::dot(normal, a);

		if (distance < closestEdge.Distance)
		{
			closestEdge.Start = static_cast<int>(i);
			closestEdge.End = static_cast<int>(j);
			closestEdge.Normal = normal;
			closestEdge.Distance = distance;
		}
	}

	return closestEdge;
}

glm::vec2 Furthest(const std::vector<glm::vec2>& vertices, const glm::vec2& dir)
{
	float maxDot = -FLT_MAX;
	glm::vec2 supportPoint(0.0f);

	for (const glm::vec2& vertex : vertices)
	{
		float dot = glm::dot(vertex, dir);
		if (dot > maxDot)
		{
			maxDot = dot;
			supportPoint = vertex;
		}
	}

	return supportPoint;
}

glm::vec2 Support(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, const glm::vec2& dir)
{
	const glm::vec2 point1 = Furthest(verts1, dir);
	const glm::vec2 point2 = Furthest(verts2, -dir);
	return point1 - point2;
}

bool DoSimplex(std::vector<glm::vec2>& outSimplex, glm::vec2& direction)
{
	if (outSimplex.size() == 2)
	{
		// 선분 AB를 기준으로 어느쪽에 원점이 있는지 판단
		glm::vec2 a = outSimplex[1];
		glm::vec2 b = outSimplex[0];
		glm::vec2 ab = b - a;
		glm::vec2 ao = -a;

		direction = glm::vec2(-ab.y, ab.x);
		if (glm::dot(direction, ao) < 0)
		{
			direction = -direction;
		}
	}
	else if (outSimplex.size() == 3)
	{
		glm::vec2 a = outSimplex[2];
		glm::vec2 b = outSimplex[1];
		glm::vec2 c = outSimplex[0];

		glm::vec2 ab = b - a;
		glm::vec2 ac = c - a;
		glm::vec2 ao = -a;

		glm::vec2 abPerp = glm::vec2(-ab.y, ab.x);
		if (glm::dot(abPerp, ac) > 0)
		{
			abPerp = -abPerp;
		}

		if (glm::dot(abPerp, ao) > 0)
		{
			outSimplex.erase(outSimplex.begin());
			direction = abPerp;
			return false;
		}

		glm::vec2 acPerp = glm::vec2(-ac.y, ac.x);
		if (glm::dot(acPerp, ab) > 0)
		{
			acPerp = -acPerp;
		}

		if (glm::dot(acPerp, ao) > 0)
		{
			outSimplex.erase(outSimplex.begin() + 1);
			direction = acPerp;
			return false;
		}

		return true;
	}

	return false;
}

Manifold EPA(const std::vector<glm::vec2>& simplex, const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2)
{
	std::vector<glm::vec2> polytope = simplex;

	for (int iteration = 0; iteration < 100; ++iteration)
	{
		Edge edge = FindClosestEdge(polytope);
		glm::vec2 supportPoint = Support(verts1, verts2, edge.Normal);
		float distance = glm::dot(supportPoint, edge.Normal);

		if (distance - edge.Distance < 0.001f)
		{
			Manifold manifold;
			manifold.bHit = true;
			manifold.Normal = edge.Normal;
			manifold.Penetration = distance;
			return manifold;
		}
		polytope.insert(polytope.begin() + edge.End, supportPoint);
	}

	Manifold noHit;
	noHit.bHit = false;
	return noHit;
}

bool GJK(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, std::vector<glm::vec2>& outSimplex)
{
	glm::vec2 direction(1.0f, 0.0f);
	glm::vec2 supportPoint = Support(verts1, verts2, direction);
	outSimplex.push_back(supportPoint);
	direction = -supportPoint;
	for (int i = 0; i < 100; ++i)
	{
		supportPoint = Support(verts1, verts2, direction);
		if (glm::dot(supportPoint, direction) <= 0)
		{
			return false;
		}
		outSimplex.push_back(supportPoint);
		if (DoSimplex(outSimplex, direction))
		{
			return true;
		}
	}

	return false;
}

Manifold Collide(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2)
{
	std::vector<glm::vec2> simplex;
	Manifold manifold;
	manifold.bHit = false;
	if (!GJK(verts1, verts2, simplex))
	{
		return manifold;
	}

	manifold = EPA(simplex, verts1, verts2);
	return manifold;
}


MainLayer::MainLayer(EventBus& eventBus)
	: ILayer(eventBus)
{
	eventBus_.Subscribe<PauseUpdateEvent>([this](const PauseUpdateEvent& e)
	{
		bShouldPauseUpdate = e.bPaused;
	});

	eventBus_.Subscribe<StepEvent>([this](const StepEvent& e)
	{
		Step(e.DeltaTime);
	});

	eventBus_.Subscribe<ChangeBroadPhaseAlgorithmEvent>([this](const ChangeBroadPhaseAlgorithmEvent& e)
	{
		broadPhaseAlgorithm_ = e.NewAlgorithm;
	});

	eventBus_.Subscribe<GridCellSizeChangedEvent>([this](const GridCellSizeChangedEvent& e)
	{
		cellSize = e.NewCellSize;
	});
}

void MainLayer::OnInit()
{
	Object& o1 = objects_.emplace_back();
	o1.SetShape(std::make_unique<RectangleShape>(glm::vec2(100.0f, 100.0f)));
	o1.GetShape()->SetColor(color1);
	o1.GetRigidbody().SetMass(0.0f);
	o1.SetPosition(glm::vec2(-30.0f, -30.0f));

	std::vector<glm::vec2> polygonVertices = {
		glm::vec2(50.0f, 70.0f),
		glm::vec2(70.0f, 30.0f),
		glm::vec2(100.0f, 20.0f),
		glm::vec2(120.0f, 70.0f)};
	NormalizeToCentroid(polygonVertices);

	Object& o2 = objects_.emplace_back();
	//o2.SetShape(std::make_unique<RectangleShape>(glm::vec2(100.0f, 100.0f)));
	o2.SetShape(std::make_unique<ConvexShape>(polygonVertices));
	o2.GetShape()->SetColor(color4);
	o2.GetRigidbody().SetMass(1.0f);
	o2.SetPosition(glm::vec2(30.0f, 80.0f));

	Object& floor = objects_.emplace_back();
	floor.SetShape(std::make_unique<RectangleShape>(glm::vec2(800.0f, 50.0f)));
	floor.GetShape()->SetColor(color1);
	floor.GetRigidbody().SetMass(0.0f);
	floor.SetPosition(glm::vec2(0.0f, -300.0f));

	Object& leftWall = objects_.emplace_back();
	leftWall.SetShape(std::make_unique<RectangleShape>(glm::vec2(50.0f, 600.0f)));
	leftWall.GetShape()->SetColor(color1);
	leftWall.GetRigidbody().SetMass(0.0f);
	leftWall.SetPosition(glm::vec2(-400.0f, 0.0f));

	Object& rightWall = objects_.emplace_back();
	rightWall.SetShape(std::make_unique<RectangleShape>(glm::vec2(50.0f, 600.0f)));
	rightWall.GetShape()->SetColor(color1);
	rightWall.GetRigidbody().SetMass(0.0f);
	rightWall.SetPosition(glm::vec2(400.0f, 0.0f));

	Object& ceiling = objects_.emplace_back();
	ceiling.SetShape(std::make_unique<RectangleShape>(glm::vec2(800.0f, 50.0f)));
	ceiling.GetShape()->SetColor(color1);
	ceiling.GetRigidbody().SetMass(0.0f);
	ceiling.SetPosition(glm::vec2(0.0f, 300.0f));

	// random circles
	for (int i = 0; i < 30; ++i)
	{
		Object& circleObj = objects_.emplace_back();
		circleObj.SetShape(std::make_unique<CircleShape>(15.0f));
		circleObj.GetShape()->SetColor(color2);
		circleObj.GetRigidbody().SetMass(1.0f);
		float x = static_cast<float>(rand() % 700 - 350);
		float y = static_cast<float>(rand() % 500 - 250);
		circleObj.SetPosition(glm::vec2(x, y));
	}
}

void MainLayer::OnUpdate(float deltaTime)
{
	if (!bShouldPauseUpdate)
	{
		Step(deltaTime);
	}
}

void MainLayer::OnRender(Renderer& renderer)
{
	renderer.BeginScene();
	for (Object& object : objects_)
	{
		object.OnRender(renderer);
		std::vector<glm::vec2> vert1 = object.GetShape()->GetVertices();
		for (glm::vec2& point : vert1)
		{
			point = object.GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		AABB aabb = ComputeAABB(vert1);
		DrawAABB(renderer, aabb, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
	}

	if (broadPhaseAlgorithm_ == BroadPhase::Type::Grid)
	{
		for (float x = -400.0f; x <= 400.0f; x += cellSize)
		{
			renderer.DrawRectangle(glm::vec2(x, 0.0f), 0.0f, glm::vec2(1.0f, 600.0f), glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
		}
		for (float y = -300.0f; y <= 300.0f; y += cellSize)
		{
			renderer.DrawRectangle(glm::vec2(0.0f, y), 0.0f, glm::vec2(800.0f, 1.0f), glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
		}
	}

	if (broadPhaseAlgorithm_ == BroadPhase::Type::Quadtree)
	{
		std::function<void(QuadTreeNode&)> drawNode = [&](QuadTreeNode& node)
		{
			DrawAABB(renderer, node.Bounds, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
			if (node.bIsDivided)
			{
				for (auto& child : node.Children)
				{
					drawNode(*child);
				}
			}
		};

		drawNode(rootNode_);
	}

	renderer.EndScene();

}

void MainLayer::Step(float deltaTime)
{
	for (Object& object : objects_)
	{
		object.OnUpdate(deltaTime);
	}

	std::vector<std::pair<size_t, size_t> > potentialCollisions;
	switch (broadPhaseAlgorithm_)
	{
	case BroadPhase::Type::Naive:
		BroadPhaseNaive(potentialCollisions);
		break;
	case BroadPhase::Type::Grid:
		BroadPhaseGrid(potentialCollisions);
		break;
	case BroadPhase::Type::Quadtree:
		BroadPhaseQuadtree(rootNode_, potentialCollisions);
		break;
	case BroadPhase::Type::SAP:
		BroadPhaseSAP(potentialCollisions);
		break;
	default:
		break;
	}

	std::vector<bool> collidedObjects(objects_.size());
	std::cout << "Potential Collisions: " << potentialCollisions.size() << std::endl;
	for (const std::pair<size_t, size_t>& pair : potentialCollisions)
	{
		const size_t i = pair.first;
		const size_t j = pair.second;
		std::vector<glm::vec2> vert1 = objects_[i].GetShape()->GetVertices();
		std::vector<glm::vec2> vert2 = objects_[j].GetShape()->GetVertices();
		for (glm::vec2& point : vert1)
		{
			point = objects_[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		for (glm::vec2& point : vert2)
		{
			point = objects_[j].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}

		Manifold manifold = Collide(vert1, vert2);
		if (manifold.bHit)
		{
			ResolveCollision(objects_[i], objects_[j], manifold);
			collidedObjects[i] = true;
			collidedObjects[j] = true;
		}
	}

	for (size_t i = 0; i < objects_.size(); ++i)
	{
		if (collidedObjects[i])
		{
			objects_[i].GetShape()->SetColor(color3);
		}
		else
		{
			objects_[i].GetShape()->SetColor(color2);
		}
	}
}

void MainLayer::BroadPhaseNaive(std::vector<std::pair<size_t, size_t> >& outPotentialCollisions)
{
	for (size_t i = 0; i < objects_.size(); ++i)
	{
		for (size_t j = i + 1; j < objects_.size(); ++j)
		{
			outPotentialCollisions.emplace_back(i, j);
		}
	}
}

uint64_t MakePairKey(size_t indexA, size_t indexB)
{
	return (static_cast<uint64_t>(std::min(indexA, indexB)) << 32) | static_cast<uint64_t>(std::max(indexA, indexB));
}

void MainLayer::BroadPhaseGrid(std::vector<std::pair<size_t, size_t> >& outPotentialCollisions)
{
	std::unordered_map<glm::ivec2, std::vector<size_t> > grid;
	grid.reserve(objects_.size());
	std::vector<AABB> aabbs(objects_.size());
	for (size_t i = 0; i < objects_.size(); ++i)
	{
		std::vector<glm::vec2> vert = objects_[i].GetShape()->GetVertices();
		for (glm::vec2& point : vert)
		{
			point = objects_[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		aabbs[i] = ComputeAABB(vert);

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

	std::unordered_set<uint64_t> checkedPairs;
	checkedPairs.reserve(objects_.size());
	outPotentialCollisions.reserve(objects_.size());
	for (const std::pair<const glm::ivec2, std::vector<size_t> >& cell : grid)
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

bool AABBOverlap(const AABB& a, const AABB& b)
{
	return (a.Min.x <= b.Max.x && a.Max.x >= b.Min.x) &&
		(a.Min.y <= b.Max.y && a.Max.y >= b.Min.y);
}

void Subdivide(QuadTreeNode& node)
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

bool AABBContains(const AABB& outer, const AABB& inner)
{
	return (inner.Min.x >= outer.Min.x && inner.Max.x <= outer.Max.x) &&
		(inner.Min.y >= outer.Min.y && inner.Max.y <= outer.Max.y);
}

void InsertNode(QuadTreeNode& node, size_t objectIndex, const AABB& aabb, const std::vector<AABB>& allAABBs, int depth)
{
	constexpr int MAX_OBJECTS_PER_NODE = 2;
	constexpr int MAX_DEPTH = 6;
	if (!AABBOverlap(node.Bounds, aabb))
	{
		return;
	}

	if (node.bIsDivided)
	{
		for (size_t i = 0; i < 4; ++i)
		{
			if (AABBContains(node.Children[i]->Bounds, aabb))
			{
				InsertNode(*node.Children[i], objectIndex, aabb, allAABBs, depth + 1);
				return;
			}
		}
		node.ObjectIndices.push_back(objectIndex);
		return;
	}

	node.ObjectIndices.push_back(objectIndex);
	if (node.ObjectIndices.size() > MAX_OBJECTS_PER_NODE && depth < MAX_DEPTH)
	{
		Subdivide(node);
		std::vector<size_t> remainings;
		for (size_t index : node.ObjectIndices)
		{
			bool bInserted = false;
			for (size_t i = 0; i < 4; ++i)
			{
				if (AABBContains(node.Children[i]->Bounds, allAABBs[index]))
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

void CollectPotentialCollisions(const QuadTreeNode& node, const std::vector<AABB>& aabbs, std::vector<size_t>& boundaryObjects, std::unordered_set<uint64_t>& checked, std::vector<std::pair<size_t, size_t> >& outPairs)
{
	// 걸친 객체들과 노드 내부 객체들끼리 검사
	for (size_t indexA : node.ObjectIndices)
	{
		const AABB& aabbA = aabbs[indexA];
		for (size_t indexB : boundaryObjects)
		{
			const AABB& aabbB = aabbs[indexB];
			if (!AABBOverlap(aabbA, aabbB))
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

void MainLayer::BroadPhaseQuadtree(QuadTreeNode& outRoot, std::vector<std::pair<size_t, size_t> >& outPotentialCollisions)
{
	std::vector<AABB> aabbs(objects_.size());
	for (size_t i = 0; i < objects_.size(); ++i)
	{
		std::vector<glm::vec2> vert = objects_[i].GetShape()->GetVertices();
		for (glm::vec2& point : vert)
		{
			point = objects_[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		aabbs[i] = ComputeAABB(vert);
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
	outRoot = QuadTreeNode();
	outRoot.Bounds = worldAABB;

	for (size_t i = 0; i < objects_.size(); ++i)
	{
		InsertNode(outRoot, i, aabbs[i], aabbs, 0);
	}

	std::unordered_set<uint64_t> checked;
	checked.reserve(objects_.size());
	std::vector<size_t> boundaryObjects;
	CollectPotentialCollisions(rootNode_, aabbs, boundaryObjects, checked, outPotentialCollisions);
}

void MainLayer::BroadPhaseSAP(std::vector<std::pair<size_t, size_t> >& outPotentialCollisions)
{
	BroadPhaseNaive(outPotentialCollisions);
}