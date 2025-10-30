#include "CollisionSystem.h"
#include <iostream>
#include "Object.h"
#include "BroadPhase/BroadPhase.h"


void CollisionSystem::Update(std::vector<Object>& objects, float deltaTime)
{
	for (Object& object : objects)
	{
		object.OnUpdate(deltaTime);
	}

	std::vector<std::pair<size_t, size_t> > potentialCollisions;
	if (broadPhase_)
	{
		broadPhase_->ComputePotentialCollisions(objects, potentialCollisions);
	}

	std::vector<bool> collidedObjects(objects.size());
	std::cout << "Potential Collisions: " << potentialCollisions.size() << std::endl;
	for (const std::pair<size_t, size_t>& pair : potentialCollisions)
	{
		const size_t i = pair.first;
		const size_t j = pair.second;
		std::vector<glm::vec2> vert1 = objects[i].GetShape()->GetVertices();
		std::vector<glm::vec2> vert2 = objects[j].GetShape()->GetVertices();
		for (glm::vec2& point : vert1)
		{
			point = objects[i].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		for (glm::vec2& point : vert2)
		{
			point = objects[j].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}

		Manifold manifold = Collide(vert1, vert2);
		if (manifold.bHit)
		{
			ResolveCollision(objects[i], objects[j], manifold);
			collidedObjects[i] = true;
			collidedObjects[j] = true;
		}
	}

	for (size_t i = 0; i < objects.size(); ++i)
	{
		if (collidedObjects[i])
		{
			objects[i].GetShape()->SetColor(color3);
		}
		else
		{
			objects[i].GetShape()->SetColor(color2);
		}
	}
}

void CollisionSystem::SetBroadPhaseAlgorithm(std::unique_ptr<IBroadPhase> broadPhase)
{
	broadPhase_ = std::move(broadPhase);
}

void CollisionSystem::DrawDebug(const std::vector<Object>& objects, Renderer& renderer)
{
	for (const Object& object : objects)
	{
		std::vector<glm::vec2> vert1 = object.GetShape()->GetVertices();
		for (glm::vec2& point : vert1)
		{
			point = object.GetTransform() * glm::vec4(point, 0.0f, 1.0f);
		}
		AABB aabb = AABB::Compute(vert1);
		aabb.Draw(renderer, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
	}

	if (broadPhase_)
	{
		broadPhase_->DrawDebug(renderer);
	}
}

void CollisionSystem::ResolveCollision(Object& obj1, Object& obj2, const Manifold& manifold)
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

CollisionSystem::Edge CollisionSystem::FindClosestEdge(const std::vector<glm::vec2>& simplex)
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

glm::vec2 CollisionSystem::Furthest(const std::vector<glm::vec2>& vertices, const glm::vec2& dir)
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

glm::vec2 CollisionSystem::Support(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, const glm::vec2& dir)
{
	const glm::vec2 point1 = Furthest(verts1, dir);
	const glm::vec2 point2 = Furthest(verts2, -dir);
	return point1 - point2;
}

bool CollisionSystem::DoSimplex(std::vector<glm::vec2>& outSimplex, glm::vec2& direction)
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

CollisionSystem::Manifold CollisionSystem::EPA(const std::vector<glm::vec2>& simplex, const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2)
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

bool CollisionSystem::GJK(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, std::vector<glm::vec2>& outSimplex)
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

CollisionSystem::Manifold CollisionSystem::Collide(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2)
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