#include "CollisionSystem.h"
#include <iostream>
#include <mutex>
#include <span>
#include <thread>

#include "Object.h"
#include "BroadPhase/BroadPhase.h"

#include "Core/JobSystem.h"


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

	struct CollisionTask
	{
		size_t ObjectIndex1;
		size_t ObjectIndex2;
		Manifold Manifold;
	};

	std::vector<CollisionTask> collisionTasks;
	std::mutex collisionTasksMutex;

	JobSystem jobSystem;
	jobSystem.Prepare(potentialCollisions.size());
	for (const std::pair<size_t, size_t>& pair : potentialCollisions)
	{
		jobSystem.Enqueue([&, pair]()
		{
			const size_t objectIndex1 = pair.first;
			const size_t objectIndex2 = pair.second;
			std::vector<glm::vec2> vert1 = objects[objectIndex1].GetShape()->GetVertices();
			std::vector<glm::vec2> vert2 = objects[objectIndex2].GetShape()->GetVertices();
			for (glm::vec2& point : vert1)
			{
				point = objects[objectIndex1].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
			}
			for (glm::vec2& point : vert2)
			{
				point = objects[objectIndex2].GetTransform() * glm::vec4(point, 0.0f, 1.0f);
			}

			Manifold manifold = Collide(vert1, vert2);
			if (manifold.bHit)
			{
				std::lock_guard<std::mutex> lock(collisionTasksMutex);
				collisionTasks.emplace_back(objectIndex1, objectIndex2, manifold);
			}
		});
	}
	jobSystem.WaitAll();

	for (Object& object : objects)
	{
		object.GetShape()->SetColor(color2);
	}

	for (const CollisionTask& ct : collisionTasks)
	{
		ResolveCollision(objects[ct.ObjectIndex1], objects[ct.ObjectIndex2], ct.Manifold);
		objects[ct.ObjectIndex1].GetShape()->SetColor(color1);
		objects[ct.ObjectIndex2].GetShape()->SetColor(color1);
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
	body1.ApplyImpulse(-impulse, obj1.GetPosition(), manifold.ContactPoint);
	body2.ApplyImpulse(impulse, obj2.GetPosition(), manifold.ContactPoint);

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

CollisionSystem::Edge CollisionSystem::FindClosestEdge(const std::vector<SupportPoint>& simplex)
{
	Edge closestEdge;
	closestEdge.Distance = FLT_MAX;

	for (size_t i = 0; i < simplex.size(); ++i)
	{
		size_t j = (i + 1) % simplex.size();
		glm::vec2 a = simplex[i].MinkowskiPoint;
		glm::vec2 b = simplex[j].MinkowskiPoint;
		glm::vec2 edge = b - a;
		glm::vec2 normal = glm::normalize(glm::vec2(-edge.y, edge.x));
		if (glm::dot(normal, a) < 0)
		{
			normal = -normal;
		}

		float distance = glm::dot(normal, a);

		if (distance < closestEdge.Distance)
		{
			closestEdge.StartIndex = static_cast<int>(i);
			closestEdge.EndIndex = static_cast<int>(j);
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

CollisionSystem::SupportPoint CollisionSystem::Support(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, const glm::vec2& dir)
{
	const glm::vec2 point1 = Furthest(verts1, dir);
	const glm::vec2 point2 = Furthest(verts2, -dir);
	return SupportPoint{point1 - point2, point1, point2};
}

bool CollisionSystem::DoSimplex(std::vector<SupportPoint>& outSimplex, glm::vec2& direction)
{
	if (outSimplex.size() == 2)
	{
		// 선분 AB를 기준으로 어느쪽에 원점이 있는지 판단
		glm::vec2 a = outSimplex[1].MinkowskiPoint;
		glm::vec2 b = outSimplex[0].MinkowskiPoint;
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
		glm::vec2 a = outSimplex[2].MinkowskiPoint;
		glm::vec2 b = outSimplex[1].MinkowskiPoint;
		glm::vec2 c = outSimplex[0].MinkowskiPoint;

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

CollisionSystem::Manifold CollisionSystem::EPA(const std::vector<SupportPoint>& simplex, const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2)
{
	std::vector<SupportPoint> polytope = simplex;

	for (int iteration = 0; iteration < 100; ++iteration)
	{
		Edge edge = FindClosestEdge(polytope);
		const SupportPoint supportPoint = Support(verts1, verts2, edge.Normal);
		const float distance = glm::dot(supportPoint.MinkowskiPoint, edge.Normal);
		if (distance - edge.Distance < 0.001f)
		{
			Manifold manifold;
			manifold.bHit = true;
			manifold.Normal = edge.Normal;
			manifold.Penetration = distance;

			const SupportPoint& s1 = polytope[edge.StartIndex];
			const SupportPoint& s2 = polytope[edge.EndIndex];
			glm::vec2 ab = s2.MinkowskiPoint - s1.MinkowskiPoint;
			float t = glm::dot(-s1.MinkowskiPoint, ab) / glm::dot(ab, ab);
			t = glm::clamp(t, 0.0f, 1.0f);

			glm::vec2 contactPointA = s1.PointA + t * (s2.PointA - s1.PointA);
			glm::vec2 contactPointB = s1.PointB + t * (s2.PointB - s1.PointB);
			manifold.ContactPoint = 0.5f * (contactPointA + contactPointB);
			return manifold;
		}
		polytope.insert(polytope.begin() + edge.EndIndex, supportPoint);
	}

	Manifold noHit;
	noHit.bHit = false;
	return noHit;
}

bool CollisionSystem::GJK(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, std::vector<SupportPoint>& outSimplex)
{
	glm::vec2 direction(1.0f, 0.0f);
	SupportPoint supportPoint = Support(verts1, verts2, direction);
	outSimplex.push_back(supportPoint);
	direction = -supportPoint.MinkowskiPoint;
	for (int i = 0; i < 100; ++i)
	{
		supportPoint = Support(verts1, verts2, direction);
		if (glm::dot(supportPoint.MinkowskiPoint, direction) <= 0)
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
	std::vector<SupportPoint> simplex;
	Manifold manifold;
	manifold.bHit = false;
	if (!GJK(verts1, verts2, simplex))
	{
		return manifold;
	}

	manifold = EPA(simplex, verts1, verts2);
	return manifold;
}