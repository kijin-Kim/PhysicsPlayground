#include "CollisionSystem.h"

#include "Events.h"

#include <iostream>
#include <mutex>
#include <span>
#include <thread>

#include "BroadPhase/BroadPhase.h"
#include "Object.h"

#include "glm/gtx/exterior_product.hpp"
#include "tracy/Tracy.hpp"
#include "tracy/TracyC.h"

CollisionSystem::CollisionSystem()
{
	EventBus& eventBus = EventBus::GetInstance();
	subscribedEvents_.push_back(eventBus.Subscribe<DrawDebugAABBsEvent>([this](const DrawDebugAABBsEvent& e)
																		{ bDrawDebugAABBs_ = e.bEnabled; }));
	subscribedEvents_.push_back(eventBus.Subscribe<DrawDebugBroadPhaseEvent>([this](const DrawDebugBroadPhaseEvent& e)
																			 { bDrawDebugBroadPhase_ = e.bEnabled; }));
}
CollisionSystem::~CollisionSystem()
{
	for (const EventBus::EventHandle& handle : subscribedEvents_)
	{
		EventBus::GetInstance().Unsubscribe(handle);
	}
}

void CollisionSystem::IntegrateForces(std::vector<Object>& objects, float deltaTime)
{
	for (Object& object : objects)
	{
		if (!object.GetRigidbody().IsSleeping() && !object.GetRigidbody().IsStatic())
		{
			object.GetRigidbody().IntegrateForce(deltaTime);
		}
	}
}
std::vector<std::pair<size_t, size_t>> CollisionSystem::BroadPhaseDetect(const std::vector<Object>& objects)
{
	std::vector<std::pair<size_t, size_t>> potentialCollisions;
	if (broadPhase_)
	{
		ZoneScopedN("BroadPhase");
		broadPhase_->ComputePotentialCollisions(objects, potentialCollisions);
	}

	std::erase_if(potentialCollisions,
				  [&objects](const std::pair<size_t, size_t>& pair)
				  {
					  const Rigidbody& body1 = objects[pair.first].GetRigidbody();
					  const Rigidbody& body2 = objects[pair.second].GetRigidbody();
					  return (body1.IsSleeping() && body2.IsSleeping()) || (body1.IsStatic() && body2.IsStatic());
				  });
	return potentialCollisions;
}

std::vector<CollisionSystem::CollisionTask>
CollisionSystem::NarrowPhaseDetect(std::vector<Object>& objects,
								   const std::vector<std::pair<size_t, size_t>>& potentialCollisions)
{

	std::vector<CollisionTask> collisionTasks;
	std::mutex collisionTasksMutex;

	JobSystem& jobSystem = JobSystem::GetInstance();
	jobSystem.Prepare(potentialCollisions.size());
	{
		ZoneScopedN("NarrowPhase");
		for (const std::pair<size_t, size_t>& pair : potentialCollisions)
		{
			jobSystem.Enqueue(
				[&, pair]()
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
	}
	return collisionTasks;
}
void CollisionSystem::SolveVelocityContacts(std::vector<CollisionTask>& collisionTasks, std::vector<Object>& objects, float deltaTime)
{
	for (const CollisionTask& ct : collisionTasks)
	{
		Object& obj1 = objects[ct.ObjectIndex1];
		Object& obj2 = objects[ct.ObjectIndex2];
		Rigidbody& body1 = obj1.GetRigidbody();
		Rigidbody& body2 = obj2.GetRigidbody();

		const float totalInvMass = body1.GetInvMass() + body2.GetInvMass();
		if (totalInvMass <= 0.0f)
			continue;

		// --- 접점 벡터 계산 ---
		const glm::vec2 r1 = ct.Manifold.ContactPoint - obj1.GetPosition();
		const glm::vec2 r2 = ct.Manifold.ContactPoint - obj2.GetPosition();

		// --- 최신 velocity 읽기 ---
		glm::vec2 v1
			= body1.GetVelocity() + glm::vec2(-body1.GetAngularVelocity() * r1.y, body1.GetAngularVelocity() * r1.x);
		glm::vec2 v2
			= body2.GetVelocity() + glm::vec2(-body2.GetAngularVelocity() * r2.y, body2.GetAngularVelocity() * r2.x);
		glm::vec2 relativeVelocity = v2 - v1;

		const float velAlongNormal = glm::dot(relativeVelocity, ct.Manifold.Normal);
		if (velAlongNormal > 0.0f)
			continue; // 이미 분리 중이면 skip

		// --- 탄성 계산 ---
		float e = glm::max(body1.GetElasticity(), body2.GetElasticity());
		constexpr float RESTITUTION_THRESHOLD = 0.5f;
		if (glm::abs(velAlongNormal) < RESTITUTION_THRESHOLD)
			e = 0.0f;

		// --- 법선 방향 충격량 계산 ---
		const float angN1 = glm::cross(r1, ct.Manifold.Normal);
		const float angN2 = glm::cross(r2, ct.Manifold.Normal);
		float denomN = totalInvMass + angN1 * angN1 * body1.GetInvInertia() + angN2 * angN2 * body2.GetInvInertia();
		if (denomN <= 1e-8f)
			denomN = 1e-8f;

		constexpr float BAUMGARTE = 0.2f;
		constexpr float SLOP = 0.01f;
		const float penetration = glm::max(ct.Manifold.Penetration - SLOP, 0.0f);
		const float bias = -(BAUMGARTE / deltaTime) * penetration;
		const float jn = -((1.0f + e) * velAlongNormal + bias) / denomN;
		const glm::vec2 normalImpulse = jn * ct.Manifold.Normal;

		// --- 충격량 적용 (즉시 velocity 갱신) ---
		body1.ApplyImpulse(-normalImpulse, obj1.GetPosition(), ct.Manifold.ContactPoint);
		body2.ApplyImpulse(normalImpulse, obj2.GetPosition(), ct.Manifold.ContactPoint);

		// --- 다시 velocity 읽어서 마찰 계산 ---
		v1 = body1.GetVelocity() + glm::vec2(-body1.GetAngularVelocity() * r1.y, body1.GetAngularVelocity() * r1.x);
		v2 = body2.GetVelocity() + glm::vec2(-body2.GetAngularVelocity() * r2.y, body2.GetAngularVelocity() * r2.x);
		relativeVelocity = v2 - v1;

		glm::vec2 tangent = glm::vec2(-ct.Manifold.Normal.y, ct.Manifold.Normal.x);
		if (glm::dot(tangent, relativeVelocity) < 0.0f)
			tangent = -tangent;

		const float angT1 = glm::cross(r1, tangent);
		const float angT2 = glm::cross(r2, tangent);
		float denomT = totalInvMass + angT1 * angT1 * body1.GetInvInertia() + angT2 * angT2 * body2.GetInvInertia();
		if (denomT <= 1e-8f)
			denomT = 1e-8f;

		float jt = -glm::dot(relativeVelocity, tangent) / denomT;

		const float mu = glm::sqrt(body1.GetFriction() * body2.GetFriction());
		const float maxFriction = mu * jn;
		jt = glm::clamp(jt, -maxFriction, maxFriction);

		const glm::vec2 frictionImpulse = jt * tangent;
		body1.ApplyImpulse(-frictionImpulse, obj1.GetPosition(), ct.Manifold.ContactPoint);
		body2.ApplyImpulse(frictionImpulse, obj2.GetPosition(), ct.Manifold.ContactPoint);
	}
}
void CollisionSystem::SolvePositionContacts(std::vector<CollisionTask>& collisionTasks, std::vector<Object>& objects)
{
	for (const CollisionTask& ct : collisionTasks)
	{
		// Positional correction
		const float percent = 0.2f; // 보정 비율 (0~1)
		const float slop = 0.02f;	// 허용 오차
		const float penetration = std::max(ct.Manifold.Penetration - slop, 0.0f);
		if (penetration <= 0.0f)
		{
			continue;
		}

		Object& obj1 = objects[ct.ObjectIndex1];
		Object& obj2 = objects[ct.ObjectIndex2];
		Rigidbody& body1 = obj1.GetRigidbody();
		Rigidbody& body2 = obj2.GetRigidbody();
		const float totalInvMass = body1.GetInvMass() + body2.GetInvMass();
		if (totalInvMass <= 0.0f)
		{
			continue;
		}
		float correctionMag = (penetration / totalInvMass) * percent;
		glm::vec2 correction = correctionMag * ct.Manifold.Normal;
		if (body1.GetInvMass() > 0)
		{
			obj1.SetPosition(obj1.GetPosition() - correction * body1.GetInvMass());
		}

		if (body2.GetInvMass() > 0)
		{
			obj2.SetPosition(obj2.GetPosition() + correction * body2.GetInvMass());
		}
	}
}

void CollisionSystem::ApplyDamping(std::vector<Object>& objects, float deltaTime)
{
	for (Object& object : objects)
	{
		if (!object.GetRigidbody().IsSleeping() && !object.GetRigidbody().IsStatic())
		{
			object.GetRigidbody().ApplyDamping(deltaTime);
		}
	}
}
void CollisionSystem::IntegrateVelocities(std::vector<Object>& objects, float deltaTime)
{
	for (Object& object : objects)
	{
		if (!object.GetRigidbody().IsSleeping() && !object.GetRigidbody().IsStatic())
		{
			glm::vec2 position = object.GetPosition();
			float rotation = object.GetRotation();
			object.GetRigidbody().IntegrateVelocity(deltaTime, position, rotation);
			object.SetPosition(position);
			object.SetRotation(rotation);
		}
	}
}
void CollisionSystem::TrySleepOrWake(std::vector<Object>& objects, float deltaTime)
{
	for (Object& object : objects)
	{
		object.GetRigidbody().TrySleepOrWake(deltaTime);
	}
}

void CollisionSystem::PhysicsStep(std::vector<Object>& objects, float deltaTime)
{
	ZoneScoped;

	IntegrateForces(objects, deltaTime * 0.5f);
	std::vector<std::pair<size_t, size_t>> potentialCollisions = BroadPhaseDetect(objects);
	std::vector<CollisionTask> collisionTasks = NarrowPhaseDetect(objects, potentialCollisions);
	for (int i = 0; i < 8; ++i)
	{
		SolveVelocityContacts(collisionTasks, objects, deltaTime);
	}
	for (int i = 0; i < 3; ++i)
	{
		SolvePositionContacts(collisionTasks, objects);
	}

	IntegrateVelocities(objects, deltaTime);
	IntegrateForces(objects, deltaTime * 0.5f);
	ApplyDamping(objects, deltaTime);
	TrySleepOrWake(objects, deltaTime);

	// for (Object& object : objects)
	// {
	// 	if (object.GetRigidbody().IsSleeping())
	// 	{
	// 		object.GetShape()->SetColor(PColor::Grey);
	// 	}
	// 	else
	// 	{
	// 		object.GetShape()->SetColor(PColor::LavenderPurple);
	// 	}
	// }
}

void CollisionSystem::SetBroadPhaseAlgorithm(std::unique_ptr<IBroadPhase> broadPhase)
{
	broadPhase_ = std::move(broadPhase);
}

void CollisionSystem::DrawDebug(const std::vector<Object>& objects, Renderer& renderer)
{

	if (bDrawDebugAABBs_)
	{
		for (const Object& object : objects)
		{
			std::vector<glm::vec2> vert1 = object.GetShape()->GetVertices();
			for (glm::vec2& point : vert1)
			{
				point = object.GetTransform() * glm::vec4(point, 0.0f, 1.0f);
			}
			AABB aabb = AABB::Compute(vert1);
			aabb.Draw(renderer, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
		}
	}

	if (broadPhase_ && bDrawDebugBroadPhase_)
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

	const glm::vec2 r1 = manifold.ContactPoint - obj1.GetPosition();
	const glm::vec2 r2 = manifold.ContactPoint - obj2.GetPosition();
	glm::vec2 velocity1
		= body1.GetVelocity() + glm::vec2(-body1.GetAngularVelocity() * r1.y, body1.GetAngularVelocity() * r1.x);
	glm::vec2 velocity2
		= body2.GetVelocity() + glm::vec2(-body2.GetAngularVelocity() * r2.y, body2.GetAngularVelocity() * r2.x);
	glm::vec2 relativeVelocity = velocity2 - velocity1;
	const float velocityAlongNormal = glm::dot(relativeVelocity, manifold.Normal);
	if (velocityAlongNormal > 0.0f)
	{
		return;
	}

	float e = glm::max(body1.GetElasticity(), body2.GetElasticity());
	constexpr float RESTITUTION_THRESHOLD = 0.5f;
	if (glm::abs(velocityAlongNormal) < RESTITUTION_THRESHOLD)
	{
		e = 0.0f;
	}

	const float angN1 = glm::cross(r1, manifold.Normal);
	const float angN2 = glm::cross(r2, manifold.Normal);
	float denomN = totalInvMass + angN1 * angN1 * body1.GetInvInertia() + angN2 * angN2 * body2.GetInvInertia();
	denomN = denomN > 0.0f ? denomN : 1e-8f;
	const float jn = -(1.0f + e) * velocityAlongNormal / denomN;
	const glm::vec2 impulse = jn * manifold.Normal;
	body1.ApplyImpulse(-impulse, obj1.GetPosition(), manifold.ContactPoint);
	body2.ApplyImpulse(impulse, obj2.GetPosition(), manifold.ContactPoint);

	glm::vec2 tangent = glm::vec2(-manifold.Normal.y, manifold.Normal.x);
	if (glm::dot(tangent, relativeVelocity) < 0)
	{
		tangent = -tangent;
	}

	velocity1 = body1.GetVelocity() + glm::vec2(-body1.GetAngularVelocity() * r1.y, body1.GetAngularVelocity() * r1.x);
	velocity2 = body2.GetVelocity() + glm::vec2(-body2.GetAngularVelocity() * r2.y, body2.GetAngularVelocity() * r2.x);
	relativeVelocity = velocity2 - velocity1;
	const float angT1 = glm::cross(r1, tangent);
	const float angT2 = glm::cross(r2, tangent);
	float denomT = totalInvMass + angT1 * angT1 * body1.GetInvInertia() + angT2 * angT2 * body2.GetInvInertia();
	denomT = denomT > 0.0f ? denomT : 1e-8f;

	float jt = -glm::dot(relativeVelocity, tangent) / denomT;
	const float maxFriction = glm::sqrt(body1.GetFriction() * body2.GetFriction()) * jn;
	jt = glm::clamp(jt, -maxFriction, maxFriction);
	const glm::vec2 frictionImpulse = jt * tangent;
	body1.ApplyImpulse(-frictionImpulse, obj1.GetPosition(), manifold.ContactPoint);
	body2.ApplyImpulse(frictionImpulse, obj2.GetPosition(), manifold.ContactPoint);

	// Positional correction
	const float percent = 0.2f; // 보정 비율 (0~1)
	const float slop = 0.02f;	// 허용 오차
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

CollisionSystem::SupportPoint CollisionSystem::Support(const std::vector<glm::vec2>& verts1,
													   const std::vector<glm::vec2>& verts2, const glm::vec2& dir)
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

CollisionSystem::Manifold CollisionSystem::EPA(const std::vector<SupportPoint>& simplex,
											   const std::vector<glm::vec2>& verts1,
											   const std::vector<glm::vec2>& verts2)
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

bool CollisionSystem::GJK(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2,
						  std::vector<SupportPoint>& outSimplex)
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

CollisionSystem::Manifold CollisionSystem::Collide(const std::vector<glm::vec2>& verts1,
												   const std::vector<glm::vec2>& verts2)
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