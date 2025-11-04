#pragma once
#include "Core/EventBus.h"
#include "Core/JobSystem.h"

#include <memory>
#include <vector>

#include "glm/vec2.hpp"
#include "glm/vec4.hpp"

class Renderer;
class IBroadPhase;
class Object;


class CollisionSystem
{
public:
	struct Edge
	{
		int StartIndex;
		int EndIndex;
		glm::vec2 Normal;
		float Distance;
	};

	struct SupportPoint
	{
		glm::vec2 MinkowskiPoint;
		glm::vec2 PointA;
		glm::vec2 PointB;
	};

	struct Manifold
	{
		bool bHit;
		glm::vec2 Normal;
		float Penetration;
		glm::vec2 ContactPoint;
	};


	struct CollisionTask
	{
		size_t ObjectIndex1;
		size_t ObjectIndex2;
		Manifold Manifold;
	};

public:
	CollisionSystem();
	~CollisionSystem();
	void IntegrateForces(std::vector<Object>& objects, float deltaTime);
	std::vector<std::pair<size_t, size_t>> BroadPhaseDetect(const std::vector<Object>& objects);
	std::vector<CollisionTask> NarrowPhaseDetect(std::vector<Object>& objects,
											   const std::vector<std::pair<size_t, size_t>>& potentialCollisions);
	void SolveVelocityContacts(std::vector<CollisionTask>& collisionTasks, std::vector<Object>& objects, float deltaTime);
	void SolvePositionContacts(std::vector<CollisionTask>& collisionTasks, std::vector<Object>& objects);
	void ApplyDamping(std::vector<Object>& objects, float deltaTime);
	void IntegrateVelocities(std::vector<Object>& objects, float deltaTime);
	void TrySleepOrWake(std::vector<Object>& objects, float deltaTime);
	void PhysicsStep(std::vector<Object>& objects, float deltaTime);
	void SetBroadPhaseAlgorithm(std::unique_ptr<IBroadPhase> broadPhase);
	void DrawDebug(const std::vector<Object>& objects, Renderer& renderer);

private:
	void ResolveCollision(Object& obj1, Object& obj2, const Manifold& manifold);
	Edge FindClosestEdge(const std::vector<SupportPoint>& simplex);
	glm::vec2 Furthest(const std::vector<glm::vec2>& vertices, const glm::vec2& dir);
	SupportPoint Support(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2,
						 const glm::vec2& dir);
	bool DoSimplex(std::vector<SupportPoint>& outSimplex, glm::vec2& direction);
	Manifold EPA(const std::vector<SupportPoint>& simplex, const std::vector<glm::vec2>& verts1,
				 const std::vector<glm::vec2>& verts2);
	bool GJK(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2,
			 std::vector<SupportPoint>& outSimplex);
	Manifold Collide(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2);

private:
	std::unique_ptr<IBroadPhase> broadPhase_;
	bool bDrawDebugAABBs_ = false;
	bool bDrawDebugBroadPhase_ = false;
	std::vector<EventBus::EventHandle> subscribedEvents_;
};