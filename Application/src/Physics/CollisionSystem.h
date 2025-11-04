#pragma once
#include "Core/JobSystem.h"

#include <memory>
#include <vector>

#include "glm/vec2.hpp"
#include "glm/vec4.hpp"

class Renderer;
class IBroadPhase;
class Object;

inline constexpr glm::vec4 OceanBlue = glm::vec4(0.27f, 0.68f, 0.73f, 1.0f);
inline constexpr glm::vec4 CarrotOrange = glm::vec4(0.75f, 0.57f, 0.31f, 1.0f);
inline constexpr glm::vec4 LavenderPurple = glm::vec4(0.71f, 0.49f, 0.72f, 1.0f);
inline constexpr glm::vec4 RoyalBlue = glm::vec4(0.08f, 0.47f, 0.81f, 1.0f);
inline constexpr glm::vec4 LimeGreen = glm::vec4(0.64f, 0.73f, 0.36f, 1.0f);

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

public:
	void IntegrateForces(std::vector<Object>& objects, float deltaTime);
	void CollisionDetectAndResolve(std::vector<Object>& objects);
	void ApplyDamping(std::vector<Object>& objects, float deltaTime);
	void IntegrateVelocities(std::vector<Object>& objects, float deltaTime);
	void TrySleepOrWake(std::vector<Object>& objects, float deltaTime);
	void Update(std::vector<Object>& objects, float deltaTime);
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
	JobSystem jobSystem_;
};