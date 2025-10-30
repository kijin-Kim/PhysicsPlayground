#pragma once
#include <memory>
#include <vector>

#include "glm/vec2.hpp"
#include "glm/vec4.hpp"


class Renderer;
class IBroadPhase;
class Object;


inline constexpr glm::vec4 color1 = glm::vec4(1.0f, 0.5f, 0.2f, 1.0f);
inline constexpr glm::vec4 color2 = glm::vec4(0.2f, 0.5f, 1.0f, 1.0f);
inline constexpr glm::vec4 color3 = glm::vec4(0.5f, 0.2f, 0.2f, 1.0f);
inline constexpr glm::vec4 color4 = glm::vec4(0.71f, 0.49f, 0.72f, 1.0f);


class CollisionSystem
{
public:
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

	void Update(std::vector<Object>& objects, float deltaTime);
	void SetBroadPhaseAlgorithm(std::unique_ptr<IBroadPhase> broadPhase);
	void DrawDebug(const std::vector<Object>& objects, Renderer& renderer);

private:
	void ResolveCollision(Object& obj1, Object& obj2, const Manifold& manifold);
	Edge FindClosestEdge(const std::vector<glm::vec2>& simplex);
	glm::vec2 Furthest(const std::vector<glm::vec2>& vertices, const glm::vec2& dir);
	glm::vec2 Support(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, const glm::vec2& dir);
	bool DoSimplex(std::vector<glm::vec2>& outSimplex, glm::vec2& direction);
	Manifold EPA(const std::vector<glm::vec2>& simplex, const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2);
	bool GJK(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2, std::vector<glm::vec2>& outSimplex);
	Manifold Collide(const std::vector<glm::vec2>& verts1, const std::vector<glm::vec2>& verts2);

private:
	std::unique_ptr<IBroadPhase> broadPhase_;
};