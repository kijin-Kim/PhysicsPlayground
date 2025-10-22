#include "MainLayer.h"

#include "GLFW/glfw3.h"

#include <memory>

#include "Physics/Object.h"

#include "Renderer/Primitives/CircleMesh.h"
#include "Renderer/Primitives/PrimitiveVertex.h"
#include "Renderer/Primitives/RectangleMesh.h"
#include "Renderer/Primitives/TriangleMesh.h"

#include "glm/ext/matrix_transform.hpp"

#include <iostream>

constexpr glm::vec4 color1 = glm::vec4(1.0f, 0.5f, 0.2f, 1.0f);
constexpr glm::vec4 color2 = glm::vec4(0.2f, 0.5f, 1.0f, 1.0f);
constexpr glm::vec4 color3 = glm::vec4(0.5f, 0.2f, 0.2f, 1.0f);

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
	glm::vec2 point = Support(verts1, verts2, direction);
	outSimplex.push_back(point);
	direction = -point;
	for (int i = 0; i < 100; ++i)
	{
		point = Support(verts1, verts2, direction);
		if (glm::dot(point, direction) <= 0)
		{
			return false;
		}
		outSimplex.push_back(point);
		if (DoSimplex(outSimplex, direction))
		{
			return true;
		}
	}

	return false;
}

Manifold Collide(const Object& obj1, const Object& obj2)
{

	glm::mat4 transform1 = obj1.GetTransform();
	const std::array<float, 8> verts1 = PrimitiveVertex::RectangleVertices;
	std::vector<glm::vec2> transformedVerts1;
	for (size_t i = 0; i < verts1.size(); i += 2)
	{
		glm::vec4 vertex = transform1 * glm::vec4(verts1[i], verts1[i + 1], 0.0f, 1.0f);
		transformedVerts1.emplace_back(vertex.x, vertex.y);
	}

	glm::mat4 transform2 = obj2.GetTransform();
	const std::array<float, 8> verts2 = PrimitiveVertex::RectangleVertices;
	std::vector<glm::vec2> transformedVerts2;
	for (size_t i = 0; i < verts2.size(); i += 2)
	{
		glm::vec4 vertex = transform2 * glm::vec4(verts2[i], verts2[i + 1], 0.0f, 1.0f);
		transformedVerts2.emplace_back(vertex.x, vertex.y);
	}

	std::vector<glm::vec2> simplex;
	Manifold manifold;
	manifold.bHit = false;
	if (!GJK(transformedVerts1, transformedVerts2, simplex))
	{
		return manifold;
	}

	manifold = EPA(simplex, transformedVerts1, transformedVerts2);
	return manifold;
}

MainLayer::MainLayer()
{
	Object& o1 = objects_.emplace_back(std::make_unique<RectangleMesh>(color1, DrawMode::Lines));
	o1.GetRigidbody().SetMass(0.0f);
	o1.SetPosition(glm::vec2(-30.0f, -30.0f));
	Object& o2 = objects_.emplace_back(std::make_unique<RectangleMesh>(color2, DrawMode::Lines));
	o2.GetRigidbody().SetMass(0.0f);
	o2.SetPosition(glm::vec2(30.0f, 30.0f));
}

void MainLayer::OnInit()
{
	for (Object& object : objects_)
	{
		object.OnInit();
	}
}

void MainLayer::OnUpdate(float deltaTime)
{
	for (Object& object : objects_)
	{
		object.OnUpdate(deltaTime);
	}

	// Object& o2 = objects_[1];
	// glm::vec2 position = o2.GetPosition();
	// position.x += cos(static_cast<float>(glfwGetTime())) * 50.0f * deltaTime;
	// o2.SetPosition(position);
	// float rotation = o2.GetRotation();
	// rotation += glm::radians(90.0f) * deltaTime;
	// o2.SetRotation(rotation);

	for (size_t i = 0; i < objects_.size(); ++i)
	{
		for (size_t j = i + 1; j < objects_.size(); ++j)
		{
			Manifold manifold = Collide(objects_[i], objects_[j]);
			if (manifold.bHit)
			{
				objects_[i].GetMesh()->SetColor(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
				objects_[j].GetMesh()->SetColor(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
				std::cout << manifold.Penetration << std::endl;
			}
			else
			{
				objects_[i].GetMesh()->SetColor(color1);
				objects_[j].GetMesh()->SetColor(color2);
			}
		}
	}
}

void MainLayer::OnRender(Renderer& renderer)
{
	for (Object& object : objects_)
	{
		object.OnRender(renderer);
	}
}