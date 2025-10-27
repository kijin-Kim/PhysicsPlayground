#include "MainLayer.h"

#include "GLFW/glfw3.h"
#include "Renderer/Shapes.h"

#include "Physics/Object.h"
#include "Renderer/Renderer.h"


#include "glm/ext/matrix_transform.hpp"

#include <iostream>

constexpr glm::vec4 color1 = glm::vec4(1.0f, 0.5f, 0.2f, 1.0f);
constexpr glm::vec4 color2 = glm::vec4(0.2f, 0.5f, 1.0f, 1.0f);
constexpr glm::vec4 color3 = glm::vec4(0.5f, 0.2f, 0.2f, 1.0f);
constexpr glm::vec4 color4 = glm::vec4(0.71f, 0.49f, 0.72f, 1.0f);

void NormalizeToCenteroid(std::vector<glm::vec2>& outPoints)
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
	NormalizeToCenteroid(polygonVertices);

	Object& o2 = objects_.emplace_back();
	//o2.SetShape(std::make_unique<RectangleShape>(glm::vec2(100.0f, 100.0f)));
	o2.SetShape(std::make_unique<ConvexShape>(polygonVertices));
	o2.GetShape()->SetColor(color4);
	o2.GetRigidbody().SetMass(0.0f);
	o2.SetPosition(glm::vec2(30.0f, 30.0f));
}

void MainLayer::OnUpdate(float deltaTime)
{
	for (Object& object : objects_)
	{
		object.OnUpdate(deltaTime);
	}

	Object& o2 = objects_[1];
	glm::vec2 position = o2.GetPosition();
	position.x += cos(static_cast<float>(glfwGetTime())) * 50.0f * deltaTime;
	o2.SetPosition(position);
	float rotation = o2.GetRotation();
	rotation += glm::radians(90.0f) * deltaTime;
	o2.SetRotation(rotation);

	for (size_t i = 0; i < objects_.size(); ++i)
	{
		for (size_t j = i + 1; j < objects_.size(); ++j)
		{
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
				objects_[i].GetShape()->SetColor(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
				objects_[j].GetShape()->SetColor(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
				std::cout << manifold.Penetration << std::endl;
			}
			else
			{
				objects_[i].GetShape()->SetColor(color1);
				objects_[j].GetShape()->SetColor(color2);
			}
		}
	}
}

void MainLayer::OnRender(Renderer& renderer)
{
	renderer.BeginScene();

	for (Object& object : objects_)
	{
		object.OnRender(renderer);
	}
	renderer.EndScene();

}