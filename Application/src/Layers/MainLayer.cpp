#include "MainLayer.h"
#include <memory>

#include "Physics/Object.h"

#include "Renderer/Primitives/CircleMesh.h"
#include "Renderer/Primitives/RectangleMesh.h"
#include "Renderer/Primitives/TriangleMesh.h"

#include "glm/ext/matrix_transform.hpp"

MainLayer::MainLayer()
{
	constexpr glm::vec4 color1 = glm::vec4(1.0f, 0.5f, 0.2f, 1.0f);
	constexpr glm::vec4 color2 = glm::vec4(0.2f, 0.5f, 1.0f, 1.0f);
	constexpr glm::vec4 color3 = glm::vec4(0.5f, 0.2f, 0.2f, 1.0f);

	objects_.emplace_back(std::make_unique<RectangleMesh>(color1));
	objects_.emplace_back(std::make_unique<TriangleMesh>(color2));
	objects_.emplace_back(std::make_unique<CircleMesh>(color3));
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
}

void MainLayer::OnRender(Renderer& renderer)
{
	for (Object& object : objects_)
	{
		object.OnRender(renderer);
	}
}