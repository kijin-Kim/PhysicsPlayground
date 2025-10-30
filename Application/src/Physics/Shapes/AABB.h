#pragma once
#include <vector>
#include "Renderer/Renderer.h"



struct AABB
{
	glm::vec2 Min;
	glm::vec2 Max;

	AABB() = default;
	AABB(const glm::vec2& min, const glm::vec2& max);

	bool Overlaps(const AABB& other) const;
	bool Contains(const AABB& other) const;

	static AABB Compute(const std::vector<glm::vec2>& points);
	void Draw(Renderer& renderer, const glm::vec4& color) const;
};