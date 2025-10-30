#include "AABB.h"

AABB AABB::Compute(const std::vector<glm::vec2>& points)
{
	AABB box;
	box.Min = glm::vec2(FLT_MAX);
	box.Max = glm::vec2(-FLT_MAX);

	for (const glm::vec2& p : points)
	{
		box.Min = glm::min(box.Min, p);
		box.Max = glm::max(box.Max, p);
	}
	return box;
}

AABB::AABB(const glm::vec2& min, const glm::vec2& max)
	: Min(min)
	, Max(max)
{
}

bool AABB::Overlaps(const AABB& other) const
{
	return (Min.x <= other.Max.x && Max.x >= other.Min.x) &&
		(Min.y <= other.Max.y && Max.y >= other.Min.y);
}

bool AABB::Contains(const AABB& other) const
{
	return (other.Min.x >= Min.x && other.Max.x <= Max.x) &&
		(other.Min.y >= Min.y && other.Max.y <= Max.y);
}

void AABB::Draw(Renderer& renderer, const glm::vec4& color) const
{
	const glm::vec2 center = 0.5f * (Min + Max);
	const glm::vec2 size = Max - Min;
	renderer.DrawRectangle(center, 0.0f, size, color, true);
}