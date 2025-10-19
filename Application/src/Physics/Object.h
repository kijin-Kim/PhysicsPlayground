#pragma once
#include <memory>
#include <glm/glm.hpp>

#include "Rigidbody.h"


class Renderer;
class IPrimitive;

class Object
{
public:
	Object() = default;
	explicit Object(std::unique_ptr<IPrimitive> mesh);
	void OnInit();
	void OnUpdate(float deltaTime);
	void OnRender(Renderer& renderer);

private:
	std::unique_ptr<IPrimitive> mesh_;
	Rigidbody rigidBody_;
	glm::vec2 position_;
	float rotation_;
	glm::vec2 scale_;
	glm::mat4 transform_;
};