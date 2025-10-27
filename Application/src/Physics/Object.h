#pragma once
#include <memory>
#include <glm/glm.hpp>
#include "Rigidbody.h"
#include "Renderer/Shapes.h"


class Renderer;

class Object
{
public:
	Object();
	void OnUpdate(float deltaTime);

	Rigidbody& GetRigidbody() { return rigidBody_; }
	void SetPosition(const glm::vec2& position) { position_ = position; }
	glm::vec2 GetPosition() const { return position_; }
	void SetRotation(float rotation) { rotation_ = rotation; }
	float GetRotation() const { return rotation_; }
	void SetShape(std::unique_ptr<Shape> shape);
	Shape* GetShape() const { return shape_.get(); }
	const glm::mat4& GetTransform() const { return transform_; }
	void OnRender(Renderer& renderer);

private:
	std::unique_ptr<Shape> shape_;
	Rigidbody rigidBody_;
	glm::vec2 position_;
	float rotation_;
	glm::mat4 transform_;
};