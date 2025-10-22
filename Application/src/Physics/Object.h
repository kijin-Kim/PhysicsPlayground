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

	Rigidbody& GetRigidbody() { return rigidBody_; }
	IPrimitive* GetMesh() const { return mesh_.get(); }
	void SetPosition(const glm::vec2& position) { position_ = position; }
	glm::vec2 GetPosition() const { return position_; }
	void SetRotation(float rotation) { rotation_ = rotation; }
	float GetRotation() const { return rotation_; }
	const glm::mat4& GetTransform() const { return transform_; }

  private:
	std::unique_ptr<IPrimitive> mesh_;
	Rigidbody rigidBody_;
	glm::vec2 position_;
	float rotation_;
	glm::vec2 scale_;
	glm::mat4 transform_;
};