#include "Object.h"

#include "Renderer/Shapes.h"
#include "Renderer/Renderer.h"

#include "glm/ext/matrix_transform.hpp"

Object::Object()
	: rigidBody_()
	, position_(0.0f, 0.0f)
	, rotation_(0.0f)
	, transform_(1.0f)
{
}

void Object::OnUpdate(float deltaTime)
{
	rigidBody_.Integrate(deltaTime, position_);
	transform_ = glm::translate(glm::mat4(1.0f), glm::vec3(position_, 0.0f));
	transform_ = glm::rotate(transform_, rotation_, glm::vec3(0.0f, 0.0f, 1.0f));
}

void Object::SetShape(std::unique_ptr<Shape> shape)
{
	shape_ = std::move(shape);
}

void Object::OnRender(Renderer& renderer)
{
	if (shape_)
	{
		shape_->Draw(renderer, position_, rotation_);
	}
}