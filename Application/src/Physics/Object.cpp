#include "Object.h"

#include "Renderer/Renderer.h"
#include "Renderer/Primitives/Primitive.h"

#include "glm/ext/matrix_transform.hpp"

Object::Object(std::unique_ptr<IPrimitive> mesh)
	: mesh_(std::move(mesh))
	, rigidBody_()
	, position_(0.0f, 0.0f)
	, rotation_(0.0f)
	, scale_(100.0f, 100.0f)
	, transform_(1.0f)
{
}

void Object::OnInit()
{
	if (mesh_)
	{
		mesh_->UploadData();
	}
}

void Object::OnUpdate(float deltaTime)
{
	rigidBody_.Integrate(deltaTime, position_);
	transform_ = glm::translate(glm::mat4(1.0f), glm::vec3(position_, 0.0f));
	transform_ = glm::rotate(transform_, rotation_, glm::vec3(0.0f, 0.0f, 1.0f));
	transform_ = glm::scale(transform_, glm::vec3(scale_, 1.0f));
}

void Object::OnRender(Renderer& renderer)
{
	renderer.DrawPrimitive(mesh_, transform_);
}