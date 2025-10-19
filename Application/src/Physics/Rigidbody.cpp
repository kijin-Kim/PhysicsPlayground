#include "Rigidbody.h"

Rigidbody::Rigidbody()
	: velocity_(0.0f, 0.0f)
	, accumForce_(0.0f, 0.0f)
	, invMass_(1.0f)
	, damping_(0.98f)
{
}

void Rigidbody::Integrate(float deltaTime, glm::vec2& outPosition)
{
	if (invMass_ <= 0.0f)
	{
		return;
	}

	constexpr float gravityConstant = 100.0f;
	AddForce({0.0f, -gravityConstant * 1.0f / invMass_});
	const glm::vec2 acceleration = accumForce_ * invMass_;
	velocity_ += acceleration * deltaTime;
	velocity_ *= pow(damping_, deltaTime);
	outPosition += velocity_ * deltaTime;

	accumForce_ = {0.0f, 0.0f};
}

void Rigidbody::AddForce(glm::vec2 force)
{
	accumForce_ += force;
}

void Rigidbody::SetMass(float mass)
{
	invMass_ = mass <= 0.0f ? 0.0f : 1.0f / mass;
}

float Rigidbody::GetInvMass() const
{
	return invMass_;
}

void Rigidbody::AddVelocity(glm::vec2 velocity)
{
	velocity_ += velocity;
}

void Rigidbody::ApplyImpulse(glm::vec2 impulse)
{
	velocity_ += impulse * invMass_;
}