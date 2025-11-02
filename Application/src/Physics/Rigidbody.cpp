#include "Rigidbody.h"

Rigidbody::Rigidbody()
	: velocity_(0.0f, 0.0f)
	, force_(0.0f, 0.0f)
	, angularVelocity_(0.0f)
	, torque_(0.0f)
	, inertia_(1.0f)
	, invMass_(1.0f)
	, damping_(0.8f)
	, gravityScale_(1.0f)
{
}

void Rigidbody::Integrate(float deltaTime, glm::vec2& outPosition, float& outRotation)
{

	if (invMass_ <= 0.0f)
	{
		return;
	}

	constexpr float GLOBAL_GRAVITY_SCALE = 100.0f;
	AddForce({0.0f, -GLOBAL_GRAVITY_SCALE * 1.0f / invMass_ * gravityScale_});

	const glm::vec2 acceleration = force_ * invMass_;
	velocity_ += acceleration * deltaTime;
	velocity_ *= pow(damping_, deltaTime);
	outPosition += velocity_ * deltaTime;

	const float angularAcceleration = torque_ / inertia_;
	angularVelocity_ += angularAcceleration * deltaTime;
	angularVelocity_ *= pow(damping_, deltaTime);
	outRotation += angularVelocity_ * deltaTime;

	force_ = {0.0f, 0.0f};
	torque_ = 0.0f;
}


void Rigidbody::AddForce(glm::vec2 force)
{
	force_ += force;
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

void Rigidbody::ApplyImpulse(glm::vec2 impulse, glm::vec2 centroid, glm::vec2 contactPoint)
{
	velocity_ += impulse * invMass_;
	glm::vec2 r = contactPoint - centroid;
	angularVelocity_ += (r.x * impulse.y - r.y * impulse.x) / inertia_;
}

void Rigidbody::ApplyImpulse(glm::vec2 impulse)
{
	velocity_ += impulse * invMass_;
}