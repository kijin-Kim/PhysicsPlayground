#include "Rigidbody.h"
#include <glm/gtx/exterior_product.hpp>

Rigidbody::Rigidbody()
	: velocity_(0.0f, 0.0f)
	, force_(0.0f, 0.0f)
	, angularVelocity_(0.0f)
	, torque_(0.0f)
	, invInertia_(1.0f)
	, invMass_(1.0f)
	, linearDamping_(0.01f)
	, angularDamping_(0.01f)
	, elasticity_(0.5f)
	, friction_(0.5f)
	, gravityScale_(1.0f)
	, bIsSleeping_(false)
	, sleepTimer_(0.0f)
{
}

void Rigidbody::IntegrateForce(float deltaTime)
{
	constexpr float GLOBAL_GRAVITY_SCALE = 100.0f;
	glm::vec2 acceleration = force_ * invMass_ - glm::vec2(0.0f, GLOBAL_GRAVITY_SCALE * gravityScale_);
	velocity_ += acceleration * deltaTime;

	const float angularAcceleration = torque_ * invInertia_;
	angularVelocity_ += angularAcceleration * deltaTime;

	force_ = {0.0f, 0.0f};
	torque_ = 0.0f;
}
void Rigidbody::ApplyDamping(float deltaTime)
{
	velocity_ *= glm::exp(-linearDamping_ * deltaTime);
	angularVelocity_ *= glm::exp(-angularDamping_ * deltaTime);
}

void Rigidbody::IntegrateVelocity(float deltaTime, glm::vec2& outPosition, float& outRotation)
{
	outPosition += velocity_ * deltaTime;
	outRotation += angularVelocity_ * deltaTime;
}

void Rigidbody::SetMass(float mass)
{
	invMass_ = mass <= 0.0f ? 0.0f : 1.0f / mass;
}

float Rigidbody::GetMass() const
{
	return invMass_ <= 0.0f ? 0.0f : 1.0f / invMass_;
}

float Rigidbody::GetInvMass() const
{
	return invMass_;
}

void Rigidbody::SetInertia(float inertia)
{
	invInertia_ = inertia <= 0.0f ? 0.0f : 1.0f / inertia;
}

float Rigidbody::GetInertia() const
{
	return invInertia_ <= 0.0f ? 0.0f : 1.0f / invInertia_;
}

void Rigidbody::ApplyImpulse(glm::vec2 impulse, glm::vec2 centroid, glm::vec2 contactPoint)
{
	if (invMass_ <= 0.0f )
	{
		return;
	}
	velocity_ += impulse * invMass_;
	glm::vec2 r = contactPoint - centroid;
	angularVelocity_ += glm::cross(r, impulse) * invInertia_;
}
void Rigidbody::TrySleepOrWake(float deltaTime)
{
	constexpr float SLEEP_VELOCITY_THRESHOLD = 0.1f;
	constexpr float SLEEP_ANGULAR_VELOCITY_THRESHOLD = 0.1f;
	constexpr float SLEEP_TIME_THRESHOLD = 100.0f;

	if (glm::length(velocity_) < SLEEP_VELOCITY_THRESHOLD
		&& glm::abs(angularVelocity_) < SLEEP_ANGULAR_VELOCITY_THRESHOLD)
	{
		sleepTimer_ += deltaTime;
		if (sleepTimer_ >= SLEEP_TIME_THRESHOLD)
		{
			bIsSleeping_ = true;
		}
	}
	else
	{
		sleepTimer_ = 0.0f;
		bIsSleeping_ = false;
	}
}
