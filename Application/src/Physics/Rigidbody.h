#pragma once
#include <glm/glm.hpp>

class Rigidbody
{
public:
	Rigidbody();
	void IntegrateForce(float deltaTime);
	void ApplyDamping(float deltaTime);
	void IntegrateVelocity(float deltaTime, glm::vec2& outPosition, float& outRotation);

	void SetMass(float mass);
	float GetMass() const;
	float GetInvMass() const;
	void SetInertia(float inertia);
	float GetInertia() const;
	float GetInvInertia() const { return invInertia_; }
	float GetElasticity() const { return elasticity_; }
	float GetFriction() const { return friction_; }
	glm::vec2 GetVelocity() const { return velocity_; }
	float GetAngularVelocity() const { return angularVelocity_; }

	void SetGravityScale(float scale) { gravityScale_ = scale; }
	void ApplyImpulse(glm::vec2 impulse, glm::vec2 centroid, glm::vec2 contactPoint);

	void TrySleepOrWake(float deltaTime);
	bool IsSleeping() const { return bIsSleeping_; }
	bool IsStatic() const { return invMass_ <= 0.0f && invInertia_ <= 0.0f; }

private:
	glm::vec2 velocity_;
	glm::vec2 force_;
	float angularVelocity_;
	float torque_;
	float invInertia_;
	float invMass_;
	float linearDamping_; // 댐핑 계수
	float angularDamping_;
	float elasticity_;
	float friction_;
	float gravityScale_;

	bool bIsSleeping_;
	float sleepTimer_;
};