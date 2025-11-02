#pragma once
#include <glm/glm.hpp>


class Rigidbody
{
public:
	Rigidbody();
	void Integrate(float deltaTime, glm::vec2& outPosition, float& outRotation);
	void AddForce(glm::vec2 force);
	void SetMass(float mass);
	float GetInvMass() const;
	glm::vec2 GetVelocity() const { return velocity_; }
	void SetInertia(float inertia) { inertia_ = inertia; }
	void SetGravityScale(float scale) { gravityScale_ = scale; }
	void AddVelocity(glm::vec2 velocity);
	void ApplyImpulse(glm::vec2 impulse, glm::vec2 centroid, glm::vec2 contactPoint);
	void ApplyImpulse(glm::vec2 impulse);




private:
	glm::vec2 velocity_;
	glm::vec2 force_;
	float angularVelocity_;
	float torque_;
	float inertia_;
	float invMass_;
	float damping_; // 댐핑 계수
	float gravityScale_;

};