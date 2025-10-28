#pragma once
#include <glm/glm.hpp>


class Rigidbody
{
public:
	Rigidbody();
	void Integrate(float deltaTime, glm::vec2& outPosition);
	void AddForce(glm::vec2 force);
	void SetMass(float mass);
	float GetInvMass() const;
	glm::vec2 GetVelocity() const { return velocity_; }
	void SetGravityScale(float scale) { gravityScale_ = scale; }
	void AddVelocity(glm::vec2 velocity);
	void ApplyImpulse(glm::vec2 impulse);


private:
	glm::vec2 velocity_;
	glm::vec2 accumForce_;
	float invMass_;
	float damping_; // 댐핑 계수
	float gravityScale_;
};