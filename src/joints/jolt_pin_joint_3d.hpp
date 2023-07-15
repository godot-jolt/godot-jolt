#pragma once

#include "jolt_joint_3d.hpp"

class JoltPinJoint3D final : public JoltJoint3D {
	GDCLASS_NO_WARN(JoltPinJoint3D, JoltJoint3D)

private:
	static void _bind_methods();

public:
	Vector3 get_total_lambda_position() const;

private:
	void _configure(PhysicsBody3D* p_body_a, PhysicsBody3D* p_body_b) override;
};
