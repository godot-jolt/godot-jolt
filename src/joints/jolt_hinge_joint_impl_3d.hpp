#pragma once

#include "joints/jolt_joint_impl_3d.hpp"

class JoltHingeJointImpl3D final : public JoltJointImpl3D {
public:
	JoltHingeJointImpl3D(
		JoltSpace3D* p_space,
		JoltBodyImpl3D* p_body_a,
		JoltBodyImpl3D* p_body_b,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	JoltHingeJointImpl3D(
		JoltSpace3D* p_space,
		JoltBodyImpl3D* p_body_a,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	PhysicsServer3D::JointType get_type() const override {
		return PhysicsServer3D::JOINT_TYPE_HINGE;
	}

	double get_param(PhysicsServer3D::HingeJointParam p_param) const;

	void set_param(PhysicsServer3D::HingeJointParam p_param, double p_value);

	bool get_flag(PhysicsServer3D::HingeJointFlag p_flag) const;

	void set_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled);

private:
	void limits_changed();

	double limit_lower = 0.0;

	double limit_upper = 0.0;

	double motor_max_impulse = 0.0;

	bool use_limits = false;
};
