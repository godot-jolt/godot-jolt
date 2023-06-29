#pragma once

#include "joints/jolt_joint_impl_3d.hpp"

class JoltHingeJointImpl3D final : public JoltJointImpl3D {
public:
	JoltHingeJointImpl3D(
		JoltBodyImpl3D* p_body_a,
		JoltBodyImpl3D* p_body_b,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	PhysicsServer3D::JointType get_type() const override {
		return PhysicsServer3D::JOINT_TYPE_HINGE;
	}

	double get_param(PhysicsServer3D::HingeJointParam p_param) const;

	void set_param(PhysicsServer3D::HingeJointParam p_param, double p_value, bool p_lock = true);

	bool get_flag(PhysicsServer3D::HingeJointFlag p_flag) const;

	void set_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled, bool p_lock = true);

	void rebuild(bool p_lock = true) override;

private:
	static JPH::Constraint* build_hinge(
		JPH::Body* p_jolt_body_a,
		JPH::Body* p_jolt_body_b,
		const Transform3D& p_shifted_ref_a,
		const Transform3D& p_shifted_ref_b,
		float p_limit
	);

	static JPH::Constraint* build_fixed(
		JPH::Body* p_jolt_body_a,
		JPH::Body* p_jolt_body_b,
		const Transform3D& p_shifted_ref_a,
		const Transform3D& p_shifted_ref_b
	);

	bool is_fixed() const { return limit_lower == limit_upper; }

	void update_motor_state();

	void update_motor_velocity();

	void update_motor_limit();

	void limits_changed(bool p_lock = true);

	void motor_state_changed();

	void motor_speed_changed();

	void motor_limit_changed();

	double limit_lower = 0.0;

	double limit_upper = 0.0;

	double motor_target_speed = 0.0f;

	double motor_max_impulse = 0.0;

	bool use_limits = false;

	bool motor_enabled = false;
};
