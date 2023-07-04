#pragma once

#include "joints/jolt_joint_impl_3d.hpp"

class JoltGeneric6DOFJointImpl3D final : public JoltJointImpl3D {
	using JoltAxis = JPH::SixDOFConstraintSettings::EAxis;

	enum Axis {
		AXIS_LINEAR_X = JoltAxis::TranslationX,
		AXIS_LINEAR_Y = JoltAxis::TranslationY,
		AXIS_LINEAR_Z = JoltAxis::TranslationZ,
		AXIS_ANGULAR_X = JoltAxis::RotationX,
		AXIS_ANGULAR_Y = JoltAxis::RotationY,
		AXIS_ANGULAR_Z = JoltAxis::RotationZ,
		AXIS_COUNT = JoltAxis::Num,
		AXES_LINEAR = AXIS_LINEAR_X,
		AXES_ANGULAR = AXIS_ANGULAR_X
	};

public:
	JoltGeneric6DOFJointImpl3D(
		JoltBodyImpl3D* p_body_a,
		JoltBodyImpl3D* p_body_b,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	PhysicsServer3D::JointType get_type() const override {
		return PhysicsServer3D::JOINT_TYPE_6DOF;
	}

	double get_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const;

	void set_param(
		Vector3::Axis p_axis,
		PhysicsServer3D::G6DOFJointAxisParam p_param,
		double p_value,
		bool p_lock = true
	);

	bool get_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const;

	void set_flag(
		Vector3::Axis p_axis,
		PhysicsServer3D::G6DOFJointAxisFlag p_flag,
		bool p_enabled,
		bool p_lock = true
	);

	void rebuild(bool p_lock = true) override;

private:
	void update_motor_state(int32_t p_axis);

	void update_motor_velocity(int32_t p_axis);

	void update_motor_limit(int32_t p_axis);

	void update_spring_stiffness(int32_t p_axis);

	void update_spring_damping(int32_t p_axis);

	void update_spring_equilibrium(int32_t p_axis);

	void limits_changed(bool p_lock = true);

	void motor_state_changed(int32_t p_axis);

	void motor_speed_changed(int32_t p_axis);

	void motor_limit_changed(int32_t p_axis);

	void spring_state_changed(int32_t p_axis);

	void spring_stiffness_changed(int32_t p_axis);

	void spring_damping_changed(int32_t p_axis);

	void spring_equilibrium_changed(int32_t p_axis);

	double limit_lower[AXIS_COUNT] = {};

	double limit_upper[AXIS_COUNT] = {};

	double motor_speed[AXIS_COUNT] = {};

	double motor_limit[AXIS_COUNT] = {};

	double spring_stiffness[AXIS_COUNT] = {};

	double spring_damping[AXIS_COUNT] = {};

	double spring_equilibrium[AXIS_COUNT] = {};

	bool use_limits[AXIS_COUNT] = {};

	bool motor_enabled[AXIS_COUNT] = {};

	bool spring_enabled[AXIS_COUNT] = {};
};
