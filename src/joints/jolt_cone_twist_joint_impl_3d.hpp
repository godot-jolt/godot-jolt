#pragma once

#include "joints/jolt_joint_impl_3d.hpp"

class JoltConeTwistJointImpl3D final : public JoltJointImpl3D {
public:
	JoltConeTwistJointImpl3D(
		JoltBodyImpl3D* p_body_a,
		JoltBodyImpl3D* p_body_b,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	PhysicsServer3D::JointType get_type() const override {
		return PhysicsServer3D::JOINT_TYPE_CONE_TWIST;
	}

	double get_param(PhysicsServer3D::ConeTwistJointParam p_param) const;

	void set_param(
		PhysicsServer3D::ConeTwistJointParam p_param,
		double p_value,
		bool p_lock = true
	);

	void rebuild(bool p_lock = true) override;

private:
	static JPH::Constraint* build_swing_twist(
		JPH::Body* p_jolt_body_a,
		JPH::Body* p_jolt_body_b,
		const Transform3D& p_shifted_ref_a,
		const Transform3D& p_shifted_ref_b,
		float p_swing_limit,
		float p_twist_limit
	);

	void limits_changed(bool p_lock = true);

	double swing_span = 0.0;

	double twist_span = 0.0;
};
