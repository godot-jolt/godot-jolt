#pragma once

#include "jolt_joint_3d.hpp"

class JoltConeTwistJoint3D final : public JoltJoint3D {
public:
	JoltConeTwistJoint3D(
		JoltSpace3D* p_space,
		JoltBody3D* p_body_a,
		JoltBody3D* p_body_b,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	JoltConeTwistJoint3D(
		JoltSpace3D* p_space,
		JoltBody3D* p_body_a,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	PhysicsServer3D::JointType get_type() const override {
		return PhysicsServer3D::JOINT_TYPE_CONE_TWIST;
	}

	double get_param(PhysicsServer3D::ConeTwistJointParam p_param);

	void set_param(PhysicsServer3D::ConeTwistJointParam p_param, double p_value);

private:
	void spans_changed();

	double swing_span = 0.0;

	double twist_span = 0.0;
};
