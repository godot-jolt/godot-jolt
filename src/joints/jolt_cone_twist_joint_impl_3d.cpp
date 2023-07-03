#include "jolt_cone_twist_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_BIAS = 0.3;
constexpr double DEFAULT_SOFTNESS = 0.8;
constexpr double DEFAULT_RELAXATION = 1.0;

} // namespace

JoltConeTwistJointImpl3D::JoltConeTwistJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_body_b, p_local_ref_a, p_local_ref_b) {
	rebuild(p_lock);
}

double JoltConeTwistJointImpl3D::get_param(PhysicsServer3D::ConeTwistJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN: {
			return swing_span;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN: {
			return twist_span;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS: {
			return DEFAULT_BIAS;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS: {
			return DEFAULT_SOFTNESS;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION: {
			return DEFAULT_RELAXATION;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled cone twist joint parameter: '%d'", p_param));
		}
	}
}

void JoltConeTwistJointImpl3D::set_param(
	PhysicsServer3D::ConeTwistJointParam p_param,
	double p_value,
	bool p_lock
) {
	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN: {
			swing_span = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN: {
			twist_span = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(vformat(
					"Cone twist joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Cone twist joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored. ",
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_RELAXATION)) {
				WARN_PRINT(vformat(
					"Cone twist joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled cone twist joint parameter: '%d'", p_param));
		} break;
	}
}

void JoltConeTwistJointImpl3D::rebuild(bool p_lock) {
	destroy();

	JoltSpace3D* space = get_space();

	if (space == nullptr) {
		return;
	}

	JPH::BodyID body_ids[2] = {body_a->get_jolt_id()};
	int32_t body_count = 1;

	if (body_b != nullptr) {
		body_ids[1] = body_b->get_jolt_id();
		body_count = 2;
	}

	const JoltWritableBodies3D jolt_bodies = space->write_bodies(body_ids, body_count, p_lock);

	auto* jolt_body_a = static_cast<JPH::Body*>(jolt_bodies[0]);
	ERR_FAIL_COND(jolt_body_a == nullptr);

	auto* jolt_body_b = static_cast<JPH::Body*>(jolt_bodies[1]);
	ERR_FAIL_COND(jolt_body_b == nullptr && body_count == 2);

	Transform3D shifted_ref_a;
	Transform3D shifted_ref_b;

	shift_reference_frames(Vector3(), Vector3(), shifted_ref_a, shifted_ref_b);

	jolt_ref = build_swing_twist(
		jolt_body_a,
		jolt_body_b,
		shifted_ref_a,
		shifted_ref_b,
		(float)swing_span,
		(float)twist_span
	);

	space->add_joint(this);
}

JPH::Constraint* JoltConeTwistJointImpl3D::build_swing_twist(
	JPH::Body* p_jolt_body_a,
	JPH::Body* p_jolt_body_b,
	const Transform3D& p_shifted_ref_a,
	const Transform3D& p_shifted_ref_b,
	float p_swing_limit,
	float p_twist_limit
) {
	JPH::SwingTwistConstraintSettings constraint_settings;

	if (p_twist_limit >= 0 && p_twist_limit <= JPH::JPH_PI) {
		constraint_settings.mTwistMinAngle = -p_twist_limit;
		constraint_settings.mTwistMaxAngle = p_twist_limit;
	} else {
		constraint_settings.mTwistMinAngle = -JPH::JPH_PI;
		constraint_settings.mTwistMaxAngle = JPH::JPH_PI;
	}

	if (p_swing_limit >= 0 && p_swing_limit <= JPH::JPH_PI) {
		constraint_settings.mNormalHalfConeAngle = p_swing_limit;
		constraint_settings.mPlaneHalfConeAngle = p_swing_limit;
	} else {
		constraint_settings.mNormalHalfConeAngle = JPH::JPH_PI;
		constraint_settings.mPlaneHalfConeAngle = JPH::JPH_PI;

		// NOTE(mihe): As far as I can tell this emulates the behavior of Godot Physics, where the
		// twist axis becomes unbounded if the swing span is a nonsensical value.
		constraint_settings.mTwistMinAngle = -JPH::JPH_PI;
		constraint_settings.mTwistMaxAngle = JPH::JPH_PI;
	}

	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPosition1 = to_jolt(p_shifted_ref_a.origin);
	constraint_settings.mTwistAxis1 = to_jolt(-p_shifted_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis1 = to_jolt(-p_shifted_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPosition2 = to_jolt(p_shifted_ref_b.origin);
	constraint_settings.mTwistAxis2 = to_jolt(-p_shifted_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis2 = to_jolt(-p_shifted_ref_b.basis.get_column(Vector3::AXIS_Z));

	if (p_jolt_body_b != nullptr) {
		return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
	} else {
		return constraint_settings.Create(*p_jolt_body_a, JPH::Body::sFixedToWorld);
	}
}

void JoltConeTwistJointImpl3D::limits_changed(bool p_lock) {
	rebuild(p_lock);
}
