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
	: JoltJointImpl3D(p_body_a, p_body_b, p_local_ref_a, p_local_ref_b, p_lock) {
	rebuild(p_lock);
}

JoltConeTwistJointImpl3D::JoltConeTwistJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	[[maybe_unused]] const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_local_ref_b) {
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
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN: {
			twist_span = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(vformat(
					"Cone twist joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Cone twist joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored. ",
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_RELAXATION)) {
				WARN_PRINT(vformat(
					"Cone twist joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
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

	const JoltWritableBodies3D bodies = space->write_bodies(body_ids, body_count, p_lock);

	JPH::SwingTwistConstraintSettings constraint_settings;

	if (twist_span >= 0 && twist_span <= Math_PI) {
		constraint_settings.mTwistMinAngle = (float)-twist_span;
		constraint_settings.mTwistMaxAngle = (float)twist_span;
	} else {
		constraint_settings.mTwistMinAngle = -JPH::JPH_PI;
		constraint_settings.mTwistMaxAngle = JPH::JPH_PI;
	}

	if (swing_span >= 0 && swing_span <= Math_PI) {
		constraint_settings.mNormalHalfConeAngle = (float)swing_span;
		constraint_settings.mPlaneHalfConeAngle = (float)swing_span;
	} else {
		constraint_settings.mNormalHalfConeAngle = JPH::JPH_PI;
		constraint_settings.mPlaneHalfConeAngle = JPH::JPH_PI;

		// NOTE(mihe): As far as I can tell this emulates the behavior of Godot Physics, where the
		// twist axis becomes unbounded if the swing span is a nonsensical value.
		constraint_settings.mTwistMinAngle = -JPH::JPH_PI;
		constraint_settings.mTwistMaxAngle = JPH::JPH_PI;
	}

	constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
	constraint_settings.mPosition1 = to_jolt(world_ref.origin);
	constraint_settings.mTwistAxis1 = to_jolt(-world_ref.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis1 = to_jolt(-world_ref.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPosition2 = to_jolt(world_ref.origin);
	constraint_settings.mTwistAxis2 = to_jolt(-world_ref.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis2 = to_jolt(-world_ref.basis.get_column(Vector3::AXIS_Z));

	if (body_b != nullptr) {
		const JoltWritableBody3D jolt_body_a = bodies[0];
		ERR_FAIL_COND(jolt_body_a.is_invalid());

		const JoltWritableBody3D jolt_body_b = bodies[1];
		ERR_FAIL_COND(jolt_body_b.is_invalid());

		jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);
	} else {
		const JoltWritableBody3D jolt_body_a = bodies[0];
		ERR_FAIL_COND(jolt_body_a.is_invalid());

		jolt_ref = constraint_settings.Create(*jolt_body_a, JPH::Body::sFixedToWorld);
	}

	space->add_joint(this);
}
