#include "jolt_cone_twist_joint_3d.hpp"

#include "jolt_body_3d.hpp"
#include "jolt_body_access_3d.hpp"

namespace {

constexpr double GDJOLT_CONE_TWIST_JOINT_DEFAULT_BIAS = 0.3;
constexpr double GDJOLT_CONE_TWIST_JOINT_DEFAULT_SOFTNESS = 0.8;
constexpr double GDJOLT_CONE_TWIST_JOINT_DEFAULT_RELAXATION = 1.0;

} // namespace

JoltConeTwistJoint3D::JoltConeTwistJoint3D(
	JoltSpace3D* p_space,
	const JoltBody3D& p_body_a,
	const JoltBody3D& p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJoint3D(p_space) {
	const JPH::BodyID body_ids[] = {p_body_a.get_jolt_id(), p_body_b.get_jolt_id()};
	const JoltMultiBodyAccessWrite3D body_access(*space, body_ids, count_of(body_ids), p_lock);

	JPH::Body* jolt_body_a = body_access.get_body(0);
	ERR_FAIL_NULL(jolt_body_a);

	JPH::Body* jolt_body_b = body_access.get_body(1);
	ERR_FAIL_NULL(jolt_body_b);

	const JPH::Shape& jolt_shape_a = *jolt_body_a->GetShape();
	const JPH::Shape& jolt_shape_b = *jolt_body_b->GetShape();

	JPH::SwingTwistConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPosition1 = to_jolt(p_local_ref_a.origin) - jolt_shape_a.GetCenterOfMass();
	constraint_settings.mTwistAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPosition2 = to_jolt(p_local_ref_b.origin) - jolt_shape_b.GetCenterOfMass();
	constraint_settings.mTwistAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltConeTwistJoint3D::JoltConeTwistJoint3D(
	JoltSpace3D* p_space,
	const JoltBody3D& p_body_a,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJoint3D(p_space) {
	const JoltBodyAccessWrite3D body_access(*space, p_body_a.get_jolt_id(), p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::Body& jolt_body_a = body_access.get_body();
	const JPH::Shape& jolt_shape_a = *jolt_body_a.GetShape();

	JPH::SwingTwistConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPosition1 = to_jolt(p_local_ref_a.origin) - jolt_shape_a.GetCenterOfMass();
	constraint_settings.mTwistAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPosition2 = to_jolt(p_local_ref_b.origin);
	constraint_settings.mTwistAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));

	jolt_ref = constraint_settings.Create(jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

double JoltConeTwistJoint3D::get_param(PhysicsServer3D::ConeTwistJointParam p_param) {
	auto* jolt_constraint = static_cast<JPH::SwingTwistConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL_D(jolt_constraint);

	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN: {
			return swing_span;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN: {
			return twist_span;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS: {
			return GDJOLT_CONE_TWIST_JOINT_DEFAULT_BIAS;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS: {
			return GDJOLT_CONE_TWIST_JOINT_DEFAULT_SOFTNESS;
		}
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION: {
			return GDJOLT_CONE_TWIST_JOINT_DEFAULT_RELAXATION;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled cone twist joint parameter: '{}'", p_param));
		}
	}
}

void JoltConeTwistJoint3D::set_param(PhysicsServer3D::ConeTwistJointParam p_param, double p_value) {
	auto* jolt_constraint = static_cast<JPH::SwingTwistConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN: {
			swing_span = p_value;
			spans_changed();
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN: {
			twist_span = p_value;
			spans_changed();
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, GDJOLT_CONE_TWIST_JOINT_DEFAULT_BIAS)) {
				WARN_PRINT(
					"Cone twist joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, GDJOLT_CONE_TWIST_JOINT_DEFAULT_SOFTNESS)) {
				WARN_PRINT(
					"Cone twist joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, GDJOLT_CONE_TWIST_JOINT_DEFAULT_RELAXATION)) {
				WARN_PRINT(
					"Cone twist joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled cone twist joint parameter: '{}'", p_param));
		} break;
	}
}

void JoltConeTwistJoint3D::spans_changed() {
	auto* jolt_constraint = static_cast<JPH::SwingTwistConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	constexpr double basically_zero = -CMP_EPSILON;
	constexpr double basically_pi = Math_PI + CMP_EPSILON;

	const double half_twist_span = twist_span / 2.0;

	if (half_twist_span >= basically_zero && half_twist_span <= basically_pi) {
		const float half_twist_span_clamped = clamp((float)half_twist_span, 0.0f, JPH::JPH_PI);

		jolt_constraint->SetTwistMinAngle(-half_twist_span_clamped);
		jolt_constraint->SetTwistMaxAngle(half_twist_span_clamped);
	} else {
		jolt_constraint->SetTwistMinAngle(-JPH::JPH_PI);
		jolt_constraint->SetTwistMaxAngle(JPH::JPH_PI);
	}

	const double half_swing_span = swing_span / 2.0;

	if (half_swing_span >= basically_zero && half_swing_span <= basically_pi) {
		const float half_swing_span_clamped = clamp((float)half_swing_span, 0.0f, JPH::JPH_PI);

		jolt_constraint->SetNormalHalfConeAngle(half_swing_span_clamped);
		jolt_constraint->SetPlaneHalfConeAngle(half_swing_span_clamped);
	} else {
		jolt_constraint->SetNormalHalfConeAngle(JPH::JPH_PI);
		jolt_constraint->SetPlaneHalfConeAngle(JPH::JPH_PI);

		// HACK(mihe): As far as I can tell this emulates the behavior of Godot Physics, where the
		// twist axis becomes unbounded if the swing span is a nonsensical value
		jolt_constraint->SetTwistMinAngle(-JPH::JPH_PI);
		jolt_constraint->SetTwistMaxAngle(JPH::JPH_PI);
	}
}
