#include "jolt_cone_twist_joint_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_BIAS = 0.3;
constexpr double DEFAULT_SOFTNESS = 0.8;
constexpr double DEFAULT_RELAXATION = 1.0;

} // namespace

JoltConeTwistJoint3D::JoltConeTwistJoint3D(
	JoltSpace3D* p_space,
	JoltBody3D* p_body_a,
	JoltBody3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJoint3D(p_space, p_body_a, p_body_b) {
	const JPH::BodyID body_ids[] = {body_a->get_jolt_id(), body_b->get_jolt_id()};
	const JoltWritableBodies3D bodies = space->write_bodies(body_ids, count_of(body_ids), p_lock);

	const JoltWritableBody3D jolt_body_a = bodies[0];
	ERR_FAIL_COND(jolt_body_a.is_invalid());

	const JoltWritableBody3D jolt_body_b = bodies[1];
	ERR_FAIL_COND(jolt_body_b.is_invalid());

	const JoltCollisionObject3D& object_a = *jolt_body_a.as_object();
	const JoltCollisionObject3D& object_b = *jolt_body_b.as_object();

	const JPH::Vec3 point_scaled_a = to_jolt(p_local_ref_a.origin * object_a.get_scale());
	const JPH::Vec3 point_scaled_b = to_jolt(p_local_ref_b.origin * object_b.get_scale());

	const JPH::Vec3 com_scaled_a = jolt_body_a->GetShape()->GetCenterOfMass();
	const JPH::Vec3 com_scaled_b = jolt_body_b->GetShape()->GetCenterOfMass();

	JPH::SwingTwistConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPosition1 = point_scaled_a - com_scaled_a;
	constraint_settings.mTwistAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPosition2 = point_scaled_b - com_scaled_b;
	constraint_settings.mTwistAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltConeTwistJoint3D::JoltConeTwistJoint3D(
	JoltSpace3D* p_space,
	JoltBody3D* p_body_a,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJoint3D(p_space, p_body_a) {
	const JoltWritableBody3D jolt_body_a = space->write_body(*body_a, p_lock);
	ERR_FAIL_COND(jolt_body_a.is_invalid());

	const JoltCollisionObject3D& object_a = *jolt_body_a.as_object();

	const JPH::Vec3 point_scaled_a = to_jolt(p_local_ref_a.origin * object_a.get_scale());
	const JPH::Vec3 point_scaled_b = to_jolt(p_local_ref_b.origin);

	const JPH::Vec3 com_scaled_a = jolt_body_a->GetShape()->GetCenterOfMass();

	JPH::SwingTwistConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPosition1 = point_scaled_a - com_scaled_a;
	constraint_settings.mTwistAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPosition2 = point_scaled_b;
	constraint_settings.mTwistAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPlaneAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));

	jolt_ref = constraint_settings.Create(*jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

double JoltConeTwistJoint3D::get_param(PhysicsServer3D::ConeTwistJointParam p_param) const {
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

void JoltConeTwistJoint3D::set_param(PhysicsServer3D::ConeTwistJointParam p_param, double p_value) {
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
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(
					"Cone twist joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_SOFTNESS)) {
				WARN_PRINT(
					"Cone twist joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_RELAXATION)) {
				WARN_PRINT(
					"Cone twist joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled cone twist joint parameter: '%d'", p_param));
		} break;
	}
}

void JoltConeTwistJoint3D::spans_changed() {
	auto* jolt_constraint = static_cast<JPH::SwingTwistConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	constexpr double basically_zero = -CMP_EPSILON;
	constexpr double basically_pi = Math_PI + CMP_EPSILON;

	if (twist_span >= basically_zero && twist_span <= basically_pi) {
		const float twist_span_clamped = clamp((float)twist_span, 0.0f, JPH::JPH_PI);

		jolt_constraint->SetTwistMinAngle(-twist_span_clamped);
		jolt_constraint->SetTwistMaxAngle(twist_span_clamped);
	} else {
		jolt_constraint->SetTwistMinAngle(-JPH::JPH_PI);
		jolt_constraint->SetTwistMaxAngle(JPH::JPH_PI);
	}

	if (swing_span >= basically_zero && swing_span <= basically_pi) {
		const float swing_span_clamped = clamp((float)swing_span, 0.0f, JPH::JPH_PI);

		jolt_constraint->SetNormalHalfConeAngle(swing_span_clamped);
		jolt_constraint->SetPlaneHalfConeAngle(swing_span_clamped);
	} else {
		jolt_constraint->SetNormalHalfConeAngle(JPH::JPH_PI);
		jolt_constraint->SetPlaneHalfConeAngle(JPH::JPH_PI);

		// HACK(mihe): As far as I can tell this emulates the behavior of Godot Physics, where the
		// twist axis becomes unbounded if the swing span is a nonsensical value.
		jolt_constraint->SetTwistMinAngle(-JPH::JPH_PI);
		jolt_constraint->SetTwistMaxAngle(JPH::JPH_PI);
	}
}
