#include "jolt_hinge_joint_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_BIAS = 0.3;
constexpr double DEFAULT_LIMIT_BIAS = 0.3;
constexpr double DEFAULT_SOFTNESS = 0.9;
constexpr double DEFAULT_RELAXATION = 1.0;

} // namespace

JoltHingeJointImpl3D::JoltHingeJointImpl3D(
	JoltSpace3D* p_space,
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_space, p_body_a, p_body_b) {
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

	JPH::HingeConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = point_scaled_a - com_scaled_a;
	constraint_settings.mHingeAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis1 = to_jolt(p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPoint2 = point_scaled_b - com_scaled_b;
	constraint_settings.mHingeAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis2 = to_jolt(p_local_ref_b.basis.get_column(Vector3::AXIS_X));

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltHingeJointImpl3D::JoltHingeJointImpl3D(
	JoltSpace3D* p_space,
	JoltBodyImpl3D* p_body_a,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_space, p_body_a) {
	const JoltWritableBody3D jolt_body_a = space->write_body(*body_a, p_lock);
	ERR_FAIL_COND(jolt_body_a.is_invalid());

	const JoltCollisionObject3D& object_a = *jolt_body_a.as_object();

	const JPH::Vec3 point_scaled_a = to_jolt(p_local_ref_a.origin * object_a.get_scale());
	const JPH::Vec3 point_scaled_b = to_jolt(p_local_ref_b.origin);

	const JPH::Vec3 com_scaled_a = jolt_body_a->GetShape()->GetCenterOfMass();

	JPH::HingeConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = point_scaled_a - com_scaled_a;
	constraint_settings.mHingeAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis1 = to_jolt(p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPoint2 = point_scaled_b;
	constraint_settings.mHingeAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis2 = to_jolt(p_local_ref_b.basis.get_column(Vector3::AXIS_X));

	jolt_ref = constraint_settings.Create(*jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

double JoltHingeJointImpl3D::get_param(PhysicsServer3D::HingeJointParam p_param) const {
	const auto* jolt_constraint = static_cast<const JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL_D(jolt_constraint);

	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			return DEFAULT_BIAS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			return limit_upper;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			return limit_lower;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			return DEFAULT_LIMIT_BIAS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			return DEFAULT_SOFTNESS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			return DEFAULT_RELAXATION;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			return jolt_constraint->GetTargetAngularVelocity();
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			return motor_max_impulse;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint parameter: '%d'", p_param));
		}
	}
}

void JoltHingeJointImpl3D::set_param(PhysicsServer3D::HingeJointParam p_param, double p_value) {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(
					"Hinge joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			limit_upper = p_value;
			limits_changed();
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			limit_lower = p_value;
			limits_changed();
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LIMIT_BIAS)) {
				WARN_PRINT(
					"Hinge joint bias limit is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_SOFTNESS)) {
				WARN_PRINT(
					"Hinge joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_RELAXATION)) {
				WARN_PRINT(
					"Hinge joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			jolt_constraint->SetTargetAngularVelocity((float)p_value);
		} break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			motor_max_impulse = p_value;

			// HACK(mihe): This will break if the physics time step changes in any way during the
			// lifetime of this joint, but it can't really be fixed since Godot only provides a max
			// impulse and not a max force. As far as I can tell this is similarly broken in Godot
			// Physics as well, so at least we're being consistent.
			const double max_torque = motor_max_impulse / space->get_last_step();

			JPH::MotorSettings& motor_settings = jolt_constraint->GetMotorSettings();
			motor_settings.mMinTorqueLimit = (float)-max_torque;
			motor_settings.mMaxTorqueLimit = (float)+max_torque;
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint parameter: '%d'", p_param));
		} break;
	}
}

bool JoltHingeJointImpl3D::get_flag(PhysicsServer3D::HingeJointFlag p_flag) const {
	const auto* jolt_constraint = static_cast<const JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL_D(jolt_constraint);

	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			return use_limits;
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			return jolt_constraint->GetMotorState() != JPH::EMotorState::Off;
		} break;
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltHingeJointImpl3D::set_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled) {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			use_limits = p_enabled;
			limits_changed();
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			jolt_constraint->SetMotorState(
				p_enabled ? JPH::EMotorState::Velocity : JPH::EMotorState::Off
			);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltHingeJointImpl3D::limits_changed() {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	if (use_limits) {
		constexpr double basically_pos_zero = CMP_EPSILON;
		constexpr double basically_neg_zero = -CMP_EPSILON;
		constexpr double basically_pos_pi = Math_PI + CMP_EPSILON;
		constexpr double basically_neg_pi = -Math_PI - CMP_EPSILON;

		if (limit_lower < basically_neg_pi || limit_lower > basically_pos_zero) {
			WARN_PRINT(
				"Hinge joint lower limits less than -180º or greater than 0º are not supported by "
				"Godot Jolt. Values outside this range will be clamped."
			);
		}

		if (limit_upper < basically_neg_zero || limit_upper > basically_pos_pi) {
			WARN_PRINT(
				"Hinge joint upper limits less than 0º or greater than 180º are not supported by "
				"Godot Jolt. Values outside this range will be clamped."
			);
		}

		const float limit_lower_clamped = clamp((float)limit_lower, -JPH::JPH_PI, 0.0f);
		const float limit_upper_clamped = clamp((float)limit_upper, 0.0f, JPH::JPH_PI);

		jolt_constraint->SetLimits(limit_lower_clamped, limit_upper_clamped);
	} else {
		jolt_constraint->SetLimits(-JPH::JPH_PI, JPH::JPH_PI);
	}
}
