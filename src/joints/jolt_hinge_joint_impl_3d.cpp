#include "jolt_hinge_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_BIAS = 0.3;
constexpr double DEFAULT_LIMIT_BIAS = 0.3;
constexpr double DEFAULT_SOFTNESS = 0.9;
constexpr double DEFAULT_RELAXATION = 1.0;

} // namespace

JoltHingeJointImpl3D::JoltHingeJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_body_b, p_local_ref_a, p_local_ref_b) {
	rebuild(p_lock);
}

double JoltHingeJointImpl3D::get_param(PhysicsServer3D::HingeJointParam p_param) const {
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
			return motor_target_speed;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			return motor_max_impulse;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint parameter: '%d'", p_param));
		}
	}
}

void JoltHingeJointImpl3D::set_param(
	PhysicsServer3D::HingeJointParam p_param,
	double p_value,
	bool p_lock
) {
	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(vformat(
					"Hinge joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			limit_upper = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			limit_lower = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LIMIT_BIAS)) {
				WARN_PRINT(vformat(
					"Hinge joint bias limit is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Hinge joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_RELAXATION)) {
				WARN_PRINT(vformat(
					"Hinge joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			motor_target_speed = p_value;
			motor_speed_changed();
		} break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			motor_max_impulse = p_value;
			motor_limit_changed();
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint parameter: '%d'", p_param));
		} break;
	}
}

bool JoltHingeJointImpl3D::get_flag(PhysicsServer3D::HingeJointFlag p_flag) const {
	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			return use_limits;
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			return motor_enabled;
		} break;
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltHingeJointImpl3D::set_flag(
	PhysicsServer3D::HingeJointFlag p_flag,
	bool p_enabled,
	bool p_lock
) {
	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			use_limits = p_enabled;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			motor_enabled = p_enabled;
			motor_state_changed();
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltHingeJointImpl3D::rebuild(bool p_lock) {
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

	float ref_shift = 0.0f;
	float limit = JPH::JPH_PI;

	if (use_limits && limit_lower <= limit_upper) {
		const double limit_midpoint = (limit_lower + limit_upper) / 2.0f;

		ref_shift = float(-limit_midpoint);
		limit = float(limit_upper - limit_midpoint);
	}

	Transform3D shifted_ref_a;
	Transform3D shifted_ref_b;

	shift_reference_frames(Vector3(), Vector3(0.0f, 0.0f, ref_shift), shifted_ref_a, shifted_ref_b);

	if (is_fixed()) {
		jolt_ref = build_fixed(jolt_body_a, jolt_body_b, shifted_ref_a, shifted_ref_b);
	} else {
		jolt_ref = build_hinge(jolt_body_a, jolt_body_b, shifted_ref_a, shifted_ref_b, limit);
	}

	space->add_joint(this);

	update_motor_state();
	update_motor_velocity();
	update_motor_limit();
}

JPH::Constraint* JoltHingeJointImpl3D::build_hinge(
	JPH::Body* p_jolt_body_a,
	JPH::Body* p_jolt_body_b,
	const Transform3D& p_shifted_ref_a,
	const Transform3D& p_shifted_ref_b,
	float p_limit
) {
	JPH::HingeConstraintSettings constraint_settings;

	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = to_jolt(p_shifted_ref_a.origin);
	constraint_settings.mHingeAxis1 = to_jolt(-p_shifted_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis1 = to_jolt(p_shifted_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPoint2 = to_jolt(p_shifted_ref_b.origin);
	constraint_settings.mHingeAxis2 = to_jolt(-p_shifted_ref_b.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis2 = to_jolt(p_shifted_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mLimitsMin = -p_limit;
	constraint_settings.mLimitsMax = p_limit;

	if (p_jolt_body_b != nullptr) {
		return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
	} else {
		return constraint_settings.Create(*p_jolt_body_a, JPH::Body::sFixedToWorld);
	}
}

JPH::Constraint* JoltHingeJointImpl3D::build_fixed(
	JPH::Body* p_jolt_body_a,
	JPH::Body* p_jolt_body_b,
	const Transform3D& p_shifted_ref_a,
	const Transform3D& p_shifted_ref_b
) {
	JPH::FixedConstraintSettings constraint_settings;

	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mAutoDetectPoint = false;
	constraint_settings.mPoint1 = to_jolt(p_shifted_ref_a.origin);
	constraint_settings.mAxisX1 = to_jolt(p_shifted_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mAxisY1 = to_jolt(p_shifted_ref_a.basis.get_column(Vector3::AXIS_Y));
	constraint_settings.mPoint2 = to_jolt(p_shifted_ref_b.origin);
	constraint_settings.mAxisX2 = to_jolt(p_shifted_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mAxisY2 = to_jolt(p_shifted_ref_b.basis.get_column(Vector3::AXIS_Y));

	if (p_jolt_body_b != nullptr) {
		return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
	} else {
		return constraint_settings.Create(*p_jolt_body_a, JPH::Body::sFixedToWorld);
	}
}

void JoltHingeJointImpl3D::update_motor_state() {
	QUIET_FAIL_COND(is_fixed());

	if (auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr())) {
		constraint->SetMotorState(
			motor_enabled ? JPH::EMotorState::Velocity : JPH::EMotorState::Off
		);
	}
}

void JoltHingeJointImpl3D::update_motor_velocity() {
	QUIET_FAIL_COND(is_fixed());

	if (auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr())) {
		constraint->SetTargetAngularVelocity((float)motor_target_speed);
	}
}

void JoltHingeJointImpl3D::update_motor_limit() {
	QUIET_FAIL_COND(is_fixed());

	if (auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr())) {
		// HACK(mihe): This will break if the physics time step changes in any way during the
		// lifetime of this joint, but this can't really be fixed since Godot only provides a max
		// impulse and not a max force. As far as I can tell this is similarly broken in Godot
		// Physics as well, so at least we're being consistent.
		const double max_torque = motor_max_impulse / estimate_physics_step();

		JPH::MotorSettings& motor_settings = constraint->GetMotorSettings();
		motor_settings.mMinTorqueLimit = (float)-max_torque;
		motor_settings.mMaxTorqueLimit = (float)max_torque;
	}
}

void JoltHingeJointImpl3D::limits_changed(bool p_lock) {
	rebuild(p_lock);
}

void JoltHingeJointImpl3D::motor_state_changed() {
	update_motor_state();
}

void JoltHingeJointImpl3D::motor_speed_changed() {
	update_motor_velocity();
}

void JoltHingeJointImpl3D::motor_limit_changed() {
	update_motor_limit();
}
