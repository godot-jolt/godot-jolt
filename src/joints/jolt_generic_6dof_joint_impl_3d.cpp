#include "jolt_generic_6dof_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_LINEAR_LIMIT_SOFTNESS = 0.7;
constexpr double DEFAULT_LINEAR_RESTITUTION = 0.5;
constexpr double DEFAULT_LINEAR_DAMPING = 1.0;

constexpr double DEFAULT_ANGULAR_LIMIT_SOFTNESS = 0.5;
constexpr double DEFAULT_ANGULAR_DAMPING = 1.0;
constexpr double DEFAULT_ANGULAR_RESTITUTION = 0.0;
constexpr double DEFAULT_ANGULAR_FORCE_LIMIT = 0.0;
constexpr double DEFAULT_ANGULAR_ERP = 0.5;

} // namespace

JoltGeneric6DOFJointImpl3D::JoltGeneric6DOFJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_body_b, p_local_ref_a, p_local_ref_b) {
	rebuild(p_lock);
}

double JoltGeneric6DOFJointImpl3D::get_param(
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisParam p_param
) const {
	const int32_t axis_lin = AXES_LINEAR + (int32_t)p_axis;
	const int32_t axis_ang = AXES_ANGULAR + (int32_t)p_axis;

	switch ((int32_t)p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT: {
			return limit_lower[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT: {
			return limit_upper[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS: {
			return DEFAULT_LINEAR_LIMIT_SOFTNESS;
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION: {
			return DEFAULT_LINEAR_RESTITUTION;
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING: {
			return DEFAULT_LINEAR_DAMPING;
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY: {
			return motor_speed[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT: {
			return motor_limit[axis_lin];
		}
		case 7: /* G6DOF_JOINT_LINEAR_SPRING_STIFFNESS */ {
			return spring_stiffness[axis_lin];
		}
		case 8: /* G6DOF_JOINT_LINEAR_SPRING_DAMPING */ {
			return spring_damping[axis_lin];
		}
		case 9: /* G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT */ {
			return spring_equilibrium[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT: {
			return limit_lower[axis_ang];
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT: {
			return limit_upper[axis_ang];
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS: {
			return DEFAULT_ANGULAR_LIMIT_SOFTNESS;
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING: {
			return DEFAULT_ANGULAR_DAMPING;
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION: {
			return DEFAULT_ANGULAR_RESTITUTION;
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT: {
			return DEFAULT_ANGULAR_FORCE_LIMIT;
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP: {
			return DEFAULT_ANGULAR_ERP;
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY: {
			return motor_speed[axis_ang];
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT: {
			return motor_limit[axis_ang];
		}
		case 19: /* G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS */ {
			return spring_stiffness[axis_ang];
		}
		case 20: /* G6DOF_JOINT_ANGULAR_SPRING_DAMPING */ {
			return spring_damping[axis_ang];
		}
		case 21: /* G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT */ {
			return spring_equilibrium[axis_ang];
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled 6DOF joint parameter: '%d'", p_param));
		}
	}
}

void JoltGeneric6DOFJointImpl3D::set_param(
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisParam p_param,
	double p_value,
	bool p_lock
) {
	const int32_t axis_lin = AXES_LINEAR + (int32_t)p_axis;
	const int32_t axis_ang = AXES_ANGULAR + (int32_t)p_axis;

	switch ((int32_t)p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT: {
			limit_lower[axis_lin] = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT: {
			limit_upper[axis_lin] = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint linear limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint linear restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_DAMPING)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint linear damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY: {
			motor_speed[axis_lin] = p_value;
			motor_speed_changed(axis_lin);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT: {
			motor_limit[axis_lin] = p_value;
			motor_limit_changed(axis_lin);
		} break;
		case 7: /* G6DOF_JOINT_LINEAR_SPRING_STIFFNESS */ {
			spring_stiffness[axis_lin] = p_value;
			spring_stiffness_changed(axis_lin);
		} break;
		case 8: /* G6DOF_JOINT_LINEAR_SPRING_DAMPING */ {
			spring_damping[axis_lin] = p_value;
			spring_damping_changed(axis_lin);
		} break;
		case 9: /* G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT */ {
			spring_equilibrium[axis_lin] = p_value;
			spring_equilibrium_changed(axis_lin);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT: {
			limit_lower[axis_ang] = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT: {
			limit_upper[axis_ang] = p_value;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint angular limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_DAMPING)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint angular damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint angular restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_FORCE_LIMIT)) {
				WARN_PRINT(vformat(
					"Generic 6DOF angular force limit is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ERP)) {
				WARN_PRINT(vformat(
					"Generic 6DOF angular ERP is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY: {
			motor_speed[axis_ang] = p_value;
			motor_speed_changed(axis_ang);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT: {
			motor_limit[axis_ang] = p_value;
			motor_limit_changed(axis_ang);
		} break;
		case 19: /* G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS */ {
			spring_stiffness[axis_ang] = p_value;
			spring_stiffness_changed(axis_ang);
		} break;
		case 20: /* G6DOF_JOINT_ANGULAR_SPRING_DAMPING */ {
			spring_damping[axis_ang] = p_value;
			spring_damping_changed(axis_ang);
		} break;
		case 21: /* G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT */ {
			spring_equilibrium[axis_ang] = p_value;
			spring_equilibrium_changed(axis_ang);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled 6DOF joint parameter: '%d'", p_param));
		} break;
	}
}

bool JoltGeneric6DOFJointImpl3D::get_flag(
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisFlag p_flag
) const {
	const int32_t axis_lin = AXES_LINEAR + (int32_t)p_axis;
	const int32_t axis_ang = AXES_ANGULAR + (int32_t)p_axis;

	switch ((int32_t)p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT: {
			return use_limits[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT: {
			return use_limits[axis_ang];
		}
		case 2: /* G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING */ {
			return spring_enabled[axis_ang];
		}
		case 3: /* G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING */ {
			return spring_enabled[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR: {
			return motor_enabled[axis_ang];
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR: {
			return motor_enabled[axis_lin];
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled 6DOF joint flag: '%d'", p_flag));
		}
	}
}

void JoltGeneric6DOFJointImpl3D::set_flag(
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisFlag p_flag,
	bool p_enabled,
	bool p_lock
) {
	const int32_t axis_lin = AXES_LINEAR + (int32_t)p_axis;
	const int32_t axis_ang = AXES_ANGULAR + (int32_t)p_axis;

	switch ((int32_t)p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT: {
			use_limits[axis_lin] = p_enabled;
			limits_changed(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT: {
			use_limits[axis_ang] = p_enabled;
			limits_changed(p_lock);
		} break;
		case 2: /* G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING */ {
			spring_enabled[axis_ang] = p_enabled;
			spring_state_changed(axis_ang);
		} break;
		case 3: /* G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING */ {
			spring_enabled[axis_lin] = p_enabled;
			spring_state_changed(axis_lin);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR: {
			motor_enabled[axis_ang] = p_enabled;
			motor_state_changed(axis_ang);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR: {
			motor_enabled[axis_lin] = p_enabled;
			motor_state_changed(axis_lin);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled 6DOF joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltGeneric6DOFJointImpl3D::rebuild(bool p_lock) {
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

	JPH::SixDOFConstraintSettings constraint_settings;

	float ref_shift[AXIS_COUNT] = {};

	for (int32_t axis = 0; axis < AXIS_COUNT; ++axis) {
		if (!use_limits[axis]) {
			constraint_settings.MakeFreeAxis((JoltAxis)axis);
			continue;
		}

		const double lower = limit_lower[axis];
		const double upper = limit_upper[axis];

		if (lower > upper) {
			// HACK(mihe): This seems to emulate the behavior of Godot Physics, where if the limits
			// result in a negative span then the axis becomes unbounded.
			constraint_settings.MakeFreeAxis((JoltAxis)axis);
		} else {
			const double midpoint = (lower + upper) / 2.0f;

			ref_shift[axis] = (float)-midpoint;

			if (Math::is_equal_approx(lower, upper)) {
				constraint_settings.MakeFixedAxis((JoltAxis)axis);
			} else {
				const auto extent = float(upper - midpoint);
				constraint_settings.SetLimitedAxis((JoltAxis)axis, -extent, extent);
			}
		}
	}

	const Vector3 linear_shift = {
		ref_shift[AXIS_LINEAR_X],
		ref_shift[AXIS_LINEAR_Y],
		ref_shift[AXIS_LINEAR_Z]};

	const Vector3 angular_shift = {
		ref_shift[AXIS_ANGULAR_X],
		ref_shift[AXIS_ANGULAR_Y],
		ref_shift[AXIS_ANGULAR_Z]};

	Transform3D shifted_ref_a;
	Transform3D shifted_ref_b;

	shift_reference_frames(linear_shift, angular_shift, shifted_ref_a, shifted_ref_b);

	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPosition1 = to_jolt(shifted_ref_a.origin);
	constraint_settings.mAxisX1 = to_jolt(shifted_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mAxisY1 = to_jolt(shifted_ref_a.basis.get_column(Vector3::AXIS_Y));
	constraint_settings.mPosition2 = to_jolt(shifted_ref_b.origin);
	constraint_settings.mAxisX2 = to_jolt(shifted_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mAxisY2 = to_jolt(shifted_ref_b.basis.get_column(Vector3::AXIS_Y));

	for (JPH::MotorSettings& motor_settings : constraint_settings.mMotorSettings) {
		motor_settings.mSpringSettings.mMode = JPH::ESpringMode::StiffnessAndDamping;
	}

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

	for (int32_t axis = 0; axis < AXIS_COUNT; ++axis) {
		update_motor_state(axis);
		update_motor_velocity(axis);
		update_motor_limit(axis);
		update_spring_stiffness(axis);
		update_spring_damping(axis);
		update_spring_equilibrium(axis);
	}
}

void JoltGeneric6DOFJointImpl3D::update_motor_state(int32_t p_axis) {
	if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
		if (motor_enabled[p_axis]) {
			constraint->SetMotorState((JoltAxis)p_axis, JPH::EMotorState::Velocity);
		} else if (spring_enabled[p_axis]) {
			constraint->SetMotorState((JoltAxis)p_axis, JPH::EMotorState::Position);
		} else {
			constraint->SetMotorState((JoltAxis)p_axis, JPH::EMotorState::Off);
		}
	}
}

void JoltGeneric6DOFJointImpl3D::update_motor_velocity(int32_t p_axis) {
	if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
		if (p_axis >= AXIS_LINEAR_X && p_axis <= AXIS_LINEAR_Z) {
			constraint->SetTargetVelocityCS(
				{(float)motor_speed[AXIS_LINEAR_X],
				 (float)motor_speed[AXIS_LINEAR_Y],
				 (float)motor_speed[AXIS_LINEAR_Z]}
			);
		} else {
			// HACK(mihe): We're forced to flip the direction of these to match Godot Physics. This
			// means that the velocity direction is inconsistent with the velocity direction for
			// things like hinge joints.
			constraint->SetTargetAngularVelocityCS(
				{(float)-motor_speed[AXIS_ANGULAR_X],
				 (float)-motor_speed[AXIS_ANGULAR_Y],
				 (float)-motor_speed[AXIS_ANGULAR_Z]}
			);
		}
	}
}

void JoltGeneric6DOFJointImpl3D::update_motor_limit(int32_t p_axis) {
	if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
		JPH::MotorSettings& motor_settings = constraint->GetMotorSettings((JoltAxis)p_axis);

		// HACK(mihe): We only apply the motor limit if we're actually using a velocity motor, since
		// it would otherwise affect a position motor as well, which doesn't seem to be how this is
		// applied in the Bullet implementation in Godot 3.
		const auto limit = motor_enabled[p_axis] ? (float)motor_limit[p_axis] : FLT_MAX;

		if (p_axis >= AXIS_LINEAR_X && p_axis <= AXIS_LINEAR_Z) {
			motor_settings.SetForceLimit(limit);
		} else {
			motor_settings.SetTorqueLimit(limit);
		}
	}
}

void JoltGeneric6DOFJointImpl3D::update_spring_stiffness(int32_t p_axis) {
	if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
		JPH::MotorSettings& motor_settings = constraint->GetMotorSettings((JoltAxis)p_axis);
		motor_settings.mSpringSettings.mStiffness = (float)spring_stiffness[p_axis];
	}
}

void JoltGeneric6DOFJointImpl3D::update_spring_damping(int32_t p_axis) {
	if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
		JPH::MotorSettings& motor_settings = constraint->GetMotorSettings((JoltAxis)p_axis);
		motor_settings.mSpringSettings.mDamping = (float)spring_damping[p_axis];
	}
}

void JoltGeneric6DOFJointImpl3D::update_spring_equilibrium(int32_t p_axis) {
	if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
		if (p_axis >= AXIS_LINEAR_X && p_axis <= AXIS_LINEAR_Z) {
			const Vector3 target_position = Vector3(
				(float)-spring_equilibrium[AXIS_LINEAR_X],
				(float)-spring_equilibrium[AXIS_LINEAR_Y],
				(float)-spring_equilibrium[AXIS_LINEAR_Z]
			);

			constraint->SetTargetPositionCS(to_jolt(target_position));
		} else {
			// HACK(mihe): These are flipped to match Bullet in Godot 3, presumably for the same
			// reason that the angular motor velocity needs to be flipped. Godot 4 does not
			// currently have springs implemented, so can't be used as a reference.
			const Basis target_orientation = Basis::from_euler(
				{(float)spring_equilibrium[AXIS_ANGULAR_X],
				 (float)spring_equilibrium[AXIS_ANGULAR_Y],
				 (float)spring_equilibrium[AXIS_ANGULAR_Z]}
			);

			constraint->SetTargetOrientationCS(to_jolt(target_orientation));
		}
	}
}

void JoltGeneric6DOFJointImpl3D::limits_changed(bool p_lock) {
	rebuild(p_lock);
}

void JoltGeneric6DOFJointImpl3D::motor_state_changed(int32_t p_axis) {
	update_motor_state(p_axis);
	update_motor_limit(p_axis);
}

void JoltGeneric6DOFJointImpl3D::motor_speed_changed(int32_t p_axis) {
	update_motor_velocity(p_axis);
}

void JoltGeneric6DOFJointImpl3D::motor_limit_changed(int32_t p_axis) {
	update_motor_limit(p_axis);
}

void JoltGeneric6DOFJointImpl3D::spring_state_changed(int32_t p_axis) {
	update_motor_state(p_axis);
}

void JoltGeneric6DOFJointImpl3D::spring_stiffness_changed(int32_t p_axis) {
	update_spring_stiffness(p_axis);
}

void JoltGeneric6DOFJointImpl3D::spring_damping_changed(int32_t p_axis) {
	update_spring_damping(p_axis);
}

void JoltGeneric6DOFJointImpl3D::spring_equilibrium_changed(int32_t p_axis) {
	update_spring_equilibrium(p_axis);
}
