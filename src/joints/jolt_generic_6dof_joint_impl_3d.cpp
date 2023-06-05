#include "jolt_generic_6dof_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_LINEAR_LIMIT_SOFTNESS = 0.7;
constexpr double DEFAULT_LINEAR_RESTITUTION = 0.5;
constexpr double DEFAULT_LINEAR_DAMPING = 1.0;

constexpr double DEFAULT_LINEAR_SPRING_STIFFNESS = 0.01;
constexpr double DEFAULT_LINEAR_SPRING_DAMPING = 0.01;
constexpr double DEFAULT_LINEAR_SPRING_EQUILIBRIUM_POINT = 0.0;

constexpr double DEFAULT_ANGULAR_LIMIT_SOFTNESS = 0.5;
constexpr double DEFAULT_ANGULAR_DAMPING = 1.0;
constexpr double DEFAULT_ANGULAR_RESTITUTION = 0.0;
constexpr double DEFAULT_ANGULAR_FORCE_LIMIT = 0.0;
constexpr double DEFAULT_ANGULAR_ERP = 0.5;

constexpr double DEFAULT_ANGULAR_SPRING_STIFFNESS = 0.0;
constexpr double DEFAULT_ANGULAR_SPRING_DAMPING = 0.0;
constexpr double DEFAULT_ANGULAR_SPRING_EQUILIBRIUM_POINT = 0.0;

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
			return motor_velocity[axis_lin];
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT: {
			return motor_limit[axis_lin];
		}
		case 7: /* G6DOF_JOINT_LINEAR_SPRING_STIFFNESS */ {
			return DEFAULT_LINEAR_SPRING_STIFFNESS;
		}
		case 8: /* G6DOF_JOINT_LINEAR_SPRING_DAMPING */ {
			return DEFAULT_LINEAR_SPRING_DAMPING;
		}
		case 9: /* G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT */ {
			return DEFAULT_LINEAR_SPRING_EQUILIBRIUM_POINT;
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
			return motor_velocity[axis_ang];
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT: {
			return motor_limit[axis_ang];
		}
		case 19: /* G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS */ {
			return DEFAULT_ANGULAR_SPRING_STIFFNESS;
		}
		case 20: /* G6DOF_JOINT_ANGULAR_SPRING_DAMPING */ {
			return DEFAULT_ANGULAR_SPRING_DAMPING;
		}
		case 21: /* G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT */ {
			return DEFAULT_ANGULAR_SPRING_EQUILIBRIUM_POINT;
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
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT: {
			limit_upper[axis_lin] = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint linear limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint linear restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_DAMPING)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint linear damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY: {
			motor_velocity[axis_lin] = p_value;

			if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
				constraint->SetTargetVelocityCS(
					{(float)motor_velocity[AXIS_LINEAR_X],
					 (float)motor_velocity[AXIS_LINEAR_Y],
					 (float)motor_velocity[AXIS_LINEAR_Z]}
				);
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT: {
			motor_limit[axis_lin] = p_value;

			if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
				constraint->GetMotorSettings((JoltAxis)axis_lin).SetForceLimit((float)p_value);
			}
		} break;
		case 7: /* G6DOF_JOINT_LINEAR_SPRING_STIFFNESS */ {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_SPRING_STIFFNESS)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case 8: /* G6DOF_JOINT_LINEAR_SPRING_DAMPING */ {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_SPRING_DAMPING)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case 9: /* G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT */ {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_SPRING_EQUILIBRIUM_POINT)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT: {
			limit_lower[axis_ang] = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT: {
			limit_upper[axis_ang] = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint angular limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_DAMPING)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint angular damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint angular restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_FORCE_LIMIT)) {
				WARN_PRINT(vformat(
					"Generic 6DOF angular force limit is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ERP)) {
				WARN_PRINT(vformat(
					"Generic 6DOF angular ERP is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY: {
			motor_velocity[axis_ang] = p_value;

			if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
				// HACK(mihe): We're forced to flip the direction of these to match Godot Physics.
				// This does mean that the velocity direction is inconsistent with the velocity
				// direction for things like hinge joints.
				constraint->SetTargetAngularVelocityCS(
					{-(float)motor_velocity[AXIS_ANGULAR_X],
					 -(float)motor_velocity[AXIS_ANGULAR_Y],
					 -(float)motor_velocity[AXIS_ANGULAR_Z]}
				);
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT: {
			motor_limit[axis_ang] = p_value;

			if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
				constraint->GetMotorSettings((JoltAxis)axis_ang).SetTorqueLimit((float)p_value);
			}
		} break;
		case 19: /* G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS */ {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_SPRING_STIFFNESS)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case 20: /* G6DOF_JOINT_ANGULAR_SPRING_DAMPING */ {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_SPRING_DAMPING)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case 21: /* G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT */ {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_SPRING_EQUILIBRIUM_POINT)) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
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
			return false;
		}
		case 3: /* G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING */ {
			return false;
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
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT: {
			use_limits[axis_ang] = p_enabled;
			rebuild(p_lock);
		} break;
		case 2: /* G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING */ {
			if (p_enabled) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case 3: /* G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING */ {
			if (p_enabled) {
				WARN_PRINT(vformat(
					"Generic 6DOF joint springs are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR: {
			motor_enabled[axis_ang] = p_enabled;

			if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
				constraint->SetMotorState(
					(JoltAxis)axis_ang,
					p_enabled ? JPH::EMotorState::Velocity : JPH::EMotorState::Off
				);
			}
		} break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR: {
			motor_enabled[axis_lin] = p_enabled;

			if (auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr())) {
				constraint->SetMotorState(
					(JoltAxis)axis_lin,
					p_enabled ? JPH::EMotorState::Velocity : JPH::EMotorState::Off
				);
			}
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
			const double middle = Math::lerp(lower, upper, 0.5);

			ref_shift[axis] = (float)middle;

			if (Math::is_equal_approx(lower, upper)) {
				constraint_settings.MakeFixedAxis((JoltAxis)axis);
			} else {
				const auto extent = (float)Math::abs(middle - lower);
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

	for (int32_t axis = 0; axis < AXIS_COUNT; ++axis) {
		if (axis >= AXIS_LINEAR_X && axis <= AXIS_LINEAR_Z) {
			constraint_settings.mMotorSettings[axis].SetForceLimit((float)motor_limit[axis]);
		} else {
			constraint_settings.mMotorSettings[axis].SetTorqueLimit((float)motor_limit[axis]);
		}
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

	auto* constraint = static_cast<JPH::SixDOFConstraint*>(jolt_ref.GetPtr());

	constraint->SetTargetVelocityCS(
		{(float)motor_velocity[AXIS_LINEAR_X],
		 (float)motor_velocity[AXIS_LINEAR_Y],
		 (float)motor_velocity[AXIS_LINEAR_Z]}
	);

	// HACK(mihe): We're forced to flip the direction of these to match Godot Physics. This does
	// mean that the velocity direction is inconsistent with the velocity direction for things like
	// hinge joints.
	constraint->SetTargetAngularVelocityCS(
		{-(float)motor_velocity[AXIS_ANGULAR_X],
		 -(float)motor_velocity[AXIS_ANGULAR_Y],
		 -(float)motor_velocity[AXIS_ANGULAR_Z]}
	);

	for (int32_t axis = 0; axis < AXIS_COUNT; ++axis) {
		if (motor_enabled[axis]) {
			constraint->SetMotorState((JoltAxis)axis, JPH::EMotorState::Velocity);
		}
	}
}
