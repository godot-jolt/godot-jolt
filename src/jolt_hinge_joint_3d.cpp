#include "jolt_hinge_joint_3d.hpp"

#include "jolt_body_3d.hpp"
#include "jolt_body_access_3d.hpp"

namespace {

constexpr double GDJOLT_HINGE_JOINT_DEFAULT_BIAS = 0.3;
constexpr double GDJOLT_HINGE_JOINT_DEFAULT_LIMIT_BIAS = 0.3;
constexpr double GDJOLT_HINGE_JOINT_DEFAULT_SOFTNESS = 0.9;
constexpr double GDJOLT_HINGE_JOINT_DEFAULT_RELAXATION = 1.0;

} // namespace

JoltHingeJoint3D::JoltHingeJoint3D(
	JoltSpace3D* p_space,
	const JoltBody3D& p_body_a,
	const JoltBody3D& p_body_b,
	const Transform3D& p_hinge_a,
	const Transform3D& p_hinge_b,
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

	JPH::HingeConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = to_jolt(p_hinge_a.origin) - jolt_shape_a.GetCenterOfMass();
	constraint_settings.mHingeAxis1 = to_jolt(-p_hinge_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis1 = to_jolt(p_hinge_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPoint2 = to_jolt(p_hinge_b.origin) - jolt_shape_b.GetCenterOfMass();
	constraint_settings.mHingeAxis2 = to_jolt(-p_hinge_b.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis2 = to_jolt(p_hinge_b.basis.get_column(Vector3::AXIS_X));

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltHingeJoint3D::JoltHingeJoint3D(
	JoltSpace3D* p_space,
	const JoltBody3D& p_body_a,
	const Transform3D& p_hinge_a,
	const Transform3D& p_hinge_b,
	bool p_lock
)
	: JoltJoint3D(p_space) {
	const JoltBodyAccessWrite3D body_access(*space, p_body_a.get_jolt_id(), p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::Body& jolt_body_a = body_access.get_body();
	const JPH::Shape& jolt_shape_a = *jolt_body_a.GetShape();

	JPH::HingeConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = to_jolt(p_hinge_a.origin) - jolt_shape_a.GetCenterOfMass();
	constraint_settings.mHingeAxis1 = to_jolt(-p_hinge_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis1 = to_jolt(p_hinge_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPoint2 = to_jolt(p_hinge_b.origin);
	constraint_settings.mHingeAxis2 = to_jolt(-p_hinge_b.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis2 = to_jolt(p_hinge_b.basis.get_column(Vector3::AXIS_X));

	jolt_ref = constraint_settings.Create(jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

double JoltHingeJoint3D::get_param(PhysicsServer3D::HingeJointParam p_param) {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL_D(jolt_constraint);

	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			return GDJOLT_HINGE_JOINT_DEFAULT_BIAS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			return limit_upper;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			return limit_lower;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			return GDJOLT_HINGE_JOINT_DEFAULT_LIMIT_BIAS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			return GDJOLT_HINGE_JOINT_DEFAULT_SOFTNESS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			return GDJOLT_HINGE_JOINT_DEFAULT_RELAXATION;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			return jolt_constraint->GetTargetAngularVelocity();
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			return motor_max_impulse;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint parameter: '{}'", p_param));
		}
	}
}

void JoltHingeJoint3D::set_param(PhysicsServer3D::HingeJointParam p_param, double p_value) {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, GDJOLT_HINGE_JOINT_DEFAULT_BIAS)) {
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
			if (!Math::is_equal_approx(p_value, GDJOLT_HINGE_JOINT_DEFAULT_LIMIT_BIAS)) {
				WARN_PRINT(
					"Hinge joint bias limit is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, GDJOLT_HINGE_JOINT_DEFAULT_SOFTNESS)) {
				WARN_PRINT(
					"Hinge joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, GDJOLT_HINGE_JOINT_DEFAULT_RELAXATION)) {
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

			const double max_torque = motor_max_impulse / calculate_physics_step();

			JPH::MotorSettings& motor_settings = jolt_constraint->GetMotorSettings();
			motor_settings.mMinTorqueLimit = (float)-max_torque;
			motor_settings.mMaxTorqueLimit = (float)+max_torque;
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint parameter: '{}'", p_param));
		} break;
	}
}

bool JoltHingeJoint3D::get_flag(PhysicsServer3D::HingeJointFlag p_flag) {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL_D(jolt_constraint);

	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			return use_limits;
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			return jolt_constraint->GetMotorState() != JPH::EMotorState::Off;
		} break;
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint flag: '{}'", p_flag));
		} break;
	}
}

void JoltHingeJoint3D::set_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled) {
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
			ERR_FAIL_MSG(vformat("Unhandled hinge joint flag: '{}'", p_flag));
		} break;
	}
}

void JoltHingeJoint3D::limits_changed() {
	auto* jolt_constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	if (use_limits) {
		jolt_constraint->SetLimits((float)limit_lower, (float)limit_upper);
	} else {
		jolt_constraint->SetLimits(-JPH::JPH_PI, JPH::JPH_PI);
	}
}
