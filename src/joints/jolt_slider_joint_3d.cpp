#include "jolt_slider_joint_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_LINEAR_LIMIT_SOFTNESS = 1.0;
constexpr double DEFAULT_LINEAR_LIMIT_RESTITUTION = 0.7;
constexpr double DEFAULT_LINEAR_LIMIT_DAMPING = 1.0;

constexpr double DEFAULT_LINEAR_MOTION_SOFTNESS = 1.0;
constexpr double DEFAULT_LINEAR_MOTION_RESTITUTION = 0.7;
constexpr double DEFAULT_LINEAR_MOTION_DAMPING = 0.0;

constexpr double DEFAULT_LINEAR_ORTHO_SOFTNESS = 1.0;
constexpr double DEFAULT_LINEAR_ORTHO_RESTITUTION = 0.7;
constexpr double DEFAULT_LINEAR_ORTHO_DAMPING = 1.0;

constexpr double DEFAULT_ANGULAR_LIMIT_UPPER = 0.0;
constexpr double DEFAULT_ANGULAR_LIMIT_LOWER = 0.0;
constexpr double DEFAULT_ANGULAR_LIMIT_SOFTNESS = 1.0;
constexpr double DEFAULT_ANGULAR_LIMIT_RESTITUTION = 0.7;
constexpr double DEFAULT_ANGULAR_LIMIT_DAMPING = 0.0;

constexpr double DEFAULT_ANGULAR_MOTION_SOFTNESS = 1.0;
constexpr double DEFAULT_ANGULAR_MOTION_RESTITUTION = 0.7;
constexpr double DEFAULT_ANGULAR_MOTION_DAMPING = 1.0;

constexpr double DEFAULT_ANGULAR_ORTHO_SOFTNESS = 1.0;
constexpr double DEFAULT_ANGULAR_ORTHO_RESTITUTION = 0.7;
constexpr double DEFAULT_ANGULAR_ORTHO_DAMPING = 1.0;

} // namespace

JoltSliderJoint3D::JoltSliderJoint3D(
	JoltSpace3D* p_space,
	JoltBody3D* p_body_a,
	JoltBody3D* p_body_b,
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

	JPH::SliderConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mAutoDetectPoint = false;
	constraint_settings.mPoint1 = point_scaled_a - com_scaled_a;
	constraint_settings.mSliderAxis1 = to_jolt(p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mNormalAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPoint2 = point_scaled_b - com_scaled_b;
	constraint_settings.mSliderAxis2 = to_jolt(p_local_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mNormalAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltSliderJoint3D::JoltSliderJoint3D(
	JoltSpace3D* p_space,
	JoltBody3D* p_body_a,
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

	JPH::SliderConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mAutoDetectPoint = false;
	constraint_settings.mPoint1 = point_scaled_a - com_scaled_a;
	constraint_settings.mSliderAxis1 = to_jolt(p_local_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mNormalAxis1 = to_jolt(-p_local_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPoint2 = point_scaled_b;
	constraint_settings.mSliderAxis2 = to_jolt(p_local_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mNormalAxis2 = to_jolt(-p_local_ref_b.basis.get_column(Vector3::AXIS_Z));

	jolt_ref = constraint_settings.Create(*jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

double JoltSliderJoint3D::get_param(PhysicsServer3D::SliderJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER: {
			return limit_upper;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER: {
			return limit_lower;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS: {
			return DEFAULT_LINEAR_LIMIT_SOFTNESS;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION: {
			return DEFAULT_LINEAR_LIMIT_RESTITUTION;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_DAMPING: {
			return DEFAULT_LINEAR_LIMIT_DAMPING;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS: {
			return DEFAULT_LINEAR_MOTION_SOFTNESS;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION: {
			return DEFAULT_LINEAR_MOTION_RESTITUTION;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_DAMPING: {
			return DEFAULT_LINEAR_MOTION_DAMPING;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS: {
			return DEFAULT_LINEAR_ORTHO_SOFTNESS;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION: {
			return DEFAULT_LINEAR_ORTHO_RESTITUTION;
		}
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING: {
			return DEFAULT_LINEAR_ORTHO_DAMPING;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_UPPER: {
			return DEFAULT_ANGULAR_LIMIT_UPPER;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_LOWER: {
			return DEFAULT_ANGULAR_LIMIT_LOWER;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS: {
			return DEFAULT_ANGULAR_LIMIT_SOFTNESS;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION: {
			return DEFAULT_ANGULAR_LIMIT_RESTITUTION;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING: {
			return DEFAULT_ANGULAR_LIMIT_DAMPING;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS: {
			return DEFAULT_ANGULAR_MOTION_SOFTNESS;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION: {
			return DEFAULT_ANGULAR_MOTION_RESTITUTION;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_DAMPING: {
			return DEFAULT_ANGULAR_MOTION_DAMPING;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS: {
			return DEFAULT_ANGULAR_ORTHO_SOFTNESS;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION: {
			return DEFAULT_ANGULAR_ORTHO_RESTITUTION;
		}
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING: {
			return DEFAULT_ANGULAR_ORTHO_DAMPING;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled slider joint parameter: '%d'", p_param));
		}
	}
}

void JoltSliderJoint3D::set_param(PhysicsServer3D::SliderJointParam p_param, double p_value) {
	switch (p_param) {
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER: {
			limit_upper = p_value;
			limits_changed();
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER: {
			limit_lower = p_value;
			limits_changed();
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(
					"Slider joint linear limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_RESTITUTION)) {
				WARN_PRINT(
					"Slider joint linear limit restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_DAMPING)) {
				WARN_PRINT(
					"Slider joint linear limit damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_MOTION_SOFTNESS)) {
				WARN_PRINT(
					"Slider joint linear motion softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_MOTION_RESTITUTION)) {
				WARN_PRINT(
					"Slider joint linear motion restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_MOTION_DAMPING)) {
				WARN_PRINT(
					"Slider joint linear motion damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_ORTHO_SOFTNESS)) {
				WARN_PRINT(
					"Slider joint linear ortho softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_ORTHO_RESTITUTION)) {
				WARN_PRINT(
					"Slider joint linear ortho restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_ORTHO_DAMPING)) {
				WARN_PRINT(
					"Slider joint linear ortho damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_UPPER: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_UPPER)) {
				WARN_PRINT(
					"Slider joint angular limits are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"Try using Generic6DOFJoint3D instead."
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_LOWER: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_LOWER)) {
				WARN_PRINT(
					"Slider joint angular limits are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"Try using Generic6DOFJoint3D instead."
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(
					"Slider joint angular limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_RESTITUTION)) {
				WARN_PRINT(
					"Slider joint angular limit restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_DAMPING)) {
				WARN_PRINT(
					"Slider joint angular limit damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_MOTION_SOFTNESS)) {
				WARN_PRINT(
					"Slider joint angular motion softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_MOTION_RESTITUTION)) {
				WARN_PRINT(
					"Slider joint angular motion restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_MOTION_DAMPING)) {
				WARN_PRINT(
					"Slider joint angular motion damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ORTHO_SOFTNESS)) {
				WARN_PRINT(
					"Slider joint angular ortho softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ORTHO_RESTITUTION)) {
				WARN_PRINT(
					"Slider joint angular ortho restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ORTHO_DAMPING)) {
				WARN_PRINT(
					"Slider joint angular ortho damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled slider joint parameter: '%d'", p_param));
		} break;
	}
}

void JoltSliderJoint3D::limits_changed() {
	auto* jolt_constraint = static_cast<JPH::SliderConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	constexpr double basically_pos_zero = CMP_EPSILON;
	constexpr double basically_neg_zero = -CMP_EPSILON;

	if (limit_lower > basically_pos_zero) {
		WARN_PRINT(
			"Slider joint lower distances greater than 0 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);
	}

	if (limit_upper < basically_neg_zero) {
		WARN_PRINT(
			"Slider joint upper distances less than 0 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);
	}

	const float limit_lower_clamped = min((float)limit_lower, 0.0f);
	const float limit_upper_clamped = max((float)limit_upper, 0.0f);

	jolt_constraint->SetLimits(limit_lower_clamped, limit_upper_clamped);
}
