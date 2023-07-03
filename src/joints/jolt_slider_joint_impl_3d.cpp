#include "jolt_slider_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
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

JoltSliderJointImpl3D::JoltSliderJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_body_b, p_local_ref_a, p_local_ref_b) {
	rebuild(p_lock);
}

double JoltSliderJointImpl3D::get_param(PhysicsServer3D::SliderJointParam p_param) const {
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

void JoltSliderJointImpl3D::set_param(
	PhysicsServer3D::SliderJointParam p_param,
	double p_value,
	bool p_lock
) {
	switch (p_param) {
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER: {
			limit_upper = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER: {
			limit_lower = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Slider joint linear limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Slider joint linear limit restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_LIMIT_DAMPING)) {
				WARN_PRINT(vformat(
					"Slider joint linear limit damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_MOTION_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Slider joint linear motion softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_MOTION_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Slider joint linear motion restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_MOTION_DAMPING)) {
				WARN_PRINT(vformat(
					"Slider joint linear motion damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_ORTHO_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Slider joint linear ortho softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_ORTHO_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Slider joint linear ortho restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LINEAR_ORTHO_DAMPING)) {
				WARN_PRINT(vformat(
					"Slider joint linear ortho damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_UPPER: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_UPPER)) {
				WARN_PRINT(vformat(
					"Slider joint angular limits are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"Try using Generic6DOFJoint3D instead. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_LOWER: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_LOWER)) {
				WARN_PRINT(vformat(
					"Slider joint angular limits are not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"Try using Generic6DOFJoint3D instead. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Slider joint angular limit softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Slider joint angular limit restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_LIMIT_DAMPING)) {
				WARN_PRINT(vformat(
					"Slider joint angular limit damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_MOTION_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Slider joint angular motion softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_MOTION_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Slider joint angular motion restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_MOTION_DAMPING)) {
				WARN_PRINT(vformat(
					"Slider joint angular motion damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ORTHO_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Slider joint angular ortho softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ORTHO_RESTITUTION)) {
				WARN_PRINT(vformat(
					"Slider joint angular ortho restitution is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_ANGULAR_ORTHO_DAMPING)) {
				WARN_PRINT(vformat(
					"Slider joint angular ortho damping is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					bodies_to_string()
				));
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled slider joint parameter: '%d'", p_param));
		} break;
	}
}

void JoltSliderJointImpl3D::rebuild(bool p_lock) {
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
	float limit = FLT_MAX;

	if (limit_lower <= limit_upper) {
		const double limit_midpoint = (limit_lower + limit_upper) / 2.0f;

		ref_shift = float(-limit_midpoint);
		limit = float(limit_upper - limit_midpoint);
	}

	Transform3D shifted_ref_a;
	Transform3D shifted_ref_b;

	shift_reference_frames(Vector3(ref_shift, 0.0f, 0.0f), Vector3(), shifted_ref_a, shifted_ref_b);

	if (is_fixed()) {
		jolt_ref = build_fixed(jolt_body_a, jolt_body_b, shifted_ref_a, shifted_ref_b);
	} else {
		jolt_ref = build_slider(jolt_body_a, jolt_body_b, shifted_ref_a, shifted_ref_b, limit);
	}

	space->add_joint(this);
}

JPH::Constraint* JoltSliderJointImpl3D::build_slider(
	JPH::Body* p_jolt_body_a,
	JPH::Body* p_jolt_body_b,
	const Transform3D& p_shifted_ref_a,
	const Transform3D& p_shifted_ref_b,
	float p_limit
) {
	JPH::SliderConstraintSettings constraint_settings;

	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mAutoDetectPoint = false;
	constraint_settings.mPoint1 = to_jolt(p_shifted_ref_a.origin);
	constraint_settings.mSliderAxis1 = to_jolt(p_shifted_ref_a.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mNormalAxis1 = to_jolt(-p_shifted_ref_a.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mPoint2 = to_jolt(p_shifted_ref_b.origin);
	constraint_settings.mSliderAxis2 = to_jolt(p_shifted_ref_b.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mNormalAxis2 = to_jolt(-p_shifted_ref_b.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mLimitsMin = -p_limit;
	constraint_settings.mLimitsMax = p_limit;

	if (p_jolt_body_b != nullptr) {
		return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
	} else {
		return constraint_settings.Create(*p_jolt_body_a, JPH::Body::sFixedToWorld);
	}
}

JPH::Constraint* JoltSliderJointImpl3D::build_fixed(
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

void JoltSliderJointImpl3D::limits_changed(bool p_lock) {
	rebuild(p_lock);
}
