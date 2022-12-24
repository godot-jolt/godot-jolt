#include "jolt_joint_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "jolt_body_3d.hpp"
#include "jolt_body_access_3d.hpp"
#include "utility_functions.hpp"
#include "variant.hpp"

constexpr int64_t GDJOLT_JOINT_DEFAULT_SOLVER_PRIORITY = 1;
constexpr double GDJOLT_PIN_JOINT_DEFAULT_BIAS = 0.3;
constexpr double GDJOLT_PIN_JOINT_DEFAULT_DAMPING = 1.0;
constexpr double GDJOLT_PIN_JOINT_DEFAULT_IMPULSE_CLAMP = 0.0;

JoltJoint3D::JoltJoint3D(JoltSpace3D* p_space)
	: space(p_space) { }

JoltJoint3D::~JoltJoint3D() {
	if (jolt_ref != nullptr) {
		space->remove_joint(this);
	}
}

int64_t JoltJoint3D::get_solver_priority() const {
	return GDJOLT_JOINT_DEFAULT_SOLVER_PRIORITY;
}

void JoltJoint3D::set_solver_priority(int64_t p_priority) {
	if (p_priority != GDJOLT_JOINT_DEFAULT_SOLVER_PRIORITY) {
		WARN_PRINT(
			"Joint solver priority is not supported by Godot Jolt. "
			"Any such value will be ignored."
		);
	}
}

JoltPinJoint3D::JoltPinJoint3D(
	JoltSpace3D* p_space,
	const JoltBody3D* p_body_a,
	const JoltBody3D* p_body_b,
	const Vector3& p_local_a,
	const Vector3& p_local_b,
	bool p_lock
)
	: JoltJoint3D(p_space)
	, body_a(p_body_a)
	, body_b(p_body_b)
	, local_a(p_local_a)
	, local_b(p_local_b) {
	if (body_b != nullptr) {
		const JPH::BodyID body_ids[] = {body_a->get_jolt_id(), body_b->get_jolt_id()};
		const JoltMultiBodyAccessWrite3D body_access(*space, body_ids, count_of(body_ids), p_lock);

		JPH::Body* jolt_body_a = body_access.get_body(0);
		ERR_FAIL_NULL(jolt_body_a);

		JPH::Body* jolt_body_b = body_access.get_body(1);
		ERR_FAIL_NULL(jolt_body_b);

		const JPH::Mat44 world_transform_a = jolt_body_a->GetWorldTransform();
		const JPH::Mat44 world_transform_b = jolt_body_b->GetWorldTransform();

		JPH::PointConstraintSettings constraint_settings;
		constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
		constraint_settings.mPoint1 = world_transform_a * to_jolt(local_a);
		constraint_settings.mPoint2 = world_transform_b * to_jolt(local_b);

		jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);
	} else {
		const JoltBodyAccessWrite3D body_access(*space, body_a->get_jolt_id(), p_lock);
		ERR_FAIL_COND(!body_access.is_valid());

		JPH::Body& jolt_body_a = body_access.get_body();

		const JPH::Mat44 world_transform_a = jolt_body_a.GetWorldTransform();

		JPH::PointConstraintSettings constraint_settings;
		constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
		constraint_settings.mPoint1 = world_transform_a * to_jolt(local_a);
		constraint_settings.mPoint2 = to_jolt(local_b);

		jolt_ref = constraint_settings.Create(jolt_body_a, JPH::Body::sFixedToWorld);
	}

	space->add_joint(this);
}

double JoltPinJoint3D::get_param(PhysicsServer3D::PinJointParam p_param) {
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS: {
			return GDJOLT_PIN_JOINT_DEFAULT_BIAS;
		}
		case PhysicsServer3D::PIN_JOINT_DAMPING: {
			return GDJOLT_PIN_JOINT_DEFAULT_DAMPING;
		}
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP: {
			return GDJOLT_PIN_JOINT_DEFAULT_IMPULSE_CLAMP;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled pin joint parameter: '{}'", p_param));
		}
	}
}

void JoltPinJoint3D::set_param(PhysicsServer3D::PinJointParam p_param, double p_value) {
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, GDJOLT_PIN_JOINT_DEFAULT_BIAS)) {
				WARN_PRINT(
					"Pin joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::PIN_JOINT_DAMPING: {
			if (!Math::is_equal_approx(p_value, GDJOLT_PIN_JOINT_DEFAULT_DAMPING)) {
				WARN_PRINT(
					"Pin joint damping is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP: {
			if (!Math::is_equal_approx(p_value, GDJOLT_PIN_JOINT_DEFAULT_IMPULSE_CLAMP)) {
				WARN_PRINT(
					"Pin joint impulse clamp is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled pin joint parameter: '{}'", p_param));
		} break;
	}
}
