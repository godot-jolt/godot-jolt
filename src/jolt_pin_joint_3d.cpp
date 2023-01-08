#include "jolt_pin_joint_3d.hpp"

#include "jolt_body_3d.hpp"
#include "jolt_body_access_3d.hpp"

namespace {

constexpr double GDJOLT_PIN_JOINT_DEFAULT_BIAS = 0.3;
constexpr double GDJOLT_PIN_JOINT_DEFAULT_DAMPING = 1.0;
constexpr double GDJOLT_PIN_JOINT_DEFAULT_IMPULSE_CLAMP = 0.0;

} // namespace

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
	const JPH::BodyID body_ids[] = {body_a->get_jolt_id(), body_b->get_jolt_id()};
	const JoltMultiBodyAccessWrite3D body_access(*space, body_ids, count_of(body_ids), p_lock);

	JPH::Body* jolt_body_a = body_access.get_body(0);
	ERR_FAIL_NULL(jolt_body_a);

	JPH::Body* jolt_body_b = body_access.get_body(1);
	ERR_FAIL_NULL(jolt_body_b);

	const JPH::Shape& jolt_shape_a = *jolt_body_a->GetShape();
	const JPH::Shape& jolt_shape_b = *jolt_body_b->GetShape();

	JPH::PointConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = to_jolt(local_a) - jolt_shape_a.GetCenterOfMass();
	constraint_settings.mPoint2 = to_jolt(local_b) - jolt_shape_b.GetCenterOfMass();

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltPinJoint3D::JoltPinJoint3D(
	JoltSpace3D* p_space,
	const JoltBody3D* p_body_a,
	const Vector3& p_local_a,
	const Vector3& p_local_b,
	bool p_lock
)
	: JoltJoint3D(p_space)
	, body_a(p_body_a)
	, local_a(p_local_a)
	, local_b(p_local_b) {
	const JoltBodyAccessWrite3D body_access(*space, body_a->get_jolt_id(), p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::Body& jolt_body_a = body_access.get_body();
	const JPH::Shape& jolt_shape_a = *jolt_body_a.GetShape();

	JPH::PointConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = to_jolt(local_a) - jolt_shape_a.GetCenterOfMass();
	constraint_settings.mPoint2 = to_jolt(local_b);

	jolt_ref = constraint_settings.Create(jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

void JoltPinJoint3D::set_local_a(const Vector3& p_local_a, bool p_lock) {
	auto* jolt_constraint = static_cast<JPH::PointConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	const JoltBodyAccessRead3D body_access(*space, body_a->get_jolt_id(), p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	const JPH::Body& jolt_body_a = body_access.get_body();
	const JPH::Shape& jolt_shape_a = *jolt_body_a.GetShape();

	jolt_constraint->SetPoint1(
		JPH::EConstraintSpace::LocalToBodyCOM,
		to_jolt(p_local_a) - jolt_shape_a.GetCenterOfMass()
	);

	local_a = p_local_a;
}

void JoltPinJoint3D::set_local_b(const Vector3& p_local_b, bool p_lock) {
	auto* jolt_constraint = static_cast<JPH::PointConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	if (body_b != nullptr) {
		const JoltBodyAccessRead3D body_access(*space, body_b->get_jolt_id(), p_lock);
		ERR_FAIL_COND(!body_access.is_valid());

		const JPH::Body& jolt_body_b = body_access.get_body();
		const JPH::Shape& jolt_shape_b = *jolt_body_b.GetShape();

		jolt_constraint->SetPoint2(
			JPH::EConstraintSpace::LocalToBodyCOM,
			to_jolt(p_local_b) - jolt_shape_b.GetCenterOfMass()
		);
	} else {
		jolt_constraint->SetPoint2(JPH::EConstraintSpace::LocalToBodyCOM, to_jolt(p_local_b));
	}

	local_b = p_local_b;
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
