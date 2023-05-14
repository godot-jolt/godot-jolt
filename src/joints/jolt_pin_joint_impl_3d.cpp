#include "jolt_pin_joint_impl_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_BIAS = 0.3;
constexpr double DEFAULT_DAMPING = 1.0;
constexpr double DEFAULT_IMPULSE_CLAMP = 0.0;

} // namespace

JoltPinJointImpl3D::JoltPinJointImpl3D(
	JoltSpace3D* p_space,
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Vector3& p_local_a,
	const Vector3& p_local_b,
	bool p_lock
)
	: JoltJointImpl3D(p_space, p_body_a, p_body_b)
	, local_a(p_local_a)
	, local_b(p_local_b) {
	const JPH::BodyID body_ids[] = {body_a->get_jolt_id(), body_b->get_jolt_id()};
	const JoltWritableBodies3D bodies = space->write_bodies(body_ids, count_of(body_ids), p_lock);

	const JoltWritableBody3D jolt_body_a = bodies[0];
	ERR_FAIL_COND(jolt_body_a.is_invalid());

	const JoltWritableBody3D jolt_body_b = bodies[1];
	ERR_FAIL_COND(jolt_body_b.is_invalid());

	const JoltObjectImpl3D& object_a = *jolt_body_a.as_object();
	const JoltObjectImpl3D& object_b = *jolt_body_b.as_object();

	const JPH::Vec3 point_scaled_a = to_jolt(local_a * object_a.get_scale());
	const JPH::Vec3 point_scaled_b = to_jolt(local_b * object_b.get_scale());

	const JPH::Vec3 com_scaled_a = jolt_body_a->GetShape()->GetCenterOfMass();
	const JPH::Vec3 com_scaled_b = jolt_body_b->GetShape()->GetCenterOfMass();

	JPH::PointConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = point_scaled_a - com_scaled_a;
	constraint_settings.mPoint2 = point_scaled_b - com_scaled_b;

	jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);

	space->add_joint(this);
}

JoltPinJointImpl3D::JoltPinJointImpl3D(
	JoltSpace3D* p_space,
	JoltBodyImpl3D* p_body_a,
	const Vector3& p_local_a,
	const Vector3& p_local_b,
	bool p_lock
)
	: JoltJointImpl3D(p_space, p_body_a)
	, local_a(p_local_a)
	, local_b(p_local_b) {
	const JoltWritableBody3D jolt_body_a = space->write_body(*body_a, p_lock);
	ERR_FAIL_COND(jolt_body_a.is_invalid());

	const JoltObjectImpl3D& object_a = *jolt_body_a.as_object();

	const JPH::Vec3 point_scaled_a = to_jolt(local_a * object_a.get_scale());
	const JPH::Vec3 point_scaled_b = to_jolt(local_b);

	const JPH::Vec3 com_scaled_a = jolt_body_a->GetShape()->GetCenterOfMass();

	JPH::PointConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mPoint1 = point_scaled_a - com_scaled_a;
	constraint_settings.mPoint2 = point_scaled_b;

	jolt_ref = constraint_settings.Create(*jolt_body_a, JPH::Body::sFixedToWorld);

	space->add_joint(this);
}

void JoltPinJointImpl3D::set_local_a(const Vector3& p_local_a, bool p_lock) {
	local_a = p_local_a;

	auto* jolt_constraint = static_cast<JPH::PointConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	const JoltReadableBody3D jolt_body_a = space->read_body(*body_a, p_lock);
	ERR_FAIL_COND(jolt_body_a.is_invalid());

	const JoltObjectImpl3D& object_a = *jolt_body_a.as_object();
	const JPH::Vec3 point_scaled_a = to_jolt(local_a * object_a.get_scale());
	const JPH::Vec3 com_scaled_a = jolt_body_a->GetShape()->GetCenterOfMass();

	jolt_constraint->SetPoint1(
		JPH::EConstraintSpace::LocalToBodyCOM,
		point_scaled_a - com_scaled_a
	);
}

void JoltPinJointImpl3D::set_local_b(const Vector3& p_local_b, bool p_lock) {
	local_b = p_local_b;

	auto* jolt_constraint = static_cast<JPH::PointConstraint*>(jolt_ref.GetPtr());
	ERR_FAIL_NULL(jolt_constraint);

	if (body_b != nullptr) {
		const JoltReadableBody3D jolt_body_b = space->read_body(*body_b, p_lock);
		ERR_FAIL_COND(jolt_body_b.is_invalid());

		const JoltObjectImpl3D& object_b = *jolt_body_b.as_object();
		const JPH::Vec3 point_scaled_b = to_jolt(local_b * object_b.get_scale());
		const JPH::Vec3 com_scaled_b = jolt_body_b->GetShape()->GetCenterOfMass();

		jolt_constraint->SetPoint2(
			JPH::EConstraintSpace::LocalToBodyCOM,
			point_scaled_b - com_scaled_b
		);
	} else {
		jolt_constraint->SetPoint2(JPH::EConstraintSpace::LocalToBodyCOM, to_jolt(local_b));
	}
}

double JoltPinJointImpl3D::get_param(PhysicsServer3D::PinJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS: {
			return DEFAULT_BIAS;
		}
		case PhysicsServer3D::PIN_JOINT_DAMPING: {
			return DEFAULT_DAMPING;
		}
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP: {
			return DEFAULT_IMPULSE_CLAMP;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled pin joint parameter: '%d'", p_param));
		}
	}
}

void JoltPinJointImpl3D::set_param(PhysicsServer3D::PinJointParam p_param, double p_value) {
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(
					"Pin joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::PIN_JOINT_DAMPING: {
			if (!Math::is_equal_approx(p_value, DEFAULT_DAMPING)) {
				WARN_PRINT(
					"Pin joint damping is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP: {
			if (!Math::is_equal_approx(p_value, DEFAULT_IMPULSE_CLAMP)) {
				WARN_PRINT(
					"Pin joint impulse clamp is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled pin joint parameter: '%d'", p_param));
		} break;
	}
}
