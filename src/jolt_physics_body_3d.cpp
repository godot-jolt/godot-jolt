#include "jolt_physics_body_3d.hpp"

#include "body_access.hpp"
#include "conversion.hpp"
#include "error_macros.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"
#include "jolt_physics_space_3d.hpp"
#include "utility_functions.hpp"
#include "vformat.hpp"

JoltPhysicsBody3D::~JoltPhysicsBody3D() {
	if (direct_state) {
		memdelete(direct_state);
		direct_state = nullptr;
	}
}

Variant JoltPhysicsBody3D::get_state(PhysicsServer3D::BodyState p_state) {
	switch (p_state) {
	case PhysicsServer3D::BODY_STATE_TRANSFORM:
		return get_transform();
	case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY:
		return get_linear_velocity();
	case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY:
		return get_angular_velocity();
	case PhysicsServer3D::BODY_STATE_SLEEPING:
		return is_sleeping();
	case PhysicsServer3D::BODY_STATE_CAN_SLEEP:
		return can_sleep();
	default:
		ERR_FAIL_D_MSG(vformat("Unhandled body state: '{}'", p_state));
	}
}

void JoltPhysicsBody3D::set_state(PhysicsServer3D::BodyState p_state, const Variant& p_value) {
	switch (p_state) {
	case PhysicsServer3D::BODY_STATE_TRANSFORM:
		set_transform(p_value);
		break;
	case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY:
		set_linear_velocity(p_value);
		break;
	case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY:
		set_angular_velocity(p_value);
		break;
	case PhysicsServer3D::BODY_STATE_SLEEPING:
		set_sleep_state(p_value);
		break;
	case PhysicsServer3D::BODY_STATE_CAN_SLEEP:
		set_can_sleep(p_value);
		break;
	default:
		ERR_FAIL_MSG(vformat("Unhandled body state: '{}'", p_state));
	}
}

Variant JoltPhysicsBody3D::get_param(PhysicsServer3D::BodyParameter p_param) const {
	switch (p_param) {
	case PhysicsServer3D::BODY_PARAM_BOUNCE:
		return get_bounce();
	case PhysicsServer3D::BODY_PARAM_FRICTION:
		return get_friction();
	case PhysicsServer3D::BODY_PARAM_MASS:
		return get_mass();
	case PhysicsServer3D::BODY_PARAM_INERTIA:
		return get_inertia();
	default:
		ERR_FAIL_D_NOT_IMPL();
	}
}

void JoltPhysicsBody3D::set_param(PhysicsServer3D::BodyParameter p_param, const Variant& p_value) {
	switch (p_param) {
	case PhysicsServer3D::BODY_PARAM_BOUNCE:
		set_bounce(p_value);
		break;
	case PhysicsServer3D::BODY_PARAM_FRICTION:
		set_friction(p_value);
		break;
	case PhysicsServer3D::BODY_PARAM_MASS:
		set_mass(p_value);
		break;
	case PhysicsServer3D::BODY_PARAM_INERTIA:
		set_inertia(p_value);
		break;
	default:
		ERR_FAIL_NOT_IMPL();
	}
}

void JoltPhysicsBody3D::set_state_sync_callback(const Callable& p_callback) {
	body_state_callback = p_callback;
}

bool JoltPhysicsBody3D::is_sleeping(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const BodyAccessRead body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return !body_access.get_body().IsActive();
}

void JoltPhysicsBody3D::set_sleep_state(bool p_enabled, bool p_lock) {
	ERR_FAIL_NULL(space);

	if (p_enabled) {
		space->get_body_iface(p_lock).DeactivateBody(jid);
	} else {
		space->get_body_iface(p_lock).ActivateBody(jid);
	}
}

void JoltPhysicsBody3D::set_can_sleep(bool p_enabled, bool p_lock) {
	if (p_enabled == allowed_sleep) {
		return;
	}

	allowed_sleep = p_enabled;

	if (!space) {
		return;
	}

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetAllowSleeping(allowed_sleep);
}

Basis JoltPhysicsBody3D::get_inverse_inertia_tensor(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const BodyAccessRead body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetInverseInertia().GetQuaternion());
}

Vector3 JoltPhysicsBody3D::get_linear_velocity(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const BodyAccessRead body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetLinearVelocity());
}

void JoltPhysicsBody3D::set_linear_velocity(
	[[maybe_unused]] const Vector3& p_velocity,
	bool p_lock
) {
	ERR_FAIL_NULL(space);

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetLinearVelocityClamped(to_jolt(p_velocity));
}

Vector3 JoltPhysicsBody3D::get_angular_velocity(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const BodyAccessRead body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetAngularVelocity());
}

void JoltPhysicsBody3D::set_angular_velocity(
	[[maybe_unused]] const Vector3& p_velocity,
	bool p_lock
) {
	ERR_FAIL_NULL(space);

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetAngularVelocityClamped(to_jolt(p_velocity));
}

void JoltPhysicsBody3D::call_queries() {
	// TODO(mihe): Call force integration callback

	if (body_state_callback.is_valid()) {
		body_state_callback.callv(Array::make(get_direct_state()));
	}
}

JoltPhysicsDirectBodyState3D* JoltPhysicsBody3D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(JoltPhysicsDirectBodyState3D(this));
	}

	return direct_state;
}

void JoltPhysicsBody3D::set_mode(PhysicsServer3D::BodyMode p_mode, bool p_lock) {
	if (p_mode == mode) {
		return;
	}

	mode = p_mode;

	if (!space) {
		return;
	}

	JPH::EMotionType motion_type = {};

	switch (p_mode) {
	case PhysicsServer3D::BODY_MODE_STATIC:
		motion_type = JPH::EMotionType::Static;
		break;
	case PhysicsServer3D::BODY_MODE_KINEMATIC:
		motion_type = JPH::EMotionType::Kinematic;
		break;
	case PhysicsServer3D::BODY_MODE_RIGID:
	case PhysicsServer3D::BODY_MODE_RIGID_LINEAR:
		motion_type = JPH::EMotionType::Dynamic;
		break;
	default:
		ERR_FAIL_MSG(vformat("Unhandled body mode: '{}'", p_mode));
	}

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	if (!is_sleeping(false) && motion_type == JPH::EMotionType::Static) {
		set_sleep_state(true, false);
	}

	body_access.get_body().SetMotionType(motion_type);

	if (is_sleeping(false) && motion_type != JPH::EMotionType::Static) {
		set_sleep_state(false, false);
	}

	if (motion_type != JPH::EMotionType::Static) {
		mass_properties_changed(false);
	}
}

void JoltPhysicsBody3D::set_mass(float p_mass, bool p_lock) {
	if (p_mass != mass) {
		mass = p_mass;
		mass_properties_changed(p_lock);
	}
}

void JoltPhysicsBody3D::set_inertia(const Vector3& p_inertia, bool p_lock) {
	if (p_inertia != inertia) {
		inertia = p_inertia;
		mass_properties_changed(p_lock);
	}
}

void JoltPhysicsBody3D::set_bounce(float p_bounce, bool p_lock) {
	if (p_bounce < 0 || p_bounce > 1) {
		WARN_PRINT(
			"Bounce values less than 0 or greater than 1 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);

		p_bounce = clamp(p_bounce, 0.0f, 1.0f);
	}

	if (p_bounce == bounce) {
		return;
	}

	bounce = p_bounce;

	if (!space) {
		return;
	}

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetRestitution(bounce);
}

void JoltPhysicsBody3D::set_friction(float p_friction, bool p_lock) {
	if (p_friction < 0) {
		WARN_PRINT(
			"Friction values less than 0 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);

		p_friction = 0;
	}

	if (p_friction == friction) {
		return;
	}

	friction = p_friction;

	if (!space) {
		return;
	}

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetFriction(friction);
}

void JoltPhysicsBody3D::shapes_changed(bool p_lock) {
	mass_properties_changed(p_lock);
}

void JoltPhysicsBody3D::mass_properties_changed(bool p_lock) {
	if (!space || mode == PhysicsServer3D::BODY_MODE_STATIC) {
		return;
	}

	const BodyAccessWrite body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	const JPH::MassProperties mass_properties = calculate_mass_properties(false);
	body_access.get_body().GetMotionProperties()->SetMassProperties(mass_properties);
}
