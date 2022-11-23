#include "jolt_physics_body_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"
#include "jolt_physics_space_3d.hpp"
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
	default:
		ERR_FAIL_V_NOT_IMPL({});
	}
}

void JoltPhysicsBody3D::set_state(PhysicsServer3D::BodyState p_state, const Variant& p_value) {
	// NOLINTNEXTLINE(hicpp-multiway-paths-covered)
	switch (p_state) {
	case PhysicsServer3D::BODY_STATE_TRANSFORM:
		set_transform(p_value);
		break;
	default:
		ERR_FAIL_NOT_IMPL();
	}
}

Variant JoltPhysicsBody3D::get_param(PhysicsServer3D::BodyParameter p_param) const {
	switch (p_param) {
	case PhysicsServer3D::BODY_PARAM_MASS:
		return get_mass();
	case PhysicsServer3D::BODY_PARAM_INERTIA:
		return get_inertia();
	default:
		ERR_FAIL_V_NOT_IMPL({});
	}
}

void JoltPhysicsBody3D::set_param(PhysicsServer3D::BodyParameter p_param, const Variant& p_value) {
	switch (p_param) {
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
	ERR_FAIL_COND_V(!space, false);

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface =
		p_lock ? system->GetBodyInterface() : system->GetBodyInterfaceNoLock();

	return !body_iface.IsActive(jid);
}

Basis JoltPhysicsBody3D::get_inverse_inertia_tensor(bool p_lock) const {
	ERR_FAIL_COND_V(!space, Basis());

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface =
		p_lock ? system->GetBodyInterface() : system->GetBodyInterfaceNoLock();

	return to_godot(body_iface.GetInverseInertia(jid).GetQuaternion());
}

Vector3 JoltPhysicsBody3D::get_linear_velocity(bool p_lock) const {
	ERR_FAIL_COND_V(!space, Vector3());

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface =
		p_lock ? system->GetBodyInterface() : system->GetBodyInterfaceNoLock();

	return to_godot(body_iface.GetLinearVelocity(jid));
}

void JoltPhysicsBody3D::set_linear_velocity([[maybe_unused]] const Vector3& p_velocity) {
	ERR_FAIL_NOT_IMPL();
}

Vector3 JoltPhysicsBody3D::get_angular_velocity(bool p_lock) const {
	ERR_FAIL_COND_V(!space, Vector3());

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface =
		p_lock ? system->GetBodyInterface() : system->GetBodyInterfaceNoLock();

	return to_godot(body_iface.GetAngularVelocity(jid));
}

void JoltPhysicsBody3D::set_angular_velocity([[maybe_unused]] const Vector3& p_velocity) {
	ERR_FAIL_NOT_IMPL();
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

void JoltPhysicsBody3D::set_mode(PhysicsServer3D::BodyMode p_mode) {
	if (p_mode == mode) {
		return;
	}

	mode = p_mode;

	if (!space) {
		return;
	}

	JPH::PhysicsSystem* system = space->get_system();
	JPH::BodyInterface& body_iface = system->GetBodyInterface();

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

	body_iface.SetMotionType(jid, motion_type, JPH::EActivation::Activate);
}

void JoltPhysicsBody3D::set_mass(float p_mass) {
	ERR_FAIL_COND_MSG(space, "Cannot change mass after body has been created.");
	mass = p_mass;
}

void JoltPhysicsBody3D::set_inertia(const Vector3& p_inertia) {
	ERR_FAIL_COND_MSG(space, "Cannot change inertia after body has been created.");
	inertia = p_inertia;
}
