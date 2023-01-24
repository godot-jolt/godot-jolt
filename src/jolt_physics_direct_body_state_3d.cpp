#include "jolt_physics_direct_body_state_3d.hpp"

#include "jolt_body_3d.hpp"

JoltPhysicsDirectBodyState3D::JoltPhysicsDirectBodyState3D(JoltBody3D* p_body)
	: body(p_body) { }

Vector3 JoltPhysicsDirectBodyState3D::_get_total_gravity() const {
	ERR_FAIL_D_NOT_IMPL();
}

double JoltPhysicsDirectBodyState3D::_get_total_angular_damp() const {
	ERR_FAIL_D_NOT_IMPL();
}

double JoltPhysicsDirectBodyState3D::_get_total_linear_damp() const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_center_of_mass() const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_center_of_mass_local() const {
	ERR_FAIL_D_NOT_IMPL();
}

Basis JoltPhysicsDirectBodyState3D::_get_principal_inertia_axes() const {
	ERR_FAIL_D_NOT_IMPL();
}

double JoltPhysicsDirectBodyState3D::_get_inverse_mass() const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_inverse_inertia() const {
	ERR_FAIL_D_NOT_IMPL();
}

Basis JoltPhysicsDirectBodyState3D::_get_inverse_inertia_tensor() const {
	ERR_FAIL_NULL_D(body);
	return body->get_inverse_inertia_tensor();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_linear_velocity() const {
	ERR_FAIL_NULL_D(body);
	return body->get_linear_velocity();
}

void JoltPhysicsDirectBodyState3D::_set_linear_velocity([[maybe_unused]] const Vector3& p_velocity
) {
	ERR_FAIL_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_angular_velocity() const {
	ERR_FAIL_NULL_D(body);
	return body->get_angular_velocity();
}

void JoltPhysicsDirectBodyState3D::_set_angular_velocity([[maybe_unused]] const Vector3& p_velocity
) {
	ERR_FAIL_NOT_IMPL();
}

void JoltPhysicsDirectBodyState3D::_set_transform([[maybe_unused]] const Transform3D& p_transform) {
	ERR_FAIL_NOT_IMPL();
}

Transform3D JoltPhysicsDirectBodyState3D::_get_transform() const {
	ERR_FAIL_NULL_D(body);
	return body->get_transform();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_velocity_at_local_position(
	[[maybe_unused]] const Vector3& p_local_position
) const {
	ERR_FAIL_D_NOT_IMPL();
}

void JoltPhysicsDirectBodyState3D::_apply_central_impulse(const Vector3& p_impulse) {
	ERR_FAIL_NULL(body);
	return body->apply_central_impulse(p_impulse);
}

void JoltPhysicsDirectBodyState3D::_apply_impulse(
	const Vector3& p_impulse,
	const Vector3& p_position
) {
	ERR_FAIL_NULL(body);
	return body->apply_impulse(p_impulse, p_position);
}

void JoltPhysicsDirectBodyState3D::_apply_torque_impulse(const Vector3& p_impulse) {
	ERR_FAIL_NULL(body);
	return body->apply_torque_impulse(p_impulse);
}

void JoltPhysicsDirectBodyState3D::_apply_central_force(const Vector3& p_force) {
	ERR_FAIL_NULL(body);
	return body->apply_central_force(p_force);
}

void JoltPhysicsDirectBodyState3D::_apply_force(const Vector3& p_force, const Vector3& p_position) {
	ERR_FAIL_NULL(body);
	return body->apply_force(p_force, p_position);
}

void JoltPhysicsDirectBodyState3D::_apply_torque(const Vector3& p_torque) {
	ERR_FAIL_NULL(body);
	return body->apply_torque(p_torque);
}

void JoltPhysicsDirectBodyState3D::_add_constant_central_force(const Vector3& p_force) {
	ERR_FAIL_NULL(body);
	return body->add_constant_central_force(p_force);
}

void JoltPhysicsDirectBodyState3D::_add_constant_force(
	const Vector3& p_force,
	const Vector3& p_position
) {
	ERR_FAIL_NULL(body);
	return body->add_constant_force(p_force, p_position);
}

void JoltPhysicsDirectBodyState3D::_add_constant_torque(const Vector3& p_torque) {
	ERR_FAIL_NULL(body);
	return body->add_constant_torque(p_torque);
}

Vector3 JoltPhysicsDirectBodyState3D::_get_constant_force() const {
	ERR_FAIL_NULL_D(body);
	return body->get_constant_force();
}

void JoltPhysicsDirectBodyState3D::_set_constant_force(const Vector3& p_force) {
	ERR_FAIL_NULL(body);
	return body->set_constant_force(p_force);
}

Vector3 JoltPhysicsDirectBodyState3D::_get_constant_torque() const {
	ERR_FAIL_NULL_D(body);
	return body->get_constant_torque();
}

void JoltPhysicsDirectBodyState3D::_set_constant_torque(const Vector3& p_torque) {
	ERR_FAIL_NULL(body);
	return body->set_constant_torque(p_torque);
}

bool JoltPhysicsDirectBodyState3D::_is_sleeping() const {
	ERR_FAIL_NULL_D(body);
	return body->get_sleep_state();
}

void JoltPhysicsDirectBodyState3D::_set_sleep_state(bool p_enabled) {
	ERR_FAIL_NULL(body);
	body->set_sleep_state(p_enabled);
}

int64_t JoltPhysicsDirectBodyState3D::_get_contact_count() const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_local_position(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_local_normal(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_impulse([[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

int64_t JoltPhysicsDirectBodyState3D::_get_contact_local_shape(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

RID JoltPhysicsDirectBodyState3D::_get_contact_collider([[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_collider_position(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

int64_t JoltPhysicsDirectBodyState3D::_get_contact_collider_id(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

Object* JoltPhysicsDirectBodyState3D::_get_contact_collider_object(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

int64_t JoltPhysicsDirectBodyState3D::_get_contact_collider_shape(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_collider_velocity_at_position(
	[[maybe_unused]] int64_t p_contact_idx
) const {
	ERR_FAIL_D_NOT_IMPL();
}

double JoltPhysicsDirectBodyState3D::_get_step() const {
	ERR_FAIL_D_NOT_IMPL();
}

void JoltPhysicsDirectBodyState3D::_integrate_forces() {
	ERR_PRINT("Manual force integration is not supported by Godot Jolt.");
}

PhysicsDirectSpaceState3D* JoltPhysicsDirectBodyState3D::_get_space_state() {
	ERR_FAIL_D_NOT_IMPL();
}
