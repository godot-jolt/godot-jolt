#include "jolt_physics_direct_body_state_3d.hpp"

#include "jolt_body_3d.hpp"
#include "jolt_physics_direct_space_state_3d.hpp"
#include "jolt_space_3d.hpp"

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
	ERR_FAIL_NULL_D(body);
	return body->get_center_of_mass();
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

void JoltPhysicsDirectBodyState3D::_set_linear_velocity(const Vector3& p_velocity) {
	ERR_FAIL_NULL(body);
	return body->set_linear_velocity(p_velocity);
}

Vector3 JoltPhysicsDirectBodyState3D::_get_angular_velocity() const {
	ERR_FAIL_NULL_D(body);
	return body->get_angular_velocity();
}

void JoltPhysicsDirectBodyState3D::_set_angular_velocity(const Vector3& p_velocity) {
	ERR_FAIL_NULL(body);
	return body->set_angular_velocity(p_velocity);
}

void JoltPhysicsDirectBodyState3D::_set_transform(const Transform3D& p_transform) {
	ERR_FAIL_NULL(body);
	return body->set_transform(p_transform);
}

Transform3D JoltPhysicsDirectBodyState3D::_get_transform() const {
	ERR_FAIL_NULL_D(body);
	return body->get_transform();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_velocity_at_local_position(
	const Vector3& p_local_position
) const {
	ERR_FAIL_NULL_D(body);
	return body->get_velocity_at_position(body->get_position() + p_local_position);
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

int32_t JoltPhysicsDirectBodyState3D::_get_contact_count() const {
	ERR_FAIL_NULL_D(body);
	return body->get_contact_count();
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_local_position(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).position;
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_local_normal(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).normal;
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_impulse(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	const JoltBody3D::Contact& contact = body->get_contact(p_contact_idx);
	return contact.impulse;
}

int32_t JoltPhysicsDirectBodyState3D::_get_contact_local_shape(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).shape_index;
}

RID JoltPhysicsDirectBodyState3D::_get_contact_collider(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).collider_rid;
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_collider_position(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).collider_position;
}

uint64_t JoltPhysicsDirectBodyState3D::_get_contact_collider_id(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).collider_id;
}

Object* JoltPhysicsDirectBodyState3D::_get_contact_collider_object(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return ObjectDB::get_instance(body->get_contact(p_contact_idx).collider_id);
}

int32_t JoltPhysicsDirectBodyState3D::_get_contact_collider_shape(int32_t p_contact_idx) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).collider_shape_index;
}

Vector3 JoltPhysicsDirectBodyState3D::_get_contact_collider_velocity_at_position(
	int32_t p_contact_idx
) const {
	ERR_FAIL_NULL_D(body);
	ERR_FAIL_INDEX_D(p_contact_idx, body->get_contact_count());
	return body->get_contact(p_contact_idx).collider_velocity;
}

double JoltPhysicsDirectBodyState3D::_get_step() const {
	ERR_FAIL_NULL_D(body);

	// TODO(mihe): Use `calculate_physics_step` instead (and remove `last_step` entirely) once
	// godotengine/godot-cpp#889 has been fixed
	return (double)body->get_space()->get_last_step();
}

void JoltPhysicsDirectBodyState3D::_integrate_forces() {
	ERR_PRINT("Manual force integration is not supported by Godot Jolt.");
}

PhysicsDirectSpaceState3D* JoltPhysicsDirectBodyState3D::_get_space_state() {
	return body->get_space()->get_direct_state();
}
