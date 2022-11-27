#pragma once

#include "jolt_physics_collision_object_3d.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"

class JoltPhysicsBody3D final : public JoltPhysicsCollisionObject3D {
public:
	~JoltPhysicsBody3D() override;

	Variant get_state(PhysicsServer3D::BodyState p_state);

	void set_state(PhysicsServer3D::BodyState p_state, const Variant& p_value);

	Variant get_param(PhysicsServer3D::BodyParameter p_param) const;

	void set_param(PhysicsServer3D::BodyParameter p_param, const Variant& p_value);

	bool has_state_sync_callback() const { return body_state_callback.is_valid(); }

	void set_state_sync_callback(const Callable& p_callback);

	bool is_sleeping(bool p_lock = true) const;

	void set_sleep_state(bool p_enabled, bool p_lock = true);

	bool can_sleep() const override { return allowed_sleep; }

	void set_can_sleep(bool p_enabled, bool p_lock = true);

	Basis get_inverse_inertia_tensor(bool p_lock = true) const;

	Vector3 get_linear_velocity(bool p_lock = true) const;

	void set_linear_velocity(const Vector3& p_velocity, bool p_lock = true);

	Vector3 get_angular_velocity(bool p_lock = true) const;

	void set_angular_velocity(const Vector3& p_velocity, bool p_lock = true);

	void add_constant_central_force(const Vector3& p_force) { constant_force += p_force; }

	void add_constant_force(const Vector3& p_force, const Vector3& p_position = Vector3()) {
		constant_force += p_force;
		constant_torque += (p_position - get_center_of_mass()).cross(p_force);
	}

	void add_constant_torque(const Vector3& p_torque) { constant_torque += p_torque; }

	Vector3 get_constant_force() const { return constant_force; }

	void set_constant_force(const Vector3& p_force) { constant_force = p_force; }

	Vector3 get_constant_torque() const { return constant_torque; }

	void set_constant_torque(const Vector3& p_torque) { constant_torque = p_torque; }

	void call_queries() override;

	JoltPhysicsDirectBodyState3D* get_direct_state();

	PhysicsServer3D::BodyMode get_mode() const override { return mode; }

	void set_mode(PhysicsServer3D::BodyMode p_mode, bool p_lock = true);

	float get_mass() const override { return mass; }

	void set_mass(float p_mass, bool p_lock = true);

	Vector3 get_inertia() const override { return inertia; }

	void set_inertia(const Vector3& p_inertia, bool p_lock = true);

	float get_bounce() const override { return bounce; }

	void set_bounce(float p_bounce, bool p_lock = true);

	float get_friction() const override { return friction; }

	void set_friction(float p_friction, bool p_lock = true);

	float get_gravity_scale() const override { return gravity_scale; }

	void set_gravity_scale(float p_scale, bool p_lock = true);

	bool is_sensor() const override { return false; }

private:
	void shapes_changed(bool p_lock) override;

	void mass_properties_changed(bool p_lock);

	PhysicsServer3D::BodyMode mode = PhysicsServer3D::BODY_MODE_RIGID;

	float mass = 1.0f;

	Vector3 inertia;

	float bounce = 0.0f;

	float friction = 1.0f;

	float gravity_scale = 1.0f;

	bool allowed_sleep = true;

	Vector3 constant_force;

	Vector3 constant_torque;

	Callable body_state_callback;

	JoltPhysicsDirectBodyState3D* direct_state = nullptr;
};
