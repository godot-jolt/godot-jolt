#pragma once

#include "jolt_collision_object_3d.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"

class JoltBody3D final : public JoltCollisionObject3D {
public:
	~JoltBody3D() override;

	Variant get_state(PhysicsServer3D::BodyState p_state);

	void set_state(PhysicsServer3D::BodyState p_state, const Variant& p_value);

	Variant get_param(PhysicsServer3D::BodyParameter p_param) const;

	void set_param(PhysicsServer3D::BodyParameter p_param, const Variant& p_value);

	bool has_state_sync_callback() const { return body_state_callback.is_valid(); }

	void set_state_sync_callback(const Callable& p_callback) { body_state_callback = p_callback; }

	bool has_force_integration_callback() const { return force_integration_callback.is_valid(); }

	void set_force_integration_callback(const Callable& p_callback, const Variant& p_userdata) {
		force_integration_callback = p_callback;
		force_integration_userdata = p_userdata;
	}

	bool get_initial_sleep_state() const override { return initial_sleep_state; }

	bool get_sleep_state(bool p_lock = true) const;

	void set_sleep_state(bool p_enabled, bool p_lock = true);

	bool can_sleep() const override { return allowed_sleep; }

	void set_can_sleep(bool p_enabled, bool p_lock = true);

	Basis get_inverse_inertia_tensor(bool p_lock = true) const;

	Vector3 get_initial_linear_velocity() const override { return initial_linear_velocity; }

	Vector3 get_initial_angular_velocity() const override { return initial_angular_velocity; }

	Vector3 get_linear_velocity(bool p_lock = true) const;

	void set_linear_velocity(const Vector3& p_velocity, bool p_lock = true);

	Vector3 get_angular_velocity(bool p_lock = true) const;

	void set_angular_velocity(const Vector3& p_velocity, bool p_lock = true);

	bool has_custom_center_of_mass() const override { return custom_center_of_mass; }

	Vector3 get_center_of_mass_custom() const override { return center_of_mass_custom; }

	void set_center_of_mass_custom(const Vector3& p_center_of_mass, bool p_lock = true);

	void reset_mass_properties(bool p_lock = true);

	void apply_force(const Vector3& p_force, const Vector3& p_position, bool p_lock = true);

	void apply_central_force(const Vector3& p_force, bool p_lock = true);

	void apply_impulse(const Vector3& p_impulse, const Vector3& p_position, bool p_lock = true);

	void apply_central_impulse(const Vector3& p_impulse, bool p_lock = true);

	void apply_torque(const Vector3& p_torque, bool p_lock = true);

	void apply_torque_impulse(const Vector3& p_impulse, bool p_lock = true);

	void add_constant_central_force(const Vector3& p_force);

	void add_constant_force(const Vector3& p_force, const Vector3& p_position, bool p_lock = true);

	void add_constant_torque(const Vector3& p_torque);

	Vector3 get_constant_force() const;

	void set_constant_force(const Vector3& p_force);

	Vector3 get_constant_torque() const;

	void set_constant_torque(const Vector3& p_torque);

	void add_collision_exception(const RID& p_excepted_body, bool p_lock = true);

	void remove_collision_exception(const RID& p_excepted_body, bool p_lock = true);

	bool has_collision_exception(const RID& p_excepted_body, bool p_lock = true) const;

	TypedArray<RID> get_collision_exceptions(bool p_lock = true) const;

	void integrate_forces(bool p_lock = true);

	void call_queries() override;

	JoltPhysicsDirectBodyState3D* get_direct_state();

	PhysicsServer3D::BodyMode get_mode() const override { return mode; }

	void set_mode(PhysicsServer3D::BodyMode p_mode, bool p_lock = true);

	bool is_ccd_enabled() const override { return ccd_enabled; }

	void set_ccd_enabled(bool p_enable, bool p_lock = true);

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

	float get_linear_damp() const override { return linear_damp; }

	void set_linear_damp(float p_damp, bool p_lock = true);

	float get_angular_damp() const override { return angular_damp; }

	void set_angular_damp(float p_damp, bool p_lock = true);

	bool is_area() const override { return false; }

private:
	void shapes_changed(bool p_lock) override;

	void mass_properties_changed(bool p_lock);

	PhysicsServer3D::BodyMode mode = PhysicsServer3D::BODY_MODE_RIGID;

	bool ccd_enabled = false;

	float mass = 1.0f;

	Vector3 inertia;

	float bounce = 0.0f;

	float friction = 1.0f;

	float gravity_scale = 1.0f;

	float linear_damp = 0.0f;

	float angular_damp = 0.0f;

	bool initial_sleep_state = false;

	bool allowed_sleep = true;

	bool custom_center_of_mass = false;

	Vector3 initial_linear_velocity;

	Vector3 initial_angular_velocity;

	Vector3 center_of_mass_custom;

	Vector3 constant_force;

	Vector3 constant_torque;

	Callable body_state_callback;

	Callable force_integration_callback;

	Variant force_integration_userdata;

	JoltPhysicsDirectBodyState3D* direct_state = nullptr;
};
