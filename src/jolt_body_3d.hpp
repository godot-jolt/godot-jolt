#pragma once

#include "jolt_collision_object_3d.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"

class JoltArea3D;

class JoltBody3D final : public JoltCollisionObject3D {
public:
	using DampMode = PhysicsServer3D::BodyDampMode;

	struct Contact {
		float depth = 0.0f;

		float impulse = 0.0f;

		int32_t shape_index = 0;

		int32_t collider_shape_index = 0;

		ObjectID collider_id;

		RID collider_rid;

		Vector3 normal;

		Vector3 position;

		Vector3 collider_position;

		Vector3 collider_velocity;
	};

	~JoltBody3D() override;

	Variant get_state(PhysicsServer3D::BodyState p_state);

	void set_state(PhysicsServer3D::BodyState p_state, const Variant& p_value);

	Variant get_param(PhysicsServer3D::BodyParameter p_param) const;

	void set_param(PhysicsServer3D::BodyParameter p_param, const Variant& p_value);

	JPH::BroadPhaseLayer get_broad_phase_layer() const override;

	bool has_state_sync_callback() const { return body_state_callback.is_valid(); }

	void set_state_sync_callback(const Callable& p_callback) { body_state_callback = p_callback; }

	bool has_force_integration_callback() const { return force_integration_callback.is_valid(); }

	void set_force_integration_callback(const Callable& p_callback, const Variant& p_userdata) {
		force_integration_callback = p_callback;
		force_integration_userdata = p_userdata;
	}

	bool get_sleep_state(bool p_lock = true) const;

	void set_sleep_state(bool p_enabled, bool p_lock = true);

	bool can_sleep() const { return allowed_sleep; }

	void set_can_sleep(bool p_enabled, bool p_lock = true);

	Basis get_inverse_inertia_tensor(bool p_lock = true) const;

	Vector3 get_linear_velocity(bool p_lock = true) const;

	void set_linear_velocity(const Vector3& p_velocity, bool p_lock = true);

	Vector3 get_angular_velocity(bool p_lock = true) const;

	void set_angular_velocity(const Vector3& p_velocity, bool p_lock = true);

	bool has_custom_center_of_mass() const override { return custom_center_of_mass; }

	Vector3 get_center_of_mass_custom() const override { return center_of_mass_custom; }

	void set_center_of_mass_custom(const Vector3& p_center_of_mass, bool p_lock = true);

	int32_t get_max_contacts_reported() const { return contacts.size(); }

	void set_max_contacts_reported(int32_t p_count) { contacts.resize(p_count); }

	int32_t get_contact_count() const { return contact_count; }

	const Contact& get_contact(int32_t p_index) { return contacts[p_index]; }

	bool generates_contacts() const override { return contacts.size() > 0; }

	void add_contact(
		const JoltBody3D* p_collider,
		float p_depth,
		int32_t p_shape_index,
		int32_t p_collider_shape_index,
		const Vector3& p_normal,
		const Vector3& p_position,
		const Vector3& p_collider_position,
		const Vector3& p_collider_velocity,
		float p_impulse
	);

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

	void add_area(JoltArea3D* p_area);

	void remove_area(JoltArea3D* p_area);

	void integrate_forces(float p_step, bool p_lock = true);

	void call_queries();

	void pre_step(float p_step) override;

	JoltPhysicsDirectBodyState3D* get_direct_state();

	PhysicsServer3D::BodyMode get_mode() const { return mode; }

	void set_mode(PhysicsServer3D::BodyMode p_mode, bool p_lock = true);

	bool is_static() const { return mode == PhysicsServer3D::BODY_MODE_STATIC; }

	bool is_kinematic() const { return mode == PhysicsServer3D::BODY_MODE_KINEMATIC; }

	bool is_rigid() const { return mode == PhysicsServer3D::BODY_MODE_RIGID; }

	bool is_ccd_enabled() const { return ccd_enabled; }

	void set_ccd_enabled(bool p_enable, bool p_lock = true);

	float get_mass() const { return mass; }

	void set_mass(float p_mass, bool p_lock = true);

	Vector3 get_inertia() const { return inertia; }

	void set_inertia(const Vector3& p_inertia, bool p_lock = true);

	float get_bounce() const { return bounce; }

	void set_bounce(float p_bounce, bool p_lock = true);

	float get_friction() const { return friction; }

	void set_friction(float p_friction, bool p_lock = true);

	float get_gravity_scale() const { return gravity_scale; }

	void set_gravity_scale(float p_scale) { gravity_scale = p_scale; }

	float get_linear_damp() const { return linear_damp; }

	void set_linear_damp(float p_damp, bool p_lock = true);

	float get_angular_damp() const { return angular_damp; }

	void set_angular_damp(float p_damp, bool p_lock = true);

	DampMode get_linear_damp_mode() const { return linear_damp_mode; }

	void set_linear_damp_mode(DampMode p_mode) { linear_damp_mode = p_mode; }

	DampMode get_angular_damp_mode() const { return angular_damp_mode; }

	void set_angular_damp_mode(DampMode p_mode) { angular_damp_mode = p_mode; }

private:
	bool get_initial_sleep_state() const override { return initial_sleep_state; }

	JPH::EMotionType get_motion_type() const override;

	void create_in_space(bool p_lock = true) override;

	void mode_changed(bool p_lock = true);

	void shapes_changed(bool p_lock) override;

	void areas_changed(bool p_lock = true);

	void damp_changed(bool p_lock = true);

	JPH::MassProperties calculate_mass_properties(const JPH::Shape& p_shape) const;

	JPH::MassProperties calculate_mass_properties() const;

	void mass_properties_changed(bool p_lock);

	PhysicsServer3D::BodyMode mode = PhysicsServer3D::BODY_MODE_RIGID;

	DampMode linear_damp_mode = PhysicsServer3D::BODY_DAMP_MODE_COMBINE;

	DampMode angular_damp_mode = PhysicsServer3D::BODY_DAMP_MODE_COMBINE;

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

	int32_t contact_count = 0;

	LocalVector<Contact> contacts;

	LocalVector<JoltArea3D*> areas;

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
