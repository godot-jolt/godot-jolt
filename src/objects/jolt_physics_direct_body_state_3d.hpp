#pragma once

class JoltBodyImpl3D;

class JoltPhysicsDirectBodyState3D final : public PhysicsDirectBodyState3DExtension {
	GDCLASS_QUIET(JoltPhysicsDirectBodyState3D, PhysicsDirectBodyState3DExtension)

private:
	static void _bind_methods() { }

public:
	JoltPhysicsDirectBodyState3D() = default;

	explicit JoltPhysicsDirectBodyState3D(JoltBodyImpl3D* p_body);

	Vector3 _get_total_gravity() const override;

	double _get_total_linear_damp() const override;

	double _get_total_angular_damp() const override;

	Vector3 _get_center_of_mass() const override;

	Vector3 _get_center_of_mass_local() const override;

	Basis _get_principal_inertia_axes() const override;

	double _get_inverse_mass() const override;

	Vector3 _get_inverse_inertia() const override;

	Basis _get_inverse_inertia_tensor() const override;

	void _set_linear_velocity(const Vector3& p_velocity) override;

	Vector3 _get_linear_velocity() const override;

	void _set_angular_velocity(const Vector3& p_velocity) override;

	Vector3 _get_angular_velocity() const override;

	void _set_transform(const Transform3D& p_transform) override;

	Transform3D _get_transform() const override;

	Vector3 _get_velocity_at_local_position(const Vector3& p_local_position) const override;

	void _apply_central_impulse(const Vector3& p_impulse) override;

	void _apply_impulse(const Vector3& p_impulse, const Vector3& p_position) override;

	void _apply_torque_impulse(const Vector3& p_impulse) override;

	void _apply_central_force(const Vector3& p_force) override;

	void _apply_force(const Vector3& p_force, const Vector3& p_position) override;

	void _apply_torque(const Vector3& p_torque) override;

	void _add_constant_central_force(const Vector3& p_force) override;

	void _add_constant_force(const Vector3& p_force, const Vector3& p_position) override;

	void _add_constant_torque(const Vector3& p_torque) override;

	void _set_constant_force(const Vector3& p_force) override;

	Vector3 _get_constant_force() const override;

	void _set_constant_torque(const Vector3& p_torque) override;

	Vector3 _get_constant_torque() const override;

	void _set_sleep_state(bool p_enabled) override;

	bool _is_sleeping() const override;

	int32_t _get_contact_count() const override;

	Vector3 _get_contact_local_position(int32_t p_contact_idx) const override;

	Vector3 _get_contact_local_normal(int32_t p_contact_idx) const override;

	Vector3 _get_contact_impulse(int32_t p_contact_idx) const override;

	int32_t _get_contact_local_shape(int32_t p_contact_idx) const override;

	Vector3 _get_contact_local_velocity_at_position(int32_t p_contact_idx) const override;

	RID _get_contact_collider(int32_t p_contact_idx) const override;

	Vector3 _get_contact_collider_position(int32_t p_contact_idx) const override;

	uint64_t _get_contact_collider_id(int32_t p_contact_idx) const override;

	Object* _get_contact_collider_object(int32_t p_contact_idx) const override;

	int32_t _get_contact_collider_shape(int32_t p_contact_idx) const override;

	Vector3 _get_contact_collider_velocity_at_position(int32_t p_contact_idx) const override;

	double _get_step() const override;

	void _integrate_forces() override;

	PhysicsDirectSpaceState3D* _get_space_state() override;

private:
	JoltBodyImpl3D* body = nullptr;
};
