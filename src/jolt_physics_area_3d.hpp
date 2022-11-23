#pragma once

#include "jolt_physics_collision_object_3d.hpp"

class JoltPhysicsArea3D final : public JoltPhysicsCollisionObject3D {
public:
	bool has_monitor_callback() const { return !monitor_callback.is_null(); }

	void set_monitor_callback(const Callable& p_callback) { monitor_callback = p_callback; }

	bool has_area_monitor_callback() const { return !area_monitor_callback.is_null(); }

	void set_area_monitor_callback(const Callable& p_callback) {
		area_monitor_callback = p_callback;
	}

	bool is_monitorable() const { return monitorable; }

	void set_monitorable(bool p_monitorable) { monitorable = p_monitorable; }

	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;
	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value);

	void call_queries() override;

	PhysicsServer3D::BodyMode get_mode() const override {
		return PhysicsServer3D::BODY_MODE_KINEMATIC;
	}

	float get_mass() const override { return 1.0f; }

	Vector3 get_inertia() const override { return {0, 0, 0}; }

	bool is_sensor() const override { return true; }

private:
	Vector3 gravity_vector = Vector3(0, -1, 0);

	float gravity = 9.81f;

	float linear_damp = 0.1f;

	float angular_damp = 0.1f;

	bool monitorable = false;

	Callable monitor_callback;

	Callable area_monitor_callback;
};
