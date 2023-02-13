#pragma once

#include "jolt_collision_object_3d.hpp"

class JoltArea3D final : public JoltCollisionObject3D {
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

	bool has_custom_center_of_mass() const override { return false; }

	Vector3 get_center_of_mass_custom() const override { return {0, 0, 0}; }

	bool get_initial_sleep_state() const override { return false; }

private:
	JPH::EMotionType get_motion_type() const override { return JPH::EMotionType::Kinematic; }

	void create_in_space(bool p_lock = true) override;

	Vector3 gravity_vector = {0, -1, 0};

	float gravity = 9.81f;

	float linear_damp = 0.1f;

	float angular_damp = 0.1f;

	bool monitorable = false;

	Callable monitor_callback;

	Callable area_monitor_callback;
};
