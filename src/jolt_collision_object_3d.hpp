#pragma once

#include "jolt_shape_instance_3d.hpp"

class JoltSpace3D;
class JoltShape3D;

class JoltCollisionObject3D {
public:
	virtual ~JoltCollisionObject3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	int64_t get_instance_id() const { return instance_id; }

	void set_instance_id(int64_t p_id) { instance_id = p_id; }

	JPH::BodyID get_jolt_id() const { return jolt_id; }

	void set_jolt_id(JPH::BodyID p_jolt_id) { jolt_id = p_jolt_id; }

	JoltSpace3D* get_space() const { return space; }

	void set_space(JoltSpace3D* p_space);

	uint32_t get_collision_layer() const { return collision_layer; }

	void set_collision_layer(uint32_t p_layer, bool p_lock = true);

	uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_mask(uint32_t p_mask, bool p_lock = true);

	Transform3D get_initial_transform() const { return initial_transform; }

	Transform3D get_transform(bool p_lock = true) const;

	void set_transform(const Transform3D& p_transform, bool p_lock = true);

	Basis get_basis(bool p_lock = true) const;

	void set_basis(const Basis& p_basis, bool p_lock = true);

	Vector3 get_position(bool p_lock = true) const;

	void set_position(const Vector3& p_position, bool p_lock = true);

	Vector3 get_center_of_mass(bool p_lock = true) const;

	JPH::MassProperties calculate_mass_properties(const JPH::Shape& p_shape) const;

	JPH::MassProperties calculate_mass_properties(bool p_lock = true) const;

	JPH::ShapeRefC try_build_shape() const;

	void rebuild_shape(bool p_lock = true);

	void add_shape(
		JoltShape3D* p_shape,
		const Transform3D& p_transform,
		bool p_disabled,
		bool p_lock = true
	);

	void remove_shape(JoltShape3D* p_shape, bool p_lock = true);

	void remove_shape(int32_t p_index, bool p_lock = true);

	void remove_shapes(bool p_lock = true);

	const Vector<JoltShapeInstance3D>& get_shapes() const { return shapes; }

	int32_t get_shape_count() const { return shapes.size(); }

	int32_t find_shape_index(JoltShape3D* p_shape);

	void set_shape_transform(int64_t p_index, const Transform3D& p_transform, bool p_lock = true);

	void set_shape_disabled(int64_t p_index, bool p_disabled, bool p_lock = true);

	bool is_ray_pickable() const { return ray_pickable; }

	void set_ray_pickable(bool p_enable) { ray_pickable = p_enable; }

	virtual void call_queries() = 0;

	virtual Vector3 get_initial_linear_velocity() const = 0;

	virtual Vector3 get_initial_angular_velocity() const = 0;

	virtual bool has_custom_center_of_mass() const = 0;

	virtual Vector3 get_center_of_mass_custom() const = 0;

	virtual bool get_initial_sleep_state() const = 0;

	virtual PhysicsServer3D::BodyMode get_mode() const = 0;

	virtual bool is_ccd_enabled() const = 0;

	virtual float get_mass() const = 0;

	virtual Vector3 get_inertia() const = 0;

	virtual float get_bounce() const = 0;

	virtual float get_friction() const = 0;

	virtual float get_gravity_scale() const = 0;

	virtual float get_linear_damp() const = 0;

	virtual float get_angular_damp() const = 0;

	virtual bool is_area() const = 0;

	virtual bool can_sleep() const = 0;

protected:
	virtual void shapes_changed([[maybe_unused]] bool p_lock) { }

	RID rid;

	int64_t instance_id = 0LL;

	JPH::BodyID jolt_id;

	JoltSpace3D* space = nullptr;

	uint32_t collision_layer = 1;

	uint32_t collision_mask = 1;

	Vector<JoltShapeInstance3D> shapes;

	bool ray_pickable = false;

	Transform3D initial_transform;
};
