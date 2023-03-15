#pragma once

#include "jolt_shape_instance_3d.hpp"

class JoltSpace3D;
class JoltShape3D;

class JoltCollisionObject3D {
public:
	virtual ~JoltCollisionObject3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	ObjectID get_instance_id() const { return instance_id; }

	void set_instance_id(ObjectID p_id) { instance_id = p_id; }

	JPH::BodyID get_jolt_id() const { return jolt_id; }

	void set_jolt_id(JPH::BodyID p_jolt_id) { jolt_id = p_jolt_id; }

	void* get_instance() const;

	Object* get_instance_unsafe() const;

	Object* get_instance_wrapped() const;

	JoltSpace3D* get_space() const { return space; }

	void set_space(JoltSpace3D* p_space, bool p_lock = true);

	uint32_t get_collision_layer() const { return collision_layer; }

	void set_collision_layer(uint32_t p_layer, bool p_lock = true);

	uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_mask(uint32_t p_mask, bool p_lock = true);

	Transform3D get_transform(bool p_lock = true) const;

	void set_transform(const Transform3D& p_transform, bool p_lock = true);

	Basis get_basis(bool p_lock = true) const;

	void set_basis(const Basis& p_basis, bool p_lock = true);

	Vector3 get_position(bool p_lock = true) const;

	void set_position(const Vector3& p_position, bool p_lock = true);

	Vector3 get_center_of_mass(bool p_lock = true) const;

	Vector3 get_linear_velocity(bool p_lock = true) const;

	Vector3 get_angular_velocity(bool p_lock = true) const;

	Vector3 get_velocity_at_position(const Vector3& p_position, bool p_lock = true) const;

	JPH::ShapeRefC try_build_shape();

	void rebuild_shape(bool p_lock = true);

	const JPH::Shape* get_jolt_shape() const { return jolt_shape; }

	const JPH::Shape* get_previous_jolt_shape() const { return previous_jolt_shape; }

	void add_shape(
		JoltShape3D* p_shape,
		const Transform3D& p_transform,
		bool p_disabled,
		bool p_lock = true
	);

	void remove_shape(JoltShape3D* p_shape, bool p_lock = true);

	void remove_shape(int32_t p_index, bool p_lock = true);

	void clear_shapes(bool p_lock = true);

	int32_t get_shape_count() const { return shapes.size(); }

	int32_t find_shape_index(uint32_t p_shape_instance_id) const;

	int32_t find_shape_index(const JPH::SubShapeID& p_sub_shape_id) const;

	JoltShape3D* find_shape(uint32_t p_shape_instance_id) const;

	JoltShape3D* find_shape(const JPH::SubShapeID& p_sub_shape_id) const;

	JoltShape3D* get_shape(int32_t p_index) const;

	void set_shape(int32_t p_index, JoltShape3D* p_shape, bool p_lock = true);

	Transform3D get_shape_transform(int32_t p_index) const;

	void set_shape_transform(int32_t p_index, const Transform3D& p_transform, bool p_lock = true);

	void set_shape_disabled(int32_t p_index, bool p_disabled, bool p_lock = true);

	bool is_ray_pickable() const { return ray_pickable; }

	void set_ray_pickable(bool p_enable) { ray_pickable = p_enable; }

	virtual void pre_step(float p_step);

	virtual void post_step(float p_step);

	virtual bool generates_contacts() const = 0;

protected:
	virtual JPH::BroadPhaseLayer get_broad_phase_layer() const = 0;

	JPH::ObjectLayer get_object_layer() const;

	virtual bool has_custom_center_of_mass() const = 0;

	virtual Vector3 get_center_of_mass_custom() const = 0;

	virtual bool get_initial_sleep_state() const = 0;

	virtual JPH::EMotionType get_motion_type() const = 0;

	virtual void create_in_space(bool p_lock = true) = 0;

	JPH::BodyCreationSettings create_begin();

	JPH::Body* create_end(const JPH::BodyCreationSettings& p_settings, bool p_lock = true);

	virtual void destroy_in_space(bool p_lock = true);

	void add_to_space(bool p_lock = true);

	void remove_from_space(bool p_lock = true);

	void object_layer_changed(bool p_lock = true);

	void collision_layer_changed(bool p_lock = true);

	void collision_mask_changed(bool p_lock = true);

	virtual void shapes_changed([[maybe_unused]] bool p_lock) { }

	RID rid;

	ObjectID instance_id;

	JPH::BodyID jolt_id;

	JPH::ShapeRefC jolt_shape;

	JPH::ShapeRefC previous_jolt_shape;

	JoltSpace3D* space = nullptr;

	uint32_t collision_layer = 1;

	uint32_t collision_mask = 1;

	LocalVector<JoltShapeInstance3D> shapes;

	bool ray_pickable = false;

	Transform3D initial_transform;
};
