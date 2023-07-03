#pragma once

#include "shapes/jolt_shape_instance_3d.hpp"

class JoltShapeImpl3D;
class JoltSpace3D;

class JoltObjectImpl3D {
public:
	JoltObjectImpl3D();

	virtual ~JoltObjectImpl3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	ObjectID get_instance_id() const { return instance_id; }

	void set_instance_id(ObjectID p_id) { instance_id = p_id; }

	JPH::BodyID get_jolt_id() const { return jolt_id; }

	GodotObject* get_instance() const;

	Object* get_instance_unsafe() const;

	Object* get_instance_wrapped() const;

	JoltSpace3D* get_space() const { return space; }

	void set_space(JoltSpace3D* p_space, bool p_lock = true);

	uint32_t get_collision_layer() const { return collision_layer; }

	void set_collision_layer(uint32_t p_layer, bool p_lock = true);

	uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_mask(uint32_t p_mask, bool p_lock = true);

	Transform3D get_transform_unscaled(bool p_lock = true) const;

	Transform3D get_transform_scaled(bool p_lock = true) const;

	void set_transform(Transform3D p_transform, bool p_lock = true);

	Vector3 get_scale() const { return scale; }

	Basis get_basis(bool p_lock = true) const;

	Vector3 get_position(bool p_lock = true) const;

	Vector3 get_center_of_mass(bool p_lock = true) const;

	Vector3 get_center_of_mass_local(bool p_lock = true) const;

	Vector3 get_linear_velocity(bool p_lock = true) const;

	Vector3 get_angular_velocity(bool p_lock = true) const;

	Vector3 get_velocity_at_position(const Vector3& p_position, bool p_lock = true) const;

	virtual bool has_custom_center_of_mass() const = 0;

	virtual Vector3 get_center_of_mass_custom() const = 0;

	bool is_pickable() const { return pickable; }

	void set_pickable(bool p_enabled) { pickable = p_enabled; }

	virtual bool generates_contacts() const = 0;

	JPH::ShapeRefC try_build_shape();

	void build_shape(bool p_lock = true);

	const JPH::Shape* get_jolt_shape() const { return jolt_shape; }

	const JPH::Shape* get_previous_jolt_shape() const { return previous_jolt_shape; }

	void add_shape(
		JoltShapeImpl3D* p_shape,
		Transform3D p_transform,
		bool p_disabled,
		bool p_lock = true
	);

	void remove_shape(const JoltShapeImpl3D* p_shape, bool p_lock = true);

	void remove_shape(int32_t p_index, bool p_lock = true);

	JoltShapeImpl3D* get_shape(int32_t p_index) const;

	void set_shape(int32_t p_index, JoltShapeImpl3D* p_shape, bool p_lock = true);

	void clear_shapes(bool p_lock = true);

	int32_t get_shape_count() const { return shapes.size(); }

	int32_t find_shape_index(uint32_t p_shape_instance_id) const;

	int32_t find_shape_index(const JPH::SubShapeID& p_sub_shape_id) const;

	JoltShapeImpl3D* find_shape(uint32_t p_shape_instance_id) const;

	JoltShapeImpl3D* find_shape(const JPH::SubShapeID& p_sub_shape_id) const;

	Transform3D get_shape_transform_unscaled(int32_t p_index) const;

	Transform3D get_shape_transform_scaled(int32_t p_index) const;

	Vector3 get_shape_scale(int32_t p_index) const;

	void set_shape_transform(int32_t p_index, Transform3D p_transform, bool p_lock = true);

	bool is_shape_disabled(int32_t p_index) const;

	void set_shape_disabled(int32_t p_index, bool p_disabled, bool p_lock = true);

	virtual void pre_step(float p_step, JPH::Body& p_jolt_body);

	virtual void post_step(float p_step, JPH::Body& p_jolt_body);

	String to_string() const;

protected:
	friend class JoltShapeImpl3D;

	virtual JPH::BroadPhaseLayer get_broad_phase_layer() const = 0;

	JPH::ObjectLayer get_object_layer() const;

	virtual JPH::EMotionType get_motion_type() const = 0;

	virtual void create_in_space() = 0;

	virtual void add_to_space(bool p_lock = true);

	virtual void remove_from_space(bool p_lock = true);

	virtual void destroy_in_space(bool p_lock = true);

	virtual void apply_transform(const Transform3D& p_transform, bool p_lock = true);

	void create_begin();

	JPH::Body* create_end();

	void update_object_layer(bool p_lock = true);

	virtual void collision_layer_changed(bool p_lock = true);

	virtual void collision_mask_changed(bool p_lock = true);

	virtual void shapes_changed(bool p_lock = true);

	virtual void shapes_built([[maybe_unused]] bool p_lock = true) { }

	virtual void space_changing([[maybe_unused]] bool p_lock = true) { }

	virtual void space_changed([[maybe_unused]] bool p_lock = true) { }

	virtual void transform_changed([[maybe_unused]] bool p_lock = true) { }

	LocalVector<JoltShapeInstance3D> shapes;

	Vector3 scale = {1.0f, 1.0f, 1.0f};

	RID rid;

	ObjectID instance_id;

	JoltSpace3D* space = nullptr;

	JPH::BodyCreationSettings* jolt_settings = new JPH::BodyCreationSettings();

	JPH::ShapeRefC jolt_shape;

	JPH::ShapeRefC previous_jolt_shape;

	JPH::BodyID jolt_id;

	uint32_t collision_layer = 1;

	uint32_t collision_mask = 1;

	bool pickable = false;
};
