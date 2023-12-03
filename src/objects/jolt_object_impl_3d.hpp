#pragma once

#include "shapes/jolt_shape_instance_3d.hpp"

class JoltAreaImpl3D;
class JoltBodyImpl3D;
class JoltShapeImpl3D;
class JoltSpace3D;

class JoltObjectImpl3D {
public:
	enum ObjectType : int8_t {
		OBJECT_TYPE_INVALID,
		OBJECT_TYPE_BODY,
		OBJECT_TYPE_AREA
	};

	explicit JoltObjectImpl3D(ObjectType p_object_type);

	virtual ~JoltObjectImpl3D() = 0;

	bool is_body() const { return object_type == OBJECT_TYPE_BODY; };

	bool is_area() const { return object_type == OBJECT_TYPE_AREA; };

	JoltBodyImpl3D* as_body() {
		return is_body() ? reinterpret_cast<JoltBodyImpl3D*>(this) : nullptr;
	}

	const JoltBodyImpl3D* as_body() const {
		return is_body() ? reinterpret_cast<const JoltBodyImpl3D*>(this) : nullptr;
	}

	JoltAreaImpl3D* as_area() {
		return is_area() ? reinterpret_cast<JoltAreaImpl3D*>(this) : nullptr;
	}

	const JoltAreaImpl3D* as_area() const {
		return is_area() ? reinterpret_cast<const JoltAreaImpl3D*>(this) : nullptr;
	}

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	ObjectID get_instance_id() const { return instance_id; }

	void set_instance_id(ObjectID p_id) { instance_id = p_id; }

	JPH::BodyID get_jolt_id() const { return jolt_id; }

	GodotObject* get_instance() const;

	Object* get_instance_unsafe() const;

	Object* get_instance_wrapped() const;

	JoltSpace3D* get_space() const { return space; }

	void set_space(JoltSpace3D* p_space);

	uint32_t get_collision_layer() const { return collision_layer; }

	void set_collision_layer(uint32_t p_layer);

	uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_mask(uint32_t p_mask);

	Transform3D get_transform_unscaled() const;

	Transform3D get_transform_scaled() const;

	void set_transform(Transform3D p_transform);

	Vector3 get_scale() const { return scale; }

	Basis get_basis() const;

	Vector3 get_position() const;

	Vector3 get_center_of_mass() const;

	Vector3 get_center_of_mass_local() const;

	Vector3 get_linear_velocity() const;

	Vector3 get_angular_velocity() const;

	virtual Vector3 get_velocity_at_position(const Vector3& p_position) const = 0;

	virtual bool has_custom_center_of_mass() const = 0;

	virtual Vector3 get_center_of_mass_custom() const = 0;

	bool is_pickable() const { return pickable; }

	void set_pickable(bool p_enabled) { pickable = p_enabled; }

	virtual bool reports_contacts() const = 0;

	JPH::ShapeRefC try_build_shape();

	void build_shape();

	const JPH::Shape* get_jolt_shape() const { return jolt_shape; }

	const JPH::Shape* get_previous_jolt_shape() const { return previous_jolt_shape; }

	void add_shape(JoltShapeImpl3D* p_shape, Transform3D p_transform, bool p_disabled);

	void remove_shape(const JoltShapeImpl3D* p_shape);

	void remove_shape(int32_t p_index);

	JoltShapeImpl3D* get_shape(int32_t p_index) const;

	void set_shape(int32_t p_index, JoltShapeImpl3D* p_shape);

	void clear_shapes();

	int32_t get_shape_count() const { return shapes.size(); }

	int32_t find_shape_index(uint32_t p_shape_instance_id) const;

	int32_t find_shape_index(const JPH::SubShapeID& p_sub_shape_id) const;

	JoltShapeImpl3D* find_shape(uint32_t p_shape_instance_id) const;

	JoltShapeImpl3D* find_shape(const JPH::SubShapeID& p_sub_shape_id) const;

	Transform3D get_shape_transform_unscaled(int32_t p_index) const;

	Transform3D get_shape_transform_scaled(int32_t p_index) const;

	Vector3 get_shape_scale(int32_t p_index) const;

	void set_shape_transform(int32_t p_index, Transform3D p_transform);

	bool is_shape_disabled(int32_t p_index) const;

	void set_shape_disabled(int32_t p_index, bool p_disabled);

	virtual void pre_step(float p_step, JPH::Body& p_jolt_body);

	virtual void post_step(float p_step, JPH::Body& p_jolt_body);

	String to_string() const;

protected:
	friend class JoltShapeImpl3D;

	virtual JPH::BroadPhaseLayer _get_broad_phase_layer() const = 0;

	JPH::ObjectLayer _get_object_layer() const;

	virtual JPH::EMotionType _get_motion_type() const = 0;

	virtual void _create_in_space() = 0;

	virtual void _add_to_space();

	virtual void _remove_from_space();

	virtual void _destroy_in_space();

	virtual void _apply_transform(const Transform3D& p_transform);

	void _create_begin();

	JPH::Body* _create_end();

	void _update_object_layer();

	virtual void _collision_layer_changed();

	virtual void _collision_mask_changed();

	virtual void _shapes_changed();

	virtual void _shapes_built() { }

	virtual void _space_changing() { }

	virtual void _space_changed() { }

	virtual void _transform_changed() { }

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

	ObjectType object_type = OBJECT_TYPE_INVALID;

	bool pickable = false;
};
