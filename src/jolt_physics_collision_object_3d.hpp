#pragma once

class JoltPhysicsSpace3D;
class JoltPhysicsShape3D;

class JoltPhysicsCollisionObject3D {
public:
	struct Shape {
		JoltPhysicsShape3D* ref = nullptr;
		bool disabled = false;
		Transform3D transform;
	};

	virtual ~JoltPhysicsCollisionObject3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	int64_t get_instance_id() const { return instance_id; }

	void set_instance_id(int64_t p_id) { instance_id = p_id; }

	JPH::BodyID get_jid() const { return jid; }

	void set_jid(JPH::BodyID p_jid) { jid = p_jid; }

	JoltPhysicsSpace3D* get_space() const { return space; }

	void set_space(JoltPhysicsSpace3D* p_space);

	uint32_t get_collision_layer() const { return collision_layer; }

	void set_collision_layer(uint32_t p_layer, bool p_lock = true);

	uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_mask(uint32_t p_mask, bool p_lock = true);

	Transform3D get_initial_transform() const { return initial_transform; }

	Transform3D get_transform(bool p_lock = true) const;

	void set_transform(const Transform3D& p_transform, bool p_lock = true);

	Vector3 get_center_of_mass(bool p_lock = true) const;

	JPH::MassProperties calculate_mass_properties(const JPH::Shape& p_root_shape) const;

	JPH::MassProperties calculate_mass_properties(bool p_lock = true) const;

	void add_shape(
		JoltPhysicsShape3D* p_shape,
		const Transform3D& p_transform,
		bool p_disabled,
		bool p_lock = true
	);

	void remove_shape(JoltPhysicsShape3D* p_shape, bool p_lock = true);

	void remove_shape(int p_index, bool p_lock = true);

	const Vector<Shape>& get_shapes() const { return shapes; }

	int get_shape_count() const { return shapes.size(); }

	int find_shape_index(JoltPhysicsShape3D* p_shape);

	void set_shape_transform(int64_t p_index, const Transform3D& p_transform, bool p_lock = true);

	bool is_ray_pickable() const { return ray_pickable; }

	void set_ray_pickable(bool p_enable) { ray_pickable = p_enable; }

	virtual void call_queries() = 0;

	virtual PhysicsServer3D::BodyMode get_mode() const = 0;

	virtual float get_mass() const = 0;

	virtual Vector3 get_inertia() const = 0;

	virtual float get_bounce() const = 0;

	virtual float get_friction() const = 0;

	virtual float get_gravity_scale() const = 0;

	virtual float get_linear_damp() const = 0;

	virtual float get_angular_damp() const = 0;

	virtual bool is_sensor() const = 0;

	virtual bool can_sleep() const = 0;

protected:
	virtual void shapes_changed([[maybe_unused]] bool p_lock) { }

	JPH::MutableCompoundShape* get_root_shape() const;

	RID rid;

	int64_t instance_id = 0LL;

	JPH::BodyID jid;

	JoltPhysicsSpace3D* space = nullptr;

	uint32_t collision_layer = 1;

	uint32_t collision_mask = 1;

	Vector<Shape> shapes;

	bool ray_pickable = false;

	Transform3D initial_transform;
};
