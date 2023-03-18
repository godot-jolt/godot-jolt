#pragma once

class JoltCollisionObject3D;

class JoltShape3D {
public:
	using ShapeType = PhysicsServer3D::ShapeType;

	virtual ~JoltShape3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	void add_owner(JoltCollisionObject3D* p_owner);

	void remove_owner(JoltCollisionObject3D* p_owner);

	void remove_self(bool p_lock = true);

	virtual ShapeType get_type() const = 0;

	virtual Variant get_data() const = 0;

	virtual void set_data(const Variant& p_data) = 0;

	virtual bool is_valid() const = 0;

	JPH::ShapeRefC try_build();

	const JPH::Shape* get_jolt_ref() const { return jolt_ref; }

	Vector3 get_center_of_mass() const;

	static JPH::ShapeRefC with_scale(const JPH::Shape* p_shape, const Vector3& p_scale);

	static JPH::ShapeRefC with_basis_origin(
		const JPH::Shape* p_shape,
		const Basis& p_basis,
		const Vector3& p_origin
	);

	static JPH::ShapeRefC with_transform(const JPH::Shape* p_shape, const Transform3D& p_transform);

	static JPH::ShapeRefC with_center_of_mass_offset(
		const JPH::Shape* p_shape,
		const Vector3& p_offset
	);

	static JPH::ShapeRefC with_center_of_mass(
		const JPH::Shape* p_shape,
		const Vector3& p_center_of_mass
	);

	static JPH::ShapeRefC with_user_data(const JPH::Shape* p_shape, uint64_t p_user_data);

	template<typename TCallable>
	static JPH::ShapeRefC as_compound(TCallable&& p_callable);

protected:
	virtual JPH::ShapeRefC build() const = 0;

	RID rid;

	JPH::ShapeRefC jolt_ref;

	HashMap<JoltCollisionObject3D*, int32_t> ref_counts_by_owner;
};

class JoltWorldBoundaryShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_WORLD_BOUNDARY; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(Plane p_plane);

	bool is_valid() const override { return plane != Plane(); }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	Plane plane;
};

class JoltSeparationRayShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_SEPARATION_RAY; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(float p_length, bool p_slide_on_slope);

	bool is_valid() const override { return length > 0.0f; }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	float length = 0.0f;

	bool slide_on_slope = false;
};

class JoltSphereShape3D final : public JoltShape3D {
	ShapeType get_type() const override { return ShapeType::SHAPE_SPHERE; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(float p_radius);

	bool is_valid() const override { return radius > 0; }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	float radius = 0.0f;
};

class JoltBoxShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_BOX; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(Vector3 p_half_extents);

	bool is_valid() const override { return half_extents.x > 0; }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	Vector3 half_extents;
};

class JoltCapsuleShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CAPSULE; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(float p_height, float p_radius);

	bool is_valid() const override { return radius > 0; }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	float height = 0.0f;

	float radius = 0.0f;
};

class JoltCylinderShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CYLINDER; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(float p_height, float p_radius);

	bool is_valid() const override { return radius > 0; }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	float height = 0.0f;

	float radius = 0.0f;
};

class JoltConvexPolygonShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CONVEX_POLYGON; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(PackedVector3Array p_vertices);

	bool is_valid() const override { return !vertices.is_empty(); }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	PackedVector3Array vertices;
};

class JoltConcavePolygonShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CONCAVE_POLYGON; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(PackedVector3Array p_faces, bool p_backface_collision);

	bool is_valid() const override { return !faces.is_empty(); }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	PackedVector3Array faces;

	bool backface_collision = false;
};

class JoltHeightMapShape3D final : public JoltShape3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_HEIGHTMAP; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	void set_data(PackedFloat32Array p_heights, int32_t p_width, int32_t p_depth);

	bool is_valid() const override { return width > 0; }

private:
	void clear();

	JPH::ShapeRefC build() const override;

	PackedFloat32Array heights;

	int32_t width = 0;

	int32_t depth = 0;
};

#include "jolt_shape_3d.inl"
