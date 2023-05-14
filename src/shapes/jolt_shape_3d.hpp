#pragma once

class JoltObjectImpl3D;

class JoltShapeImpl3D {
public:
	using ShapeType = PhysicsServer3D::ShapeType;

	virtual ~JoltShapeImpl3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	void add_owner(JoltObjectImpl3D* p_owner);

	void remove_owner(JoltObjectImpl3D* p_owner);

	void remove_self(bool p_lock = true);

	virtual ShapeType get_type() const = 0;

	virtual bool is_convex() const = 0;

	virtual Variant get_data() const = 0;

	virtual void set_data(const Variant& p_data) = 0;

	virtual float get_margin() const = 0;

	virtual void set_margin(float p_margin) = 0;

	JPH::ShapeRefC try_build();

	void destroy() { jolt_ref = nullptr; }

	const JPH::Shape* get_jolt_ref() const { return jolt_ref; }

	static JPH::ShapeRefC with_scale(const JPH::Shape* p_shape, const Vector3& p_scale);

	static JPH::ShapeRefC with_basis_origin(
		const JPH::Shape* p_shape,
		const Basis& p_basis,
		const Vector3& p_origin
	);

	static JPH::ShapeRefC with_transform(
		const JPH::Shape* p_shape,
		const Transform3D& p_transform,
		const Vector3& p_scale
	);

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

	virtual void invalidated(bool p_lock = true);

	HashMap<JoltObjectImpl3D*, int32_t> ref_counts_by_owner;

	RID rid;

	JPH::ShapeRefC jolt_ref;
};

class JoltWorldBoundaryShapeImpl3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_WORLD_BOUNDARY; }

	bool is_convex() const override { return false; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

private:
	JPH::ShapeRefC build() const override;

	Plane plane;
};

class JoltSeparationRayShapeImpl3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_SEPARATION_RAY; }

	bool is_convex() const override { return true; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	float length = 0.0f;

	bool slide_on_slope = false;
};

class JoltSphereShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_SPHERE; }

	bool is_convex() const override { return true; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	float radius = 0.0f;
};

class JoltBoxShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_BOX; }

	bool is_convex() const override { return true; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return margin; }

	void set_margin(float p_margin) override;

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	Vector3 half_extents;

	float margin = 0.04f;
};

class JoltCapsuleShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CAPSULE; }

	bool is_convex() const override { return true; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	float height = 0.0f;

	float radius = 0.0f;
};

class JoltCylinderShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CYLINDER; }

	bool is_convex() const override { return true; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return margin; }

	void set_margin(float p_margin) override;

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	float height = 0.0f;

	float radius = 0.0f;

	float margin = 0.04f;
};

class JoltConvexPolygonShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CONVEX_POLYGON; }

	bool is_convex() const override { return true; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return margin; }

	void set_margin(float p_margin) override;

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	PackedVector3Array vertices;

	float margin = 0.04f;
};

class JoltConcavePolygonShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CONCAVE_POLYGON; }

	bool is_convex() const override { return false; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	PackedVector3Array faces;

	bool backface_collision = false;
};

class JoltHeightMapShape3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_HEIGHTMAP; }

	bool is_convex() const override { return false; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC build() const override;

	PackedFloat32Array heights;

	int32_t width = 0;

	int32_t depth = 0;
};

#include "jolt_shape_3d.inl"
