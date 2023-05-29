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

	float get_solver_bias() const;

	void set_solver_bias(float p_bias);

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

	String owners_to_string() const;

	HashMap<JoltObjectImpl3D*, int32_t> ref_counts_by_owner;

	RID rid;

	JPH::ShapeRefC jolt_ref;
};

#include "jolt_shape_impl_3d.inl"
