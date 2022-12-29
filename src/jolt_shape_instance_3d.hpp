#pragma once

class JoltShape3D;

class JoltShapeInstance3D {
public:
	struct Built {
		const JoltShapeInstance3D* shape = nullptr;
		JPH::ShapeRefC jolt_ref;
	};

	JoltShapeInstance3D() = default;

	JoltShapeInstance3D(JoltShape3D* p_shape, const Transform3D& p_transform, bool p_disabled)
		: transform(p_transform)
		, shape(p_shape)
		, disabled(p_disabled) { }

	JoltShape3D* get() const { return shape; }

	const Transform3D& get_transform() const { return transform; }

	void set_transform(const Transform3D& p_transform) { transform = p_transform; }

	bool is_disabled() const { return disabled; }

	bool is_enabled() const { return !disabled; }

	void set_disabled(bool p_disabled) { disabled = p_disabled; }

	JoltShape3D* operator->() const { return shape; }

	JoltShape3D& operator*() const { return *shape; }

	explicit operator JoltShape3D*() const { return shape; }

	bool operator==(const JoltShapeInstance3D& p_other) { return shape == p_other.shape; }

	friend bool operator==(const JoltShapeInstance3D& p_lhs, JoltShape3D* p_rhs) {
		return p_lhs.shape == p_rhs;
	}

	friend bool operator==(JoltShape3D* p_lhs, const JoltShapeInstance3D& p_rhs) {
		return p_lhs == p_rhs.shape;
	}

	static bool try_build(
		const JoltShapeInstance3D& p_shape,
		uint64_t p_user_data,
		Built& p_built_shape
	);

	static void try_build(
		const Vector<JoltShapeInstance3D>& p_shapes,
		Vector<Built>& p_built_shapes
	);

	static JPH::ShapeRefC build_compound(const Vector<Built>& p_built_shapes);

private:
	Transform3D transform;

	JoltShape3D* shape = nullptr;

	bool disabled = false;
};
