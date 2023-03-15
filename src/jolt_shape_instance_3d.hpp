#pragma once

class JoltCollisionObject3D;
class JoltShape3D;

class JoltShapeInstance3D {
public:
	JoltShapeInstance3D(
		JoltCollisionObject3D* p_parent,
		JoltShape3D* p_shape,
		const Transform3D& p_transform = {},
		bool p_disabled = false
	);

	JoltShapeInstance3D(const JoltShapeInstance3D& p_other) = delete;

	JoltShapeInstance3D(JoltShapeInstance3D&& p_other) noexcept;

	~JoltShapeInstance3D();

	uint32_t get_id() const { return id; }

	JoltShape3D* get_shape() const { return shape; }

	const JPH::Shape* get_jolt_ref() const { return jolt_ref; }

	const Transform3D& get_transform() const { return transform; }

	void set_transform(const Transform3D& p_transform) { transform = p_transform; }

	bool is_built() const { return jolt_ref != nullptr; }

	bool is_enabled() const { return !disabled; }

	bool is_disabled() const { return disabled; }

	void enable() { disabled = false; }

	void disable() { disabled = true; }

	bool try_build();

	JoltShapeInstance3D& operator=(const JoltShapeInstance3D& p_other) = delete;

	JoltShapeInstance3D& operator=(JoltShapeInstance3D&& p_other) noexcept;

private:
	inline static uint32_t next_id = 1;

	Transform3D transform;

	JPH::ShapeRefC jolt_ref;

	JoltCollisionObject3D* parent = nullptr;

	JoltShape3D* shape = nullptr;

	uint32_t id = next_id++;

	bool disabled = false;
};
