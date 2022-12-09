#pragma once

class JoltCollisionObject3D;

class JoltShape3D {
public:
	virtual ~JoltShape3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	virtual JoltCollisionObject3D* get_owner() const { return owner; }

	virtual void set_owner(JoltCollisionObject3D* p_owner) { owner = p_owner; }

	virtual Variant get_data() const = 0;

	virtual void set_data(const Variant& p_data) = 0;

	const JPH::ShapeRefC& get_jolt_ref() const { return jolt_ref; }

	bool is_valid() const { return jolt_ref != nullptr; }

protected:
	virtual void clear_data();

	RID rid;

	JoltCollisionObject3D* owner = nullptr;

	JPH::ShapeRefC jolt_ref;
};

class JoltSphereShape3D final : public JoltShape3D {
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	void clear_data() override;

	float radius = 0.0f;
};

class JoltBoxShape3D final : public JoltShape3D {
public:
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	void clear_data() override;

	Vector3 half_extents;
};

class JoltCapsuleShape3D final : public JoltShape3D {
public:
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	void clear_data() override;

	float height = 0.0f;

	float radius = 0.0f;
};

class JoltCylinderShape3D final : public JoltShape3D {
public:
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	void clear_data() override;

	float height = 0.0f;

	float radius = 0.0f;
};

class JoltConvexPolygonShape3D final : public JoltShape3D {
public:
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	void clear_data() override;

	PackedVector3Array vertices;
};
