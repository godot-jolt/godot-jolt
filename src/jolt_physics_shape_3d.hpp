#pragma once

class JoltPhysicsCollisionObject3D;

class JoltPhysicsShape3D {
public:
	virtual ~JoltPhysicsShape3D() = 0;

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	virtual JoltPhysicsCollisionObject3D* get_owner() const { return owner; }

	virtual void set_owner(JoltPhysicsCollisionObject3D* p_owner) { owner = p_owner; }

	virtual Variant get_data() const = 0;

	virtual void set_data(const Variant& p_data) = 0;

	JPH::Shape* get_jref() const { return jref; }

protected:
	RID rid;

	JoltPhysicsCollisionObject3D* owner = nullptr;

	JPH::Ref<JPH::Shape> jref;
};

class JoltPhysicsSphereShape3D final : public JoltPhysicsShape3D {
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float radius = 0.0f;
};

class JoltPhysicsBoxShape3D final : public JoltPhysicsShape3D {
public:
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	Vector3 half_extents;
};

class JoltPhysicsCapsuleShape3D final : public JoltPhysicsShape3D {
public:
	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

private:
	float height = 0.0f;

	float radius = 0.0f;
};
