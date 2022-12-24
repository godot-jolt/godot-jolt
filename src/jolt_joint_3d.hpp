#pragma once

class JoltBody3D;
class JoltSpace3D;

class JoltJoint3D {
public:
	JoltJoint3D() = default;

	explicit JoltJoint3D(JoltSpace3D* p_space);

	virtual ~JoltJoint3D();

	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_TYPE_MAX; }

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	JoltSpace3D* get_space() const { return space; }

	JPH::Constraint* get_jolt_ref() const { return jolt_ref; }

	int64_t get_solver_priority() const;

	void set_solver_priority(int64_t p_priority);

protected:
	RID rid;

	JoltSpace3D* space = nullptr;

	JPH::Ref<JPH::Constraint> jolt_ref;
};

class JoltPinJoint3D final : public JoltJoint3D {
public:
	JoltPinJoint3D(
		JoltSpace3D* p_space,
		const JoltBody3D* p_body_a,
		const JoltBody3D* p_body_b,
		const Vector3& p_local_a,
		const Vector3& p_local_b,
		bool p_lock = true
	);

	PhysicsServer3D::JointType get_type() const override { return PhysicsServer3D::JOINT_TYPE_PIN; }

	double get_param(PhysicsServer3D::PinJointParam p_param);

	void set_param(PhysicsServer3D::PinJointParam p_param, double p_value);

	const JoltBody3D* get_body_a() const { return body_a; }

	const JoltBody3D* get_body_b() const { return body_b; }

	Vector3 get_local_a() const { return local_a; }

	Vector3 get_local_b() const { return local_b; }

	JoltPinJoint3D* with_local_a(const Vector3& p_local_a, bool p_lock = true) const {
		return memnew(JoltPinJoint3D(space, body_a, body_b, p_local_a, local_b, p_lock));
	}

	JoltPinJoint3D* with_local_b(const Vector3& p_local_b, bool p_lock = true) const {
		return memnew(JoltPinJoint3D(space, body_a, body_b, local_a, p_local_b, p_lock));
	}

private:
	const JoltBody3D* body_a = nullptr;

	const JoltBody3D* body_b = nullptr;

	Vector3 local_a;

	Vector3 local_b;
};
