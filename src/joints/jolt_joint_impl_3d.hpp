#pragma once

class JoltBodyImpl3D;
class JoltSpace3D;

class JoltJointImpl3D {
public:
	JoltJointImpl3D() = default;

	JoltJointImpl3D(
		JoltBodyImpl3D* p_body_a,
		JoltBodyImpl3D* p_body_b,
		const Transform3D& p_local_ref_a,
		const Transform3D& p_local_ref_b,
		bool p_lock = true
	);

	JoltJointImpl3D(JoltBodyImpl3D* p_body_a, const Transform3D& p_world_ref);

	virtual ~JoltJointImpl3D();

	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_TYPE_MAX; }

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	JoltSpace3D* get_space() const;

	JPH::Constraint* get_jolt_ref() const { return jolt_ref; }

	int32_t get_solver_priority() const;

	void set_solver_priority(int32_t p_priority);

	bool is_collision_disabled() const { return collision_disabled; }

	void set_collision_disabled(bool p_disabled);

	void destroy();

	virtual void rebuild([[maybe_unused]] bool p_lock = true) { }

protected:
	String owners_to_string() const;

	bool collision_disabled = false;

	JPH::Ref<JPH::Constraint> jolt_ref;

	JoltBodyImpl3D* body_a = nullptr;

	JoltBodyImpl3D* body_b = nullptr;

	RID rid;

	Transform3D world_ref;
};
