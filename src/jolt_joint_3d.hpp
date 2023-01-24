#pragma once

class JoltBody3D;
class JoltSpace3D;

class JoltJoint3D {
public:
	JoltJoint3D() = default;

	JoltJoint3D(JoltSpace3D* p_space, JoltBody3D* p_body_a, JoltBody3D* p_body_b = nullptr);

	virtual ~JoltJoint3D();

	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_TYPE_MAX; }

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	JoltSpace3D* get_space() const { return space; }

	JPH::Constraint* get_jolt_ref() const { return jolt_ref; }

	int64_t get_solver_priority() const;

	void set_solver_priority(int64_t p_priority);

	bool is_collision_disabled() const { return collision_disabled; }

	void set_collision_disabled(bool p_disabled);

protected:
	RID rid;

	JoltSpace3D* space = nullptr;

	JoltBody3D* body_a = nullptr;

	JoltBody3D* body_b = nullptr;

	JPH::Ref<JPH::Constraint> jolt_ref;

	bool collision_disabled = false;
};
