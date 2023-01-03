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
