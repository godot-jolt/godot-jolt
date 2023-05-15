#pragma once

class JoltBodyImpl3D;
class JoltSpace3D;

class JoltJointImpl3D {
public:
	JoltJointImpl3D() = default;

	JoltJointImpl3D(
		JoltSpace3D* p_space,
		JoltBodyImpl3D* p_body_a,
		JoltBodyImpl3D* p_body_b = nullptr
	);

	virtual ~JoltJointImpl3D();

	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_TYPE_MAX; }

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	JoltSpace3D* get_space() const { return space; }

	JPH::Constraint* get_jolt_ref() const { return jolt_ref; }

	int32_t get_solver_priority() const;

	void set_solver_priority(int32_t p_priority);

	bool is_collision_disabled() const { return collision_disabled; }

	void set_collision_disabled(bool p_disabled);

protected:
	RID rid;

	JoltSpace3D* space = nullptr;

	JoltBodyImpl3D* body_a = nullptr;

	JoltBodyImpl3D* body_b = nullptr;

	JPH::Ref<JPH::Constraint> jolt_ref;

	bool collision_disabled = false;
};
