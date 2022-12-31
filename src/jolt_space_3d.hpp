#pragma once

class JoltArea3D;
class JoltBody3D;
class JoltCollisionObject3D;
class JoltJoint3D;
class JoltPhysicsDirectSpaceState3D;

class JoltSpace3D final {
public:
	JoltSpace3D(JPH::JobSystem* p_job_system, JPH::GroupFilter* p_group_filter);

	~JoltSpace3D();

	void step(float p_step);

	void call_queries();

	bool is_locked() const { return locked; }

	void lock() { locked = true; }

	void unlock() { locked = false; }

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	JPH::BodyInterface& get_body_iface(bool p_locked = true);

	const JPH::BodyInterface& get_body_iface(bool p_locked = true) const;

	const JPH::BodyLockInterface& get_body_lock_iface(bool p_locked = true) const;

	const JPH::NarrowPhaseQuery& get_narrow_phase_query(bool p_locked = true) const;

	JoltPhysicsDirectSpaceState3D* get_direct_state();

	void set_default_area(JoltArea3D* p_area) { area = p_area; }

	JoltArea3D* get_default_area() const { return area; }

	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value);

	void create_object(JoltCollisionObject3D* p_object);

	void add_object(JoltCollisionObject3D* p_object);

	void remove_object(JoltCollisionObject3D* p_object);

	void destroy_object(JoltCollisionObject3D* p_object);

	void add_joint(JoltJoint3D* p_joint);

	void remove_joint(JoltJoint3D* p_joint);

private:
	void update_gravity();

	RID rid;

	JPH::JobSystem* job_system = nullptr;

	JPH::TempAllocator* temp_allocator = nullptr;

	JPH::BroadPhaseLayerInterface* layer_mapper = nullptr;

	JPH::PhysicsSystem* physics_system = nullptr;

	JPH::GroupFilter* group_filter = nullptr;

	JoltPhysicsDirectSpaceState3D* direct_state = nullptr;

	JoltArea3D* area = nullptr;

	Vector3 gravity_vector = Vector3(0, -1, 0);

	float gravity = 9.81f;

	bool locked = false;
};
