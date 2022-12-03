#pragma once

class JoltPhysicsArea3D;
class JoltPhysicsBody3D;
class JoltPhysicsCollisionObject3D;

class JoltPhysicsSpace3D final {
public:
	JoltPhysicsSpace3D(JPH::JobSystem* p_job_system, JPH::GroupFilter* p_group_filter);

	~JoltPhysicsSpace3D();

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

	PhysicsDirectSpaceState3D* get_direct_state() const;

	void set_default_area(JoltPhysicsArea3D* p_area) { area = p_area; }

	JoltPhysicsArea3D* get_default_area() const { return area; }

	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value);

	void create_object(JoltPhysicsCollisionObject3D* p_object);

	void add_object(JoltPhysicsCollisionObject3D* p_object);

	void remove_object(JoltPhysicsCollisionObject3D* p_object);

	void destroy_object(JoltPhysicsCollisionObject3D* p_object);

private:
	void update_gravity();

	RID rid;

	JPH::JobSystem* job_system = nullptr;

	JPH::TempAllocator* temp_allocator = nullptr;

	JPH::BroadPhaseLayerInterface* layer_mapper = nullptr;

	JPH::PhysicsSystem* physics_system = nullptr;

	JPH::GroupFilter* group_filter = nullptr;

	PhysicsDirectSpaceState3D* direct_state = nullptr;

	SelfList<JoltPhysicsBody3D>::List state_query_list;

	SelfList<JoltPhysicsArea3D>::List monitor_query_list;

	JoltPhysicsArea3D* area = nullptr;

	Vector3 gravity_vector = Vector3(0, -1, 0);

	float gravity = 9.81f;

	bool locked = false;
};
