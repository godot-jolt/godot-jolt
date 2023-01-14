#pragma once

#include "jolt_body_accessor_3d.hpp"

class JoltArea3D;
class JoltBody3D;
class JoltCollisionObject3D;
class JoltJoint3D;
class JoltLayerMapper;
class JoltPhysicsDirectSpaceState3D;

class JoltSpace3D final {
public:
	JoltSpace3D(JPH::JobSystem* p_job_system, JPH::GroupFilter* p_group_filter);

	~JoltSpace3D();

	void step(float p_step);

	void call_queries();

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	JPH::PhysicsSystem* get_physics_system() const { return physics_system; }

	JPH::BodyInterface& get_body_iface(bool p_locked = true);

	const JPH::BodyInterface& get_body_iface(bool p_locked = true) const;

	const JPH::BodyLockInterface& get_body_lock_iface(bool p_locked = true) const;

	const JPH::NarrowPhaseQuery& get_narrow_phase_query(bool p_locked = true) const;

	JoltReadableBody3D read_body(const JPH::BodyID& p_body_id, bool p_lock = true) const;

	JoltReadableBody3D read_body(const JoltBody3D& p_body, bool p_lock = true) const;

	JoltWritableBody3D write_body(const JPH::BodyID& p_body_id, bool p_lock = true) const;

	JoltWritableBody3D write_body(const JoltBody3D& p_body, bool p_lock = true) const;

	JoltReadableBodies3D read_bodies(
		const JPH::BodyID* p_body_ids,
		int p_body_count,
		bool p_lock = true
	) const;

	JoltWritableBodies3D write_bodies(
		const JPH::BodyID* p_body_ids,
		int p_body_count,
		bool p_lock = true
	) const;

	JoltPhysicsDirectSpaceState3D* get_direct_state();

	void set_default_area(JoltArea3D* p_area) { area = p_area; }

	JoltArea3D* get_default_area() const { return area; }

	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value);

	void create_object(JoltCollisionObject3D* p_object, bool p_lock = true);

	void add_object(JoltCollisionObject3D* p_object, bool p_lock = true);

	void remove_object(JoltCollisionObject3D* p_object, bool p_lock = true);

	void destroy_object(JoltCollisionObject3D* p_object, bool p_lock = true);

	void add_joint(JoltJoint3D* p_joint);

	void remove_joint(JoltJoint3D* p_joint);

private:
	void integrate_forces(bool p_lock = true);

	void update_gravity();

	RID rid;

	JPH::JobSystem* job_system = nullptr;

	JPH::TempAllocator* temp_allocator = nullptr;

	JoltLayerMapper* layer_mapper = nullptr;

	JPH::PhysicsSystem* physics_system = nullptr;

	JPH::GroupFilter* group_filter = nullptr;

	JoltBodyWriter3D body_accessor;

	JoltPhysicsDirectSpaceState3D* direct_state = nullptr;

	JoltArea3D* area = nullptr;

	Vector3 gravity_vector = Vector3(0, -1, 0);

	float gravity = 9.81f;
};
