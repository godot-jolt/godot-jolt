#pragma once

#include "spaces/jolt_body_accessor_3d.hpp"

class JoltAreaImpl3D;
class JoltContactListener3D;
class JoltJointImpl3D;
class JoltLayerMapper;
class JoltObjectImpl3D;
class JoltPhysicsDirectSpaceState3D;

class JoltSpace3D final {
public:
	explicit JoltSpace3D(JPH::JobSystem* p_job_system);

	~JoltSpace3D();

	void step(float p_step);

	void call_queries();

	RID get_rid() const { return rid; }

	void set_rid(const RID& p_rid) { rid = p_rid; }

	double get_param(PhysicsServer3D::SpaceParameter p_param) const;

	void set_param(PhysicsServer3D::SpaceParameter p_param, double p_value);

	JPH::PhysicsSystem& get_physics_system() const { return *physics_system; }

	JPH::BodyInterface& get_body_iface();

	const JPH::BodyInterface& get_body_iface() const;

	const JPH::BodyLockInterface& get_lock_iface() const;

	const JPH::BroadPhaseQuery& get_broad_phase_query() const;

	const JPH::NarrowPhaseQuery& get_narrow_phase_query() const;

	JPH::ObjectLayer map_to_object_layer(
		JPH::BroadPhaseLayer p_broad_phase_layer,
		uint32_t p_collision_layer,
		uint32_t p_collision_mask
	);

	void map_from_object_layer(
		JPH::ObjectLayer p_object_layer,
		JPH::BroadPhaseLayer& p_broad_phase_layer,
		uint32_t& p_collision_layer,
		uint32_t& p_collision_mask
	) const;

	JoltReadableBody3D read_body(const JPH::BodyID& p_body_id) const;

	JoltReadableBody3D read_body(const JoltObjectImpl3D& p_object) const;

	JoltWritableBody3D write_body(const JPH::BodyID& p_body_id) const;

	JoltWritableBody3D write_body(const JoltObjectImpl3D& p_object) const;

	JoltReadableBodies3D read_bodies(const JPH::BodyID* p_body_ids, int32_t p_body_count) const;

	JoltWritableBodies3D write_bodies(const JPH::BodyID* p_body_ids, int32_t p_body_count) const;

	JoltPhysicsDirectSpaceState3D* get_direct_state();

	void set_default_area(JoltAreaImpl3D* p_area) { default_area = p_area; }

	JoltAreaImpl3D* get_default_area() const { return default_area; }

	float get_last_step() const { return last_step; }

	void add_joint(JPH::Constraint* p_jolt_ref);

	void add_joint(JoltJointImpl3D* p_joint);

	void remove_joint(JPH::Constraint* p_jolt_ref);

	void remove_joint(JoltJointImpl3D* p_joint);

#ifdef GDJ_CONFIG_EDITOR
	void dump_debug_snapshot(const String& p_dir);

	const PackedVector3Array& get_debug_contacts() const;

	int32_t get_debug_contact_count() const;

	int32_t get_max_debug_contacts() const;

	void set_max_debug_contacts(int32_t p_count);
#endif // GDJ_CONFIG_EDITOR

private:
	void _pre_step(float p_step);

	void _post_step(float p_step);

	JoltBodyWriter3D body_accessor;

	RID rid;

	JPH::JobSystem* job_system = nullptr;

	JPH::TempAllocator* temp_allocator = nullptr;

	JoltLayerMapper* layer_mapper = nullptr;

	JoltContactListener3D* contact_listener = nullptr;

	JPH::PhysicsSystem* physics_system = nullptr;

	JoltPhysicsDirectSpaceState3D* direct_state = nullptr;

	JoltAreaImpl3D* default_area = nullptr;

	float last_step = 0.0f;

	bool has_stepped = false;
};
