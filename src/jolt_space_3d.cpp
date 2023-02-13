#include "jolt_space_3d.hpp"

#include "jolt_area_3d.hpp"
#include "jolt_body_3d.hpp"
#include "jolt_broad_phase_layer.hpp"
#include "jolt_contact_listener.hpp"
#include "jolt_group_filter_rid.hpp"
#include "jolt_joint_3d.hpp"
#include "jolt_layer_mapper.hpp"
#include "jolt_physics_direct_space_state_3d.hpp"
#include "jolt_shape_3d.hpp"
#include "jolt_temp_allocator.hpp"

namespace {

constexpr uint32_t GDJOLT_TEMP_CAPACITY = 8 * 1024 * 1024;
constexpr uint32_t GDJOLT_MAX_BODIES = 8192;
constexpr uint32_t GDJOLT_BODY_MUTEX_COUNT = 0; // 0 = default
constexpr uint32_t GDJOLT_MAX_BODY_PAIRS = 65536;
constexpr uint32_t GDJOLT_MAX_CONTACT_CONSTRAINTS = 8192;

} // namespace

JoltSpace3D::JoltSpace3D(JPH::JobSystem* p_job_system)
	: job_system(p_job_system)
	, temp_allocator(new JoltTempAllocator(GDJOLT_TEMP_CAPACITY))
	, layer_mapper(new JoltLayerMapper())
	, contact_listener(new JoltContactListener(this))
	, physics_system(new JPH::PhysicsSystem())
	, body_accessor(this) {
	physics_system->Init(
		GDJOLT_MAX_BODIES,
		GDJOLT_BODY_MUTEX_COUNT,
		GDJOLT_MAX_BODY_PAIRS,
		GDJOLT_MAX_CONTACT_CONSTRAINTS,
		*layer_mapper,
		*layer_mapper,
		*layer_mapper
	);

	physics_system->SetGravity(JPH::Vec3::sZero());
	physics_system->SetContactListener(contact_listener);
}

JoltSpace3D::~JoltSpace3D() {
	memdelete_safely(direct_state);
	delete_safely(physics_system);
	delete_safely(contact_listener);
	delete_safely(layer_mapper);
	delete_safely(temp_allocator);
}

void JoltSpace3D::step(float p_step) {
	pre_step(p_step);

	physics_system->Update(p_step, 1, 1, temp_allocator, job_system);

	post_step(p_step);
}

void JoltSpace3D::call_queries() {
	body_accessor.acquire_active();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (!body->IsStatic() && !body->IsSensor()) {
				reinterpret_cast<JoltBody3D*>(body->GetUserData())->call_queries();
			}
		}
	}

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (!body->IsStatic() && body->IsSensor()) {
				reinterpret_cast<JoltArea3D*>(body->GetUserData())->call_queries();
			}
		}
	}

	body_accessor.release();
}

JPH::BodyInterface& JoltSpace3D::get_body_iface(bool p_locked) {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyInterface();
	} else {
		return physics_system->GetBodyInterfaceNoLock();
	}
}

const JPH::BodyInterface& JoltSpace3D::get_body_iface(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyInterface();
	} else {
		return physics_system->GetBodyInterfaceNoLock();
	}
}

const JPH::BodyLockInterface& JoltSpace3D::get_lock_iface(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyLockInterface();
	} else {
		return physics_system->GetBodyLockInterfaceNoLock();
	}
}

const JPH::NarrowPhaseQuery& JoltSpace3D::get_narrow_phase_query(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetNarrowPhaseQuery();
	} else {
		return physics_system->GetNarrowPhaseQueryNoLock();
	}
}

JPH::ObjectLayer JoltSpace3D::map_to_object_layer(
	JPH::BroadPhaseLayer p_broad_phase_layer,
	uint32_t p_collision_layer,
	uint32_t p_collision_mask
) {
	return layer_mapper->to_object_layer(p_broad_phase_layer, p_collision_layer, p_collision_mask);
}

JoltReadableBody3D JoltSpace3D::read_body(const JPH::BodyID& p_body_id, bool p_lock) const {
	return {*this, p_body_id, p_lock};
}

JoltReadableBody3D JoltSpace3D::read_body(const JoltBody3D& p_body, bool p_lock) const {
	return read_body(p_body.get_jolt_id(), p_lock);
}

JoltWritableBody3D JoltSpace3D::write_body(const JPH::BodyID& p_body_id, bool p_lock) const {
	return {*this, p_body_id, p_lock};
}

JoltWritableBody3D JoltSpace3D::write_body(const JoltBody3D& p_body, bool p_lock) const {
	return write_body(p_body.get_jolt_id(), p_lock);
}

JoltReadableBodies3D JoltSpace3D::read_bodies(
	const JPH::BodyID* p_body_ids,
	int32_t p_body_count,
	bool p_lock
) const {
	return {*this, p_body_ids, p_body_count, p_lock};
}

JoltWritableBodies3D JoltSpace3D::write_bodies(
	const JPH::BodyID* p_body_ids,
	int32_t p_body_count,
	bool p_lock
) const {
	return {*this, p_body_ids, p_body_count, p_lock};
}

JoltPhysicsDirectSpaceState3D* JoltSpace3D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(JoltPhysicsDirectSpaceState3D(this));
	}

	return direct_state;
}

void JoltSpace3D::add_joint(JoltJoint3D* p_joint) {
	physics_system->AddConstraint(p_joint->get_jolt_ref());
}

void JoltSpace3D::remove_joint(JoltJoint3D* p_joint) {
	physics_system->RemoveConstraint(p_joint->get_jolt_ref());
}

void JoltSpace3D::pre_step(float p_step) {
	body_accessor.acquire_all(true);

	contact_listener->pre_step();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* jolt_body = body_accessor.try_get(i)) {
			auto* object = reinterpret_cast<JoltCollisionObject3D*>(jolt_body->GetUserData());

			object->pre_step(p_step);

			if (object->generates_contacts()) {
				contact_listener->listen_for(object);
			}
		}
	}

	body_accessor.release();
}

void JoltSpace3D::post_step(float p_step) {
	body_accessor.acquire_all(true);

	contact_listener->post_step();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* jolt_body = body_accessor.try_get(i)) {
			auto* object = reinterpret_cast<JoltCollisionObject3D*>(jolt_body->GetUserData());

			object->post_step(p_step);
		}
	}

	body_accessor.release();
}
