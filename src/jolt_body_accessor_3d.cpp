#include "jolt_body_accessor_3d.hpp"

#include "jolt_space_3d.hpp"

static int32_t acquired_count = 0;

JoltBodyAccessor3D::JoltBodyAccessor3D(const JoltSpace3D* p_space)
	: space(p_space) { }

JoltBodyAccessor3D::~JoltBodyAccessor3D() = default;

void JoltBodyAccessor3D::acquire(const JPH::BodyID* p_ids, int32_t p_id_count, bool p_lock) {
	lock_iface = &space->get_body_lock_iface(p_lock);
	ids.assign(p_ids, p_ids + p_id_count);
	acquire_internal(p_ids, p_id_count);
}

void JoltBodyAccessor3D::acquire(const JPH::BodyID& p_id, bool p_lock) {
	lock_iface = &space->get_body_lock_iface(p_lock);
	ids.assign(1, p_id);
	acquire_internal(&p_id, 1);
}

void JoltBodyAccessor3D::acquire_active(bool p_lock) {
	lock_iface = &space->get_body_lock_iface(p_lock);
	space->get_physics_system()->GetActiveBodies(ids);
	acquire_internal(ids.data(), (int32_t)ids.size());
}

void JoltBodyAccessor3D::acquire_all(bool p_lock) {
	lock_iface = &space->get_body_lock_iface(p_lock);
	space->get_physics_system()->GetBodies(ids);
	acquire_internal(ids.data(), (int32_t)ids.size());
}

void JoltBodyAccessor3D::release() {
	release_internal();
	lock_iface = nullptr;
}

const JPH::BodyID* JoltBodyAccessor3D::get_ids() const {
	ERR_FAIL_COND_D(not_acquired());
	return ids.data();
}

int32_t JoltBodyAccessor3D::get_count() const {
	ERR_FAIL_COND_D(not_acquired());
	return (int32_t)ids.size();
}

JoltBodyReader3D::JoltBodyReader3D(const JoltSpace3D* p_space)
	: JoltBodyAccessor3D(p_space) { }

const JPH::Body* JoltBodyReader3D::try_get(const JPH::BodyID& p_id) const {
	ERR_FAIL_COND_D(not_acquired());
	return lock_iface->TryGetBody(p_id);
}

const JPH::Body* JoltBodyReader3D::try_get(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, get_count());
	return try_get(ids[(size_t)p_index]);
}

const JPH::Body* JoltBodyReader3D::try_get() const {
	ERR_FAIL_COND_D(get_count() < 1);
	return try_get(ids[0]);
}

void JoltBodyReader3D::acquire_internal(const JPH::BodyID* p_ids, int32_t p_id_count) {
	mutex_mask = lock_iface->GetMutexMask(p_ids, p_id_count);
	lock_iface->LockRead(mutex_mask);
	acquired_count++;
}

void JoltBodyReader3D::release_internal() {
	ERR_FAIL_COND(not_acquired());
	lock_iface->UnlockRead(mutex_mask);
	acquired_count--;
}

JoltBodyWriter3D::JoltBodyWriter3D(const JoltSpace3D* p_space)
	: JoltBodyAccessor3D(p_space) { }

JPH::Body* JoltBodyWriter3D::try_get(const JPH::BodyID& p_id) const {
	ERR_FAIL_COND_D(not_acquired());
	return lock_iface->TryGetBody(p_id);
}

JPH::Body* JoltBodyWriter3D::try_get(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, get_count());
	return try_get(ids[(size_t)p_index]);
}

JPH::Body* JoltBodyWriter3D::try_get() const {
	ERR_FAIL_COND_D(get_count() == 0);
	return try_get(ids[0]);
}

void JoltBodyWriter3D::acquire_internal(const JPH::BodyID* p_ids, int32_t p_id_count) {
	mutex_mask = lock_iface->GetMutexMask(p_ids, p_id_count);
	lock_iface->LockWrite(mutex_mask);
	acquired_count++;
}

void JoltBodyWriter3D::release_internal() {
	ERR_FAIL_COND(not_acquired());
	lock_iface->UnlockWrite(mutex_mask);
	acquired_count--;
}
