#pragma once

#include "jolt_space_3d.hpp"

template<typename TBodyLock>
class JoltBodyAccess3D {
public:
	JoltBodyAccess3D(const JoltSpace3D& p_space, const JPH::BodyID& p_jolt_id, bool p_lock)
		: lock(p_space.get_body_lock_iface(p_lock), p_jolt_id) { }

	bool is_valid() const { return lock.Succeeded(); }

	decltype(auto) get_body() const { return lock.GetBody(); }

private:
	TBodyLock lock;
};

template<typename TBodyLock>
class JoltMultiBodyAccess3D {
public:
	JoltMultiBodyAccess3D(
		const JoltSpace3D& p_space,
		const JPH::BodyID* p_jolt_ids,
		const int p_jolt_id_count,
		bool p_lock
	)
		: lock(p_space.get_body_lock_iface(p_lock), p_jolt_ids, p_jolt_id_count) { }

	JPH::Body* get_body(int p_body_idx) const { return lock.GetBody(p_body_idx); }

private:
	TBodyLock lock;
};

using JoltBodyAccessRead3D = JoltBodyAccess3D<JPH::BodyLockRead>;
using JoltBodyAccessWrite3D = JoltBodyAccess3D<JPH::BodyLockWrite>;

using JoltMultiBodyAccessRead3D = JoltMultiBodyAccess3D<JPH::BodyLockMultiRead>;
using JoltMultiBodyAccessWrite3D = JoltMultiBodyAccess3D<JPH::BodyLockMultiWrite>;
