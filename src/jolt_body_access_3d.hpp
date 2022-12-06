#pragma once

#include "jolt_space_3d.hpp"

class JoltBodyAccessRead3D {
public:
	JoltBodyAccessRead3D(const JoltSpace3D& p_space, const JPH::BodyID& p_jid, bool p_lock)
		: lock(p_space.get_body_lock_iface(p_lock), p_jid) { }

	bool is_valid() const { return lock.Succeeded(); }

	const JPH::Body& get_body() const { return lock.GetBody(); }

private:
	JPH::BodyLockRead lock;
};

class JoltBodyAccessWrite3D {
public:
	JoltBodyAccessWrite3D(const JoltSpace3D& p_space, const JPH::BodyID& p_jid, bool p_lock)
		: lock(p_space.get_body_lock_iface(p_lock), p_jid) { }

	bool is_valid() const { return lock.Succeeded(); }

	JPH::Body& get_body() const { return lock.GetBody(); }

private:
	JPH::BodyLockWrite lock;
};
