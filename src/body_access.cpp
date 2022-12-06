#include "body_access.hpp"

#include "jolt_physics_space_3d.hpp"

JoltBodyAccessRead3D::JoltBodyAccessRead3D(
	const JoltSpace3D& p_space,
	const JPH::BodyID& p_jid,
	bool p_lock
)
	: lock(p_space.get_body_lock_iface(p_lock), p_jid) { }

JoltBodyAccessWrite3D::JoltBodyAccessWrite3D(
	const JoltSpace3D& p_space,
	const JPH::BodyID& p_jid,
	bool p_lock
)
	: lock(p_space.get_body_lock_iface(p_lock), p_jid) { }
