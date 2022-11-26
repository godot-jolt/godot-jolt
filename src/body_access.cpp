#include "body_access.hpp"

#include "jolt_physics_space_3d.hpp"

BodyAccessRead::BodyAccessRead(
	const JoltPhysicsSpace3D& p_space,
	const JPH::BodyID& p_jid,
	bool p_lock
)
	: lock(p_space.get_body_lock_iface(p_lock), p_jid) { }

BodyAccessWrite::BodyAccessWrite(
	const JoltPhysicsSpace3D& p_space,
	const JPH::BodyID& p_jid,
	bool p_lock
)
	: lock(p_space.get_body_lock_iface(p_lock), p_jid) { }
