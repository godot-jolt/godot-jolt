#include "jolt_physics_group_filter_3d.hpp"

bool JoltPhysicsGroupFilter3D::CanCollide(
	const JPH::CollisionGroup& p_first,
	const JPH::CollisionGroup& p_second
) const {
	return (p_first.GetGroupID() & p_second.GetSubGroupID()) != 0 ||
		(p_second.GetGroupID() & p_first.GetSubGroupID()) != 0;
}
