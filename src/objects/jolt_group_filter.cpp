#include "jolt_group_filter.hpp"

#include "objects/jolt_area_impl_3d.hpp"
#include "objects/jolt_body_impl_3d.hpp"
#include "objects/jolt_object_impl_3d.hpp"

void JoltGroupFilter::encode_object(
	const JoltObjectImpl3D* p_object,
	JPH::CollisionGroup::GroupID& p_group_id,
	JPH::CollisionGroup::SubGroupID& p_sub_group_id
) {
	// HACK(mihe): Since group filters don't grant us access to the bodies or their user data we are
	// instead forced abuse the collision group to carry the upper and lower bits of our pointer,
	// which we can access and decode in `CanCollide`.

	const auto address = reinterpret_cast<uint64_t>(p_object);
	p_group_id = JPH::CollisionGroup::GroupID(address >> 32U);
	p_sub_group_id = JPH::CollisionGroup::SubGroupID(address & 0xFFFFFFFFULL);
}

const JoltObjectImpl3D* JoltGroupFilter::decode_object(
	JPH::CollisionGroup::GroupID p_group_id,
	JPH::CollisionGroup::SubGroupID p_sub_group_id
) {
	const auto upper_bits = (uint64_t)p_group_id << 32U;
	const auto lower_bits = (uint64_t)p_sub_group_id;
	const auto address = uint64_t(upper_bits | lower_bits);
	return reinterpret_cast<const JoltObjectImpl3D*>(address);
}

bool JoltGroupFilter::CanCollide(
	const JPH::CollisionGroup& p_group1,
	const JPH::CollisionGroup& p_group2
) const {
	const JoltObjectImpl3D* object1 = decode_object(
		p_group1.GetGroupID(),
		p_group1.GetSubGroupID()
	);

	const JoltObjectImpl3D* object2 = decode_object(
		p_group2.GetGroupID(),
		p_group2.GetSubGroupID()
	);

	const JoltAreaImpl3D* area1 = object1->is_area()
		? reinterpret_cast<const JoltAreaImpl3D*>(object1)
		: nullptr;

	const JoltAreaImpl3D* area2 = object2->is_area()
		? reinterpret_cast<const JoltAreaImpl3D*>(object2)
		: nullptr;

	const JoltBodyImpl3D* body1 = area1 == nullptr
		? reinterpret_cast<const JoltBodyImpl3D*>(object1)
		: nullptr;

	const JoltBodyImpl3D* body2 = area2 == nullptr
		? reinterpret_cast<const JoltBodyImpl3D*>(object2)
		: nullptr;

	if (body1 != nullptr && body2 != nullptr) {
		return body1->can_interact_with(*body2);
	}

	if (area1 != nullptr && area2 != nullptr) {
		return area1->can_interact_with(*area2);
	} else if (area1 != nullptr && body2 != nullptr) {
		return area1->can_interact_with(*body2);
	} else if (area2 != nullptr && body1 != nullptr) {
		return area2->can_interact_with(*body1);
	} else {
		return false;
	}
}

// NOLINTNEXTLINE(bugprone-sizeof-expression)
static_assert(sizeof(JoltObjectImpl3D*) <= 8);
static_assert(sizeof(JPH::CollisionGroup::GroupID) == 4);
static_assert(sizeof(JPH::CollisionGroup::SubGroupID) == 4);
