#include "jolt_group_filter_rid.hpp"

void JoltGroupFilterRID::encode_rid(
	const RID& p_rid,
	JPH::CollisionGroup::GroupID& p_group_id,
	JPH::CollisionGroup::SubGroupID& p_sub_group_id
) {
	const auto rid_id = (uint64_t)p_rid.get_id();
	p_group_id = JPH::CollisionGroup::GroupID(rid_id >> 32U);
	p_sub_group_id = JPH::CollisionGroup::SubGroupID(rid_id & 0xFFFFFFFFULL);
}

RID JoltGroupFilterRID::decode_rid(
	JPH::CollisionGroup::GroupID p_group_id,
	JPH::CollisionGroup::SubGroupID p_sub_group_id
) {
	const auto upper_bits = (uint64_t)p_group_id << 32U;
	const auto lower_bits = (uint64_t)p_sub_group_id;
	const auto rid_id = int64_t(upper_bits | lower_bits);
	return UtilityFunctions::rid_from_int64(rid_id);
}

bool JoltGroupFilterRID::CanCollide(
	[[maybe_unused]] const JPH::CollisionGroup& p_first,
	const JPH::CollisionGroup& p_second
) const {
	return !has_exception(decode_rid(p_second.GetGroupID(), p_second.GetSubGroupID()));
}

static_assert(sizeof(RID) == 8);
static_assert(sizeof(JPH::CollisionGroup::GroupID) == 4);
static_assert(sizeof(JPH::CollisionGroup::SubGroupID) == 4);
