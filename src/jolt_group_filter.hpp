#pragma once

class JoltGroupFilter final : public JPH::GroupFilter {
public:
	bool CanCollide(const JPH::CollisionGroup& p_first, const JPH::CollisionGroup& p_second)
		const override {
		const JPH::CollisionGroup::GroupID first_collision_layer = p_first.GetGroupID();
		const JPH::CollisionGroup::SubGroupID first_collision_mask = p_second.GetSubGroupID();

		const JPH::CollisionGroup::GroupID second_collision_layer = p_second.GetGroupID();
		const JPH::CollisionGroup::SubGroupID second_collision_mask = p_second.GetSubGroupID();

		const bool first_scans_second = (first_collision_mask & second_collision_layer) != 0;
		const bool second_scans_first = (second_collision_mask & first_collision_layer) != 0;

		return first_scans_second || second_scans_first;
	}
};
