#pragma once

class JoltGroupFilter final : public JPH::GroupFilter {
public:
	bool CanCollide(const JPH::CollisionGroup& p_first, const JPH::CollisionGroup& p_second)
		const override {
		return (p_first.GetGroupID() & p_second.GetSubGroupID()) != 0 ||
			(p_second.GetGroupID() & p_first.GetSubGroupID()) != 0;
	}
};
