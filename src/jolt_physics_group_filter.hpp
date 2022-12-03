#pragma once

class JoltPhysicsGroupFilter final : public JPH::GroupFilter {
public:
	bool CanCollide(const JPH::CollisionGroup& p_first, const JPH::CollisionGroup& p_second)
		const override;
};
