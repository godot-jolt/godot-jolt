#pragma once

class JoltBodyActivationListener3D final : public JPH::BodyActivationListener {
	void OnBodyActivated(const JPH::BodyID& p_body_id, JPH::uint64 p_body_user_data) override;

	void OnBodyDeactivated(
		[[maybe_unused]] const JPH::BodyID& p_body_id,
		[[maybe_unused]] JPH::uint64 p_body_user_data
	) override { }
};
