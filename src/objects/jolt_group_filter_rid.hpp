#pragma once

class JoltGroupFilterRID final : public JPH::GroupFilter {
public:
	static void encode_rid(
		const RID& p_rid,
		JPH::CollisionGroup::GroupID& p_group_id,
		JPH::CollisionGroup::SubGroupID& p_sub_group_id
	);

	static RID decode_rid(
		JPH::CollisionGroup::GroupID p_group_id,
		JPH::CollisionGroup::SubGroupID p_sub_group_id
	);

	void add_exception(const RID& p_exception) const { exceptions.push_back(p_exception); }

	void remove_exception(const RID& p_exception) const { exceptions.erase(p_exception); }

	bool has_exception(const RID& p_exception) const { return exceptions.find(p_exception) >= 0; }

	const RID* get_exceptions() const { return exceptions.ptr(); }

	int32_t get_exception_count() const { return exceptions.size(); }

private:
	bool CanCollide(const JPH::CollisionGroup& p_first, const JPH::CollisionGroup& p_second)
		const override;

	mutable InlineVector<RID, 1> exceptions;
};
