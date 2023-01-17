#pragma once

#include "variant.hpp"

template<typename TResource>
class RID_PtrOwner {
public:
	RID_PtrOwner() = default;

	RID_PtrOwner(const RID_PtrOwner& p_other) = default;

	RID_PtrOwner(RID_PtrOwner&& p_other) = default;

	~RID_PtrOwner() {
		if (ptr_by_id.size() > 0) {
			ERR_PRINT(vformat("{} RIDs were leaked.", ptr_by_id.size()));
		}
	}

	_FORCE_INLINE_ RID make_rid(TResource* p_ptr) {
		const int64_t id = UtilityFunctions::rid_allocate_id();
		ptr_by_id[id] = p_ptr;
		return UtilityFunctions::rid_from_int64(id);
	}

	_FORCE_INLINE_ TResource* get_or_null(const RID& p_rid) const {
		auto iter = ptr_by_id.find(p_rid.get_id());
		return iter != ptr_by_id.end() ? iter->second : nullptr;
	}

	_FORCE_INLINE_ void replace(const RID& p_rid, TResource* p_new_ptr) {
		auto iter = ptr_by_id.find(p_rid.get_id());
		ERR_FAIL_COND(iter == ptr_by_id.end());
		iter->second = p_new_ptr;
	}

	_FORCE_INLINE_ bool owns(const RID& p_rid) const { return ptr_by_id.has(p_rid.get_id()); }

	_FORCE_INLINE_ void free(const RID& p_rid) { ptr_by_id.erase(p_rid.get_id()); }

	RID_PtrOwner& operator=(const RID_PtrOwner& p_other) = default;

	RID_PtrOwner& operator=(RID_PtrOwner&& p_other) = default;

private:
	HashMap<int64_t, TResource*> ptr_by_id;
};
