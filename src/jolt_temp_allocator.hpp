#pragma once

#include "utility_functions.hpp"

class JoltTempAllocator final : public JPH::TempAllocator {
public:
	explicit JoltTempAllocator(size_t p_capacity)
		: base(static_cast<uint8_t*>(JPH::Allocate(p_capacity)))
		, capacity(p_capacity) { }

	~JoltTempAllocator() override { JPH::Free(base); }

	void* Allocate(uint32_t p_size) override {
		if (p_size == 0) {
			return nullptr;
		}

		p_size = align_up(p_size, 16U);

		const size_t new_top = top + p_size;

		void* ptr = nullptr;

		if (new_top <= capacity) {
			ptr = base + top;
		} else {
			WARN_PRINT_ONCE(
				"Temporary memory allocator exceeded capacity. "
				"Falling back to slower general-purpose allocator. "
				"Consider increasing temporary memory capacity."
			);

			ptr = JPH::Allocate(p_size);
		}

		top = new_top;

		return ptr;
	}

	void Free(void* p_ptr, uint32_t p_size) override {
		if (p_ptr == nullptr) {
			return;
		}

		p_size = align_up(p_size, 16U);

		const size_t new_top = top - p_size;

		if (top <= capacity) {
			if (base + new_top != p_ptr) {
				CRASH_NOW_MSG("Temporary memory was freed in the wrong order.");
			}
		} else {
			JPH::Free(p_ptr);
		}

		top = new_top;
	}

private:
	uint8_t* base = nullptr;

	size_t capacity = 0;

	size_t top = 0;
};
