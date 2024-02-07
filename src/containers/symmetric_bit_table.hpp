#pragma once

#include "containers/local_vector.hpp"

class SymmetricBitTable {
public:
	explicit SymmetricBitTable(int32_t p_size)
		: size(p_size) {
		const int32_t total_bits = (size * (size + 1)) / 2;
		const int32_t total_bytes = (total_bits + 7) / 8;
		bits.resize(total_bytes);
		memset(bits.ptr(), 0, (size_t)bits.size());
	}

	void set(int32_t p_x, int32_t p_y) {
		ERR_FAIL_INDEX(p_x, size);
		ERR_FAIL_INDEX(p_y, size);

		const int32_t index = _calculate_index(p_x, p_y);
		const int32_t byte_index = index / 8;
		const auto bit_index = uint8_t(index % 8);

		bits[byte_index] |= (1U << bit_index);
	}

	void unset(int32_t p_x, int32_t p_y) {
		ERR_FAIL_INDEX(p_x, size);
		ERR_FAIL_INDEX(p_y, size);

		const int32_t index = _calculate_index(p_x, p_y);
		const int32_t byte_index = index / 8;
		const auto bit_index = uint8_t(index % 8);

		bits[byte_index] &= ~(1U << bit_index);
	}

	bool has(int32_t p_x, int32_t p_y) const {
		CRASH_BAD_INDEX(p_x, size);
		CRASH_BAD_INDEX(p_y, size);

		const int32_t index = _calculate_index(p_x, p_y);
		const int32_t byte_index = index / 8;
		const auto bit_index = uint8_t(index % 8);

		return (uint8_t(bits[byte_index] >> bit_index) & 1U) != 0;
	}

private:
	int32_t _calculate_index(int32_t p_x, int32_t p_y) const {
		if (p_x < p_y) {
			std::swap(p_x, p_y);
		}

		return (p_x * (p_x + 1)) / 2 + p_y;
	}

	LocalVector<uint8_t> bits;

	int32_t size = 0;
};
