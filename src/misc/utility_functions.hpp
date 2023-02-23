#pragma once

template<typename TType, typename... TRest>
constexpr const TType& min(const TType& p_first, const TType& p_second, const TRest&... p_rest) {
	if constexpr (sizeof...(p_rest) == 0) {
		return p_first < p_second ? p_first : p_second;
	} else {
		return min(min(p_first, p_second), p_rest...);
	}
}

template<typename TType, typename... TRest>
constexpr const TType& max(const TType& p_first, const TType& p_second, const TRest&... p_rest) {
	if constexpr (sizeof...(p_rest) == 0) {
		return p_first > p_second ? p_first : p_second;
	} else {
		return max(max(p_first, p_second), p_rest...);
	}
}

template<typename TType>
constexpr const TType& clamp(const TType& p_value, const TType& p_min, const TType& p_max) {
	return min(max(p_value, p_min), p_max);
}

template<typename TValue, typename TAlignment>
constexpr TValue align_up(TValue p_value, TAlignment p_alignment) {
	return (p_value + p_alignment - 1) & ~(p_alignment - 1);
}

template<typename TValue>
constexpr bool is_power_of_2(TValue p_value) {
	return (p_value & (p_value - 1)) == 0;
}

template<typename TElement, int32_t TSize>
constexpr int32_t count_of([[maybe_unused]] TElement (&p_array)[TSize]) {
	return TSize;
}

template<typename TType>
_FORCE_INLINE_ void delete_safely(TType*& p_ptr) {
	delete p_ptr;
	p_ptr = nullptr;
}

template<typename TType>
_FORCE_INLINE_ void memdelete_safely(TType*& p_ptr) {
	if (p_ptr != nullptr) {
		memdelete(p_ptr);
		p_ptr = nullptr;
	}
}

_FORCE_INLINE_ bool try_extract_scale(Basis& p_basis, Vector3& p_scale) {
	p_scale = Vector3(1.0f, 1.0f, 1.0f);

	Vector3 x = p_basis.get_column(Vector3::AXIS_X);
	Vector3 y = p_basis.get_column(Vector3::AXIS_Y);
	Vector3 z = p_basis.get_column(Vector3::AXIS_Z);

	const Vector3 scale_squared(x.length_squared(), y.length_squared(), z.length_squared());

	if (p_scale == scale_squared) {
		return false;
	}

	p_scale = Vector3(
		Math::sqrt(scale_squared.x),
		Math::sqrt(scale_squared.y),
		Math::sqrt(scale_squared.z)
	);

	x /= p_scale.x;
	y /= p_scale.y;
	z /= p_scale.z;

	p_basis.set_column(Vector3::AXIS_X, x);
	p_basis.set_column(Vector3::AXIS_Y, y);
	p_basis.set_column(Vector3::AXIS_Z, z);

	return true;
}

_FORCE_INLINE_ bool try_extract_scale(Transform3D& p_transform, Vector3& p_scale) {
	return try_extract_scale(p_transform.basis, p_scale);
}

inline double calculate_physics_step() {
	// TODO(mihe): Replace these constants with the appropriate singleton calls once
	// godotengine/godot-cpp#889 is fixed
	const int32_t ticks_per_second = 60; // Engine::get_physics_ticks_per_second()
	const double time_scale = 1.0; // Engine::get_time_scale()

	const double step = 1.0 / ticks_per_second;
	const double step_scaled = step * time_scale;

	return step_scaled;
}
