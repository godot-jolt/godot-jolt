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

_FORCE_INLINE_ double estimate_physics_step() {
	Engine* engine = Engine::get_singleton();

	const double step = 1.0 / engine->get_physics_ticks_per_second();
	const double step_scaled = step * engine->get_time_scale();

	return step_scaled;
}
