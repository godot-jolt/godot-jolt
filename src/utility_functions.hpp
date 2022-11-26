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
