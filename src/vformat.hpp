#pragma once

template<typename... TArgs>
String vformat(TArgs&&... p_args) {
	return String(fmt::format(std::forward<TArgs>(p_args)...).c_str());
}
