#pragma once

#include "misc/error_macros.hpp"

template<typename TElement, typename TAllocator = std::allocator<TElement>>
class LocalVector {
	using Implementation = std::vector<TElement, TAllocator>;

public:
	using Iterator = typename Implementation::iterator;
	using ConstIterator = typename Implementation::const_iterator;

	LocalVector() = default;

	explicit LocalVector(int32_t p_capacity) { impl.reserve(p_capacity); }

	LocalVector(std::initializer_list<TElement> p_list)
		: impl(p_list) { }

	_FORCE_INLINE_ void push_back(const TElement& p_value) { emplace_back(p_value); }

	_FORCE_INLINE_ void push_back(TElement&& p_value) { emplace_back(std::move(p_value)); }

	template<typename... TArgs>
	_FORCE_INLINE_ TElement& emplace_back(TArgs&&... p_args) {
		return impl.emplace_back(std::forward<TArgs>(p_args)...);
	}

	_FORCE_INLINE_ void remove_at(int32_t p_index) {
		ERR_FAIL_INDEX(p_index, size());

		impl.erase(begin() + p_index);
	}

	_FORCE_INLINE_ void remove_at_unordered(int32_t p_index) {
		ERR_FAIL_INDEX(p_index, size());

		const int32_t last_index = size() - 1;

		auto element_iter = begin() + p_index;
		auto last_element_iter = begin() + last_index;

		std::swap(*element_iter, *last_element_iter);

		impl.erase(last_element_iter);
	}

	_FORCE_INLINE_ void erase(const TElement& p_value) {
		auto new_end = std::remove(begin(), end(), p_value);
		if (new_end != end()) {
			impl.erase(new_end, end());
		}
	}

	template<typename TCallable>
	_FORCE_INLINE_ int32_t erase_if(TCallable&& p_callable) {
		const auto new_end = std::remove_if(begin(), end(), std::forward<TCallable>(p_callable));
		const auto count = (int32_t)std::distance(new_end, end());
		impl.erase(new_end, end());
		return count;
	}

	_FORCE_INLINE_ void invert() { std::reverse(begin(), end()); }

	_FORCE_INLINE_ void clear() { impl.clear(); }

	_FORCE_INLINE_ bool is_empty() const { return impl.empty(); }

	_FORCE_INLINE_ int32_t get_capacity() const { return (int32_t)impl.capacity(); }

	_FORCE_INLINE_ void reserve(int32_t p_capacity) { impl.reserve((size_t)p_capacity); }

	_FORCE_INLINE_ int32_t size() const { return (int32_t)impl.size(); }

	_FORCE_INLINE_ void resize(int32_t p_size) { impl.resize((size_t)p_size); }

	_FORCE_INLINE_ void insert(int32_t p_index, const TElement& p_value) {
		emplace(p_index, p_value);
	}

	_FORCE_INLINE_ void insert(int32_t p_index, TElement&& p_value) {
		emplace(p_index, std::move(p_value));
	}

	template<typename... TArgs>
	_FORCE_INLINE_ void emplace(int32_t p_index, TArgs&&... p_args) {
		ERR_FAIL_INDEX(p_index, size() + 1);

		impl.emplace(begin() + p_index, std::forward<TArgs>(p_args)...);
	}

	_FORCE_INLINE_ void ordered_insert(const TElement& p_val) {
		auto position = std::lower_bound(begin(), end(), p_val);
		impl.insert(position, p_val);
	}

	_FORCE_INLINE_ void ordered_insert(TElement&& p_val) {
		auto position = std::lower_bound(begin(), end(), p_val);
		impl.insert(position, std::move(p_val));
	}

	_FORCE_INLINE_ int32_t find(const TElement& p_value, int32_t p_from = 0) const {
		if (p_from < size()) {
			auto found = std::find(begin() + p_from, end(), p_value);
			if (found != end()) {
				return (int32_t)std::distance(begin(), found);
			}
		}

		return -1;
	}

	template<typename TCallable>
	_FORCE_INLINE_ int32_t find_if(TCallable&& p_callable, int32_t p_from = 0) const {
		if (p_from < size()) {
			auto found = std::find_if(begin() + p_from, end(), std::forward<TCallable>(p_callable));
			if (found != end()) {
				return (int32_t)std::distance(begin(), found);
			}
		}

		return -1;
	}

	_FORCE_INLINE_ void sort() { std::sort(begin(), end()); }

	template<typename TComparer>
	_FORCE_INLINE_ void sort(TComparer&& p_comparer) {
		std::sort(begin(), end(), p_comparer);
	}

	_FORCE_INLINE_ TElement* ptr() { return impl.data(); }

	_FORCE_INLINE_ const TElement* ptr() const { return impl.data(); }

	_FORCE_INLINE_ Iterator begin() { return impl.begin(); }

	_FORCE_INLINE_ Iterator end() { return impl.end(); }

	_FORCE_INLINE_ ConstIterator begin() const { return impl.begin(); }

	_FORCE_INLINE_ ConstIterator end() const { return impl.end(); }

	_FORCE_INLINE_ ConstIterator cbegin() const { return impl.cbegin(); }

	_FORCE_INLINE_ ConstIterator cend() const { return impl.cend(); }

	_FORCE_INLINE_ TElement& operator[](int32_t p_index) {
		CRASH_BAD_INDEX(p_index, size());
		return impl[(size_t)p_index];
	}

	_FORCE_INLINE_ const TElement& operator[](int32_t p_index) const {
		CRASH_BAD_INDEX(p_index, size());
		return impl[(size_t)p_index];
	}

	_FORCE_INLINE_ LocalVector& operator=(std::initializer_list<TElement> p_list) {
		impl = p_list;
		return *this;
	}

private:
	Implementation impl;
};