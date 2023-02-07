#pragma once

template<
	typename TKey,
	typename TValue,
	typename THasher = HashMapHasherDefault,
	typename TComparator = HashMapComparatorDefault<TKey>>
class HashMap {
	struct Hasher {
		_FORCE_INLINE_ size_t operator()(const TKey& p_key) const {
			return (size_t)THasher::hash(p_key);
		}
	};

	struct Comparator {
		_FORCE_INLINE_ bool operator()(const TKey& p_lhs, const TKey& p_rhs) const {
			return TComparator::compare(p_lhs, p_rhs);
		}
	};

	using Implementation = std::unordered_map<TKey, TValue, Hasher, Comparator>;

public:
	using Iterator = typename Implementation::iterator;
	using ConstIterator = typename Implementation::const_iterator;

	HashMap() = default;

	explicit HashMap(int32_t p_capacity) { storage.reserve((size_t)p_capacity); }

	_FORCE_INLINE_ int32_t get_capacity() const {
		return int32_t(storage.max_load_factor() * storage.bucket_count());
	}

	_FORCE_INLINE_ int32_t size() const { return (int32_t)storage.size(); }

	_FORCE_INLINE_ bool is_empty() const { return storage.empty(); }

	_FORCE_INLINE_ void clear() { storage.clear(); }

	_FORCE_INLINE_ TValue& get(const TKey& p_key) { return storage.at(p_key); }

	_FORCE_INLINE_ const TValue& get(const TKey& p_key) const { return storage.at(p_key); }

	_FORCE_INLINE_ const TValue* getptr(const TKey& p_key) const {
		auto iter = storage.find(p_key);
		return iter != end() ? &iter->second : nullptr;
	}

	_FORCE_INLINE_ TValue* getptr(const TKey& p_key) {
		auto iter = storage.find(p_key);
		return iter != end() ? &iter->second : nullptr;
	}

	_FORCE_INLINE_ bool has(const TKey& p_key) const { return storage.find(p_key) != end(); }

	_FORCE_INLINE_ bool erase(const TKey& p_key) { return storage.erase(p_key) != 0; }

	_FORCE_INLINE_ void reserve(int32_t p_capacity) { storage.reserve((size_t)p_capacity); }

	_FORCE_INLINE_ Iterator find(const TKey& p_key) { return storage.find(p_key); }

	_FORCE_INLINE_ ConstIterator find(const TKey& p_key) const { return storage.find(p_key); }

	_FORCE_INLINE_ void remove(ConstIterator p_iter) { storage.erase(p_iter); }

	_FORCE_INLINE_ Iterator insert(const TKey& p_key, const TValue& p_value) {
		return emplace(p_key, p_value);
	}

	_FORCE_INLINE_ Iterator insert(const TKey& p_key, TValue&& p_value) {
		return emplace(p_key, std::move(p_value));
	}

	_FORCE_INLINE_ Iterator insert(TKey&& p_key, const TValue& p_value) {
		return emplace(std::move(p_key), p_value);
	}

	_FORCE_INLINE_ Iterator insert(TKey&& p_key, TValue&& p_value) {
		return emplace(std::move(p_key), std::move(p_value));
	}

	template<typename... TArgs>
	_FORCE_INLINE_ Iterator emplace(const TKey& p_key, TArgs&&... p_args) {
		auto [iter, inserted] = storage.try_emplace(p_key, std::forward<TArgs>(p_args)...);

		if (!inserted) {
			iter->second = TValue(std::forward<TArgs>(p_args)...);
		}

		return iter;
	}

	template<typename... TArgs>
	_FORCE_INLINE_ Iterator emplace(TKey&& p_key, TArgs&&... p_args) {
		auto [iter, inserted] =
			storage.try_emplace(std::move(p_key), std::forward<TArgs>(p_args)...);

		if (!inserted) {
			iter->second = TValue(std::forward<TArgs>(p_args)...);
		}

		return iter;
	}

	_FORCE_INLINE_ Iterator begin() { return storage.begin(); }

	_FORCE_INLINE_ Iterator end() { return storage.end(); }

	_FORCE_INLINE_ ConstIterator begin() const { return storage.begin(); }

	_FORCE_INLINE_ ConstIterator end() const { return storage.end(); }

	_FORCE_INLINE_ ConstIterator cbegin() const { return storage.cbegin(); }

	_FORCE_INLINE_ ConstIterator cend() const { return storage.cend(); }

	_FORCE_INLINE_ TValue& operator[](const TKey& p_key) { return storage[p_key]; }

	_FORCE_INLINE_ TValue& operator[](TKey&& p_key) { return storage[std::move(p_key)]; }

	_FORCE_INLINE_ const TValue& operator[](const TKey& p_key) const { return get(p_key); }

private:
	Implementation storage;
};
