#pragma once

#define GDJOLT_CONCATENATE_IMPL(m_a, m_b) m_a##m_b
#define GDJOLT_CONCATENATE(m_a, m_b) GDJOLT_CONCATENATE_IMPL(m_a, m_b)
#define GDJOLT_UNIQUE_IDENTIFIER(m_prefix) GDJOLT_CONCATENATE(m_prefix, __COUNTER__)

template<typename TCallable>
class ScopeGuard {
public:
	// NOLINTNEXTLINE(hicpp-explicit-conversions)
	ScopeGuard(TCallable&& p_callable)
		: callable(std::forward<TCallable>(p_callable)) { }

	ScopeGuard(const ScopeGuard& p_other) = delete;

	ScopeGuard(ScopeGuard&& p_other) = delete;

	~ScopeGuard() {
		if (!released) {
			callable();
		}
	}

	void release() { released = true; }

	ScopeGuard& operator=(const ScopeGuard& p_other) = delete;

	ScopeGuard& operator=(ScopeGuard&& p_other) = delete;

private:
	TCallable callable;

	bool released = false;
};

struct ScopeGuardHelper {
	template<typename TCallable>
	ScopeGuard<std::decay_t<TCallable>> operator+(TCallable&& p_callable) {
		return {std::forward<TCallable>(p_callable)};
	}
};

// NOLINTNEXTLINE(bugprone-macro-parentheses)
#define ON_SCOPE_EXIT auto GDJOLT_UNIQUE_IDENTIFIER(scope_guard) = ScopeGuardHelper() + [&]
