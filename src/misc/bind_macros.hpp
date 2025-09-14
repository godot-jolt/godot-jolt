#pragma once

#ifdef __clang__
#if __clang_major__ >= 20
#define PUSH_ZERO_VARIADIC_WARNING_SUPPRESSION \
	_Pragma("clang diagnostic push")           \
		_Pragma("clang diagnostic ignored \"-Wvariadic-macro-arguments-omitted\"")
#else // __clang_major__ >= 20
#define PUSH_ZERO_VARIADIC_WARNING_SUPPRESSION \
	_Pragma("clang diagnostic push") _Pragma("clang diagnostic ignored \"-Wc++20-extensions\"")
#endif // __clang_major__ >= 20
#define POP_ZERO_VARIADIC_WARNING_SUPPRESSION _Pragma("clang diagnostic pop")
#else // __clang__
#define PUSH_ZERO_VARIADIC_WARNING_SUPPRESSION
#define POP_ZERO_VARIADIC_WARNING_SUPPRESSION
#endif // __clang__

#define BIND_METHOD_0_ARGS(m_class, m_name) \
	ClassDB::bind_method(D_METHOD(#m_name), &m_class::m_name)

#define BIND_METHOD_N_ARGS(m_class, m_name, ...) \
	ClassDB::bind_method(D_METHOD(#m_name, __VA_ARGS__), &m_class::m_name)

#define BIND_METHOD_SELECT(_1, _2, _3, _4, _5, _6, _7, _8, _9, m_macro, ...) m_macro

#define BIND_METHOD(...)                   \
	PUSH_ZERO_VARIADIC_WARNING_SUPPRESSION \
	BIND_METHOD_SELECT(                    \
		__VA_ARGS__,                       \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_N_ARGS,                \
		BIND_METHOD_0_ARGS                 \
	)                                      \
	(__VA_ARGS__) POP_ZERO_VARIADIC_WARNING_SUPPRESSION

#define BIND_PROPERTY_HINTED(m_name, m_type, m_hint, m_hint_str) \
	ClassDB::add_property(                                       \
		get_class_static(),                                      \
		PropertyInfo(m_type, m_name, m_hint, m_hint_str),        \
		"set_" m_name,                                           \
		"get_" m_name                                            \
	)

#define BIND_PROPERTY_RANGED(m_name, m_type, m_hint_str) \
	BIND_PROPERTY_HINTED(m_name, m_type, PROPERTY_HINT_RANGE, m_hint_str)

#define BIND_PROPERTY_ENUM(m_name, m_hint_str) \
	BIND_PROPERTY_HINTED(m_name, Variant::INT, PROPERTY_HINT_ENUM, m_hint_str)

#define BIND_PROPERTY_0(m_name, m_type) BIND_PROPERTY_HINTED(m_name, m_type, PROPERTY_HINT_NONE, "")

#define BIND_PROPERTY_1(m_name, m_type, m_hint_str) \
	BIND_PROPERTY_HINTED(m_name, m_type, PROPERTY_HINT_NONE, m_hint_str)

#define BIND_PROPERTY_SELECT(_1, _2, _3, m_macro, ...) m_macro

#define BIND_PROPERTY(...)                                                           \
	PUSH_ZERO_VARIADIC_WARNING_SUPPRESSION                                           \
	BIND_PROPERTY_SELECT(__VA_ARGS__, BIND_PROPERTY_1, BIND_PROPERTY_0)(__VA_ARGS__) \
		POP_ZERO_VARIADIC_WARNING_SUPPRESSION

// HACK(mihe): Ideally we would use `ADD_SUBGROUP` instead of this `BIND_SUBPROPERTY` stuff, but
// there's a bug in `DocTools::generate` where you'll sometimes get errors about some empty class
// name when you have multiple sub-groups of the same name.

#define BIND_SUBPROPERTY_HINTED(m_prefix, m_name, m_type, m_hint, m_hint_str) \
	ClassDB::add_property(                                                    \
		get_class_static(),                                                   \
		PropertyInfo(m_type, m_prefix "/" m_name, m_hint, m_hint_str),        \
		"set_" m_prefix "_" m_name,                                           \
		"get_" m_prefix "_" m_name                                            \
	)

#define BIND_SUBPROPERTY_RANGED(m_prefix, m_name, m_type, m_hint_str) \
	BIND_SUBPROPERTY_HINTED(m_prefix, m_name, m_type, PROPERTY_HINT_RANGE, m_hint_str)

#define BIND_SUBPROPERTY_ENUM(m_prefix, m_name, m_hint_str) \
	BIND_SUBPROPERTY_HINTED(m_prefix, m_name, Variant::INT, PROPERTY_HINT_ENUM, m_hint_str)

#define BIND_SUBPROPERTY_0(m_prefix, m_name, m_type) \
	BIND_SUBPROPERTY_HINTED(m_prefix, m_name, m_type, PROPERTY_HINT_NONE, "")

#define BIND_SUBPROPERTY_1(m_prefix, m_name, m_type, m_hint_str) \
	BIND_SUBPROPERTY_HINTED(m_prefix, m_name, m_type, PROPERTY_HINT_NONE, m_hint_str)

#define BIND_SUBPROPERTY_SELECT(_1, _2, _3, _4, m_macro, ...) m_macro

#define BIND_SUBPROPERTY(...)                                                                 \
	PUSH_ZERO_VARIADIC_WARNING_SUPPRESSION                                                    \
	BIND_SUBPROPERTY_SELECT(__VA_ARGS__, BIND_SUBPROPERTY_1, BIND_SUBPROPERTY_0)(__VA_ARGS__) \
		POP_ZERO_VARIADIC_WARNING_SUPPRESSION
