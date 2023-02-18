#pragma once

#define BIND_METHOD(m_class, m_name) ClassDB::bind_method(D_METHOD(#m_name), &m_class::m_name)

#define BIND_PROPERTY(m_name, m_type)  \
	ClassDB::add_property(             \
		get_class_static(),            \
		PropertyInfo(m_type, #m_name), \
		"set_" #m_name,                \
		"get_" #m_name                 \
	)

#define BIND_PROPERTY_ENUM(m_name, m_hint_str)                               \
	ClassDB::add_property(                                                   \
		get_class_static(),                                                  \
		PropertyInfo(Variant::INT, #m_name, PROPERTY_HINT_ENUM, m_hint_str), \
		"set_" #m_name,                                                      \
		"get_" #m_name                                                       \
	)
