#pragma once

template<>
struct fmt::formatter<String> : public fmt::formatter<std::string_view> {
	template<typename TContext>
	auto format(const String& p_str, TContext& p_ctx) const {
		const CharString utf8_str = p_str.utf8();
		const std::string_view utf8_view(utf8_str.get_data(), (size_t)utf8_str.length() - 1);
		return fmt::formatter<std::string_view>::format(utf8_view, p_ctx);
	}
};

template<>
struct fmt::formatter<Variant> : public fmt::formatter<String> {
	template<typename TContext>
	auto format(const Variant& p_variant, TContext& p_ctx) const {
		return fmt::formatter<String>::format(p_variant.stringify(), p_ctx);
	}
};

#define GDJOLT_DEFINE_FORMATTER(m_type)                              \
	template<>                                                       \
	struct fmt::formatter<m_type> : public fmt::formatter<Variant> { \
		template<typename TContext>                                  \
		auto format(const m_type& p_value, TContext& p_ctx) const {  \
			return fmt::formatter<Variant>::format(p_value, p_ctx);  \
		}                                                            \
	};

GDJOLT_DEFINE_FORMATTER(Vector2);
GDJOLT_DEFINE_FORMATTER(Vector2i);
GDJOLT_DEFINE_FORMATTER(Rect2);
GDJOLT_DEFINE_FORMATTER(Rect2i);
GDJOLT_DEFINE_FORMATTER(Vector3);
GDJOLT_DEFINE_FORMATTER(Vector3i);
GDJOLT_DEFINE_FORMATTER(Vector4);
GDJOLT_DEFINE_FORMATTER(Vector4i);
GDJOLT_DEFINE_FORMATTER(Plane);
GDJOLT_DEFINE_FORMATTER(Quaternion);
GDJOLT_DEFINE_FORMATTER(AABB);
GDJOLT_DEFINE_FORMATTER(Basis);
GDJOLT_DEFINE_FORMATTER(Transform3D);
GDJOLT_DEFINE_FORMATTER(Projection);
GDJOLT_DEFINE_FORMATTER(Color);
GDJOLT_DEFINE_FORMATTER(StringName);
GDJOLT_DEFINE_FORMATTER(NodePath);
GDJOLT_DEFINE_FORMATTER(RID);
GDJOLT_DEFINE_FORMATTER(Callable);
GDJOLT_DEFINE_FORMATTER(Signal);
GDJOLT_DEFINE_FORMATTER(Dictionary);
GDJOLT_DEFINE_FORMATTER(Array);
GDJOLT_DEFINE_FORMATTER(PackedByteArray);
GDJOLT_DEFINE_FORMATTER(PackedInt32Array);
GDJOLT_DEFINE_FORMATTER(PackedInt64Array);
GDJOLT_DEFINE_FORMATTER(PackedFloat32Array);
GDJOLT_DEFINE_FORMATTER(PackedFloat64Array);
GDJOLT_DEFINE_FORMATTER(PackedStringArray);
GDJOLT_DEFINE_FORMATTER(PackedVector2Array);
GDJOLT_DEFINE_FORMATTER(PackedVector3Array);
GDJOLT_DEFINE_FORMATTER(PackedColorArray);

template<typename... TArgs>
String vformat(fmt::format_string<TArgs...> p_format, TArgs&&... p_args) {
	return String(fmt::format(p_format, std::forward<TArgs>(p_args)...).c_str());
}
