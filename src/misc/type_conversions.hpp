#pragma once

_FORCE_INLINE_ Vector3 to_godot(const JPH::Vec3& p_vec) {
	return {p_vec.GetX(), p_vec.GetY(), p_vec.GetZ()};
}

_FORCE_INLINE_ Basis to_godot(const JPH::Quat& p_quat) {
	return {Quaternion(p_quat.GetX(), p_quat.GetY(), p_quat.GetZ(), p_quat.GetW())};
}

_FORCE_INLINE_ Color to_godot(const JPH::Color& p_color) {
	const auto r = (float)p_color.r;
	const auto g = (float)p_color.g;
	const auto b = (float)p_color.b;
	const auto a = (float)p_color.a;

	return {
		r == 0.0f ? 0.0f : 255.0f / r,
		g == 0.0f ? 0.0f : 255.0f / g,
		b == 0.0f ? 0.0f : 255.0f / b,
		a == 0.0f ? 0.0f : 255.0f / a};
}

_FORCE_INLINE_ String to_godot(const JPH::String& p_str) {
	return String::utf8(p_str.c_str(), (int32_t)p_str.length());
}

_FORCE_INLINE_ JPH::Vec3 to_jolt(const Vector3& p_vec) {
	return {p_vec.x, p_vec.y, p_vec.z};
}

_FORCE_INLINE_ JPH::Quat to_jolt(const Basis& p_basis) {
	const Quaternion quat = p_basis.get_quaternion();
	return {quat.x, quat.y, quat.z, quat.w};
}

_FORCE_INLINE_ JPH::Color to_jolt(const Color& p_color) {
	return JPH::Color((JPH::uint32)p_color.to_abgr32());
}

_FORCE_INLINE_ JPH::String to_jolt(const String& p_str) {
	const CharString str_utf8 = p_str.utf8();
	return {str_utf8.get_data(), (size_t)str_utf8.length()};
}
