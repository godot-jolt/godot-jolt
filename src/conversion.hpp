#pragma once

_ALWAYS_INLINE_ Vector3 to_godot(const JPH::Vec3& p_vec) {
	return {p_vec.GetX(), p_vec.GetY(), p_vec.GetZ()};
}

_ALWAYS_INLINE_ Basis to_godot(const JPH::Quat& p_quat) {
	return {Quaternion(p_quat.GetX(), p_quat.GetY(), p_quat.GetZ(), p_quat.GetW())};
}

_ALWAYS_INLINE_ JPH::Vec3 to_jolt(const Vector3& p_vec) {
	return {p_vec.x, p_vec.y, p_vec.z};
}

_ALWAYS_INLINE_ JPH::Quat to_jolt(const Basis& p_basis) {
	const Quaternion quat = p_basis.get_quaternion();
	return {quat.x, quat.y, quat.z, quat.w};
}
