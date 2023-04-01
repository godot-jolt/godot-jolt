#pragma once

namespace godot::Math {

_FORCE_INLINE_ void normalize(Basis& p_basis, Vector3& p_scale) {
	p_scale = Vector3(1.0f, 1.0f, 1.0f);

	Vector3 x = p_basis.get_column(Vector3::AXIS_X);
	Vector3 y = p_basis.get_column(Vector3::AXIS_Y);
	Vector3 z = p_basis.get_column(Vector3::AXIS_Z);

	const Vector3 scale_squared(x.length_squared(), y.length_squared(), z.length_squared());

	if (p_scale == scale_squared) {
		return;
	}

	p_scale = Vector3(
		godot::Math::sqrt(scale_squared.x),
		godot::Math::sqrt(scale_squared.y),
		godot::Math::sqrt(scale_squared.z)
	);

	x /= p_scale.x;
	y /= p_scale.y;
	z /= p_scale.z;

	p_basis.set_column(Vector3::AXIS_X, x);
	p_basis.set_column(Vector3::AXIS_Y, y);
	p_basis.set_column(Vector3::AXIS_Z, z);
}

_FORCE_INLINE_ void normalize(Transform3D& p_transform, Vector3& p_scale) {
	normalize(p_transform.basis, p_scale);
}

_FORCE_INLINE_ void normalize(Basis& p_basis) {
	Vector3 scale;
	normalize(p_basis, scale);
}

_FORCE_INLINE_ void normalize(Transform3D& p_transform) {
	Vector3 scale;
	normalize(p_transform, scale);
}

[[nodiscard]] _FORCE_INLINE_ Basis normalized(Basis p_basis, Vector3& p_scale) {
	normalize(p_basis, p_scale);
	return p_basis;
}

[[nodiscard]] _FORCE_INLINE_ Transform3D normalized(Transform3D p_transform, Vector3& p_scale) {
	normalize(p_transform, p_scale);
	return p_transform;
}

[[nodiscard]] _FORCE_INLINE_ Basis normalized(Basis p_basis) {
	normalize(p_basis);
	return p_basis;
}

[[nodiscard]] _FORCE_INLINE_ Transform3D normalized(Transform3D p_transform) {
	normalize(p_transform);
	return p_transform;
}

} // namespace godot::Math
