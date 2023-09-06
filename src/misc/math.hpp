#pragma once

// NOLINTBEGIN(readability-identifier-naming)

#define Mathf_SQRT12 ((float)Math_SQRT12)
#define Mathf_SQRT2 ((float)Math_SQRT2)
#define Mathf_LN2 ((float)Math_LN2)
#define Mathf_PI ((float)Math_PI)
#define Mathf_TAU ((float)Math_TAU)
#define Mathf_E ((float)Math_E)
#define Mathf_INF ((float)Math_INF)
#define Mathf_NAN ((float)Math_NAN)

// NOLINTEND(readability-identifier-naming)

#define USEC_TO_SEC(m_usec) (double(m_usec) / 1000000.0)

namespace godot::Math {

_FORCE_INLINE_ void decompose(Basis& p_basis, Vector3& p_scale) {
	p_scale = p_basis.get_scale();

	if (p_scale == Vector3(1.0f, 1.0f, 1.0f)) {
		return;
	}

	Vector3 x = p_basis.get_column(Vector3::AXIS_X);
	Vector3 y = p_basis.get_column(Vector3::AXIS_Y);
	Vector3 z = p_basis.get_column(Vector3::AXIS_Z);

	x /= p_scale.x;
	y -= x * x.dot(y);
	y /= p_scale.y;
	z -= x * x.dot(z) - y * y.dot(z);
	z /= p_scale.z;

	p_basis.set_column(Vector3::AXIS_X, x);
	p_basis.set_column(Vector3::AXIS_Y, y);
	p_basis.set_column(Vector3::AXIS_Z, z);
}

_FORCE_INLINE_ void decompose(Transform3D& p_transform, Vector3& p_scale) {
	decompose(p_transform.basis, p_scale);
}

_FORCE_INLINE_ Basis decomposed(Basis p_basis, Vector3& p_scale) {
	decompose(p_basis, p_scale);
	return p_basis;
}

_FORCE_INLINE_ Transform3D decomposed(Transform3D p_transform, Vector3& p_scale) {
	decompose(p_transform, p_scale);
	return p_transform;
}

} // namespace godot::Math
